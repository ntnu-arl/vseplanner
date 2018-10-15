/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
* Modified by Tung Dang, University of Nevada, Reno.
* The provided code is an implementation of the visual saliency-aware
* exploration algorithm.
*/

#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_

#include <cstdlib>
#include <random>
#include <vseplanner/rrt.h>
#include <vseplanner/tree.hpp>

////////////////// MATLAB ///////////////////
#include <heading_sample.h>
#include <heading_sample_emxAPI.h>
#include <heading_sample_initialize.h>
#include <heading_sample_terminate.h>
#include <rt_nonfinite.h>

static void argInit_d50x1_real_T(double result_data[], int result_size[1]);
static double argInit_real_T();
// Function Definitions
static void argInit_d50x1_real_T(double result_data[], int result_size[1]) {
  int idx0;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 2;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 2; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result_data[idx0] = argInit_real_T();
  }
}

static double argInit_real_T() { return 0.0; }

vsExploration::RrtTree::RrtTree()
    : vsExploration::TreeBase<StateVec>::TreeBase() {
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(ros::Time::now().toSec());
  }

  // If logging is required, set up files here
  bool ifLog = false;
  std::string ns = ros::this_node::getName();
  ros::param::get(ns + "/vsep/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm *ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ =
        ros::package::getPath("vseplanner") + "/data/" +
        std::to_string(ptm->tm_year + 1900) + "_" +
        std::to_string(ptm->tm_mon + 1) + "_" + std::to_string(ptm->tm_mday) +
        "_" + std::to_string(ptm->tm_hour) + "_" + std::to_string(ptm->tm_min) +
        "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
  bestBranchMemory_.clear();
  erate_buf_.clear();
  cam_model_ready_ = false;
}

vsExploration::RrtTree::RrtTree(mesh::StlMesh *mesh,
                                volumetric_mapping::OctomapManager *manager) {
  mesh_ = mesh;
  manager_ = manager;
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(ros::Time::now().toSec());
  }

  // If logging is required, set up files here
  bool ifLog = false;
  std::string ns = ros::this_node::getName();
  ros::param::get(ns + "/vsep/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm *ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ =
        ros::package::getPath("vseplanner") + "/data/" +
        std::to_string(ptm->tm_year + 1900) + "_" +
        std::to_string(ptm->tm_mon + 1) + "_" + std::to_string(ptm->tm_mday) +
        "_" + std::to_string(ptm->tm_hour) + "_" + std::to_string(ptm->tm_min) +
        "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
  bestBranchMemory_.clear();
  erate_buf_.clear();
  cam_model_ready_ = false;
}

vsExploration::RrtTree::~RrtTree() {
  delete rootNode_;
  kd_free(kdTree_);
  if (fileResponse_.is_open()) {
    fileResponse_.close();
  }
  if (fileTree_.is_open()) {
    fileTree_.close();
  }
  if (filePath_.is_open()) {
    filePath_.close();
  }
}

void vsExploration::RrtTree::setStateFromPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id,
                             pose.header.stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[0] >
      params_.inspection_throttle_) {
    inspectionThrottleTime_[0] += params_.inspection_throttle_;
    if (mesh_) {
      /*geometry_msgs::Pose poseTransformed;
      tf::poseTFToMsg(transform * poseTF, poseTransformed);
      mesh_->setPeerPose(poseTransformed, 0);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
      // Publish the mesh marker for visualization in rviz
      visualization_msgs::Marker inspected;
      inspected.ns = "meshInspected";
      inspected.id = 0;
      inspected.header.seq = inspected.id;
      inspected.header.stamp = pose.header.stamp;
      inspected.header.frame_id = params_.navigationFrame_;
      inspected.type = visualization_msgs::Marker::TRIANGLE_LIST;
      inspected.lifetime = ros::Duration(10);
      inspected.action = visualization_msgs::Marker::ADD;
      inspected.pose.position.x = 0.0;
      inspected.pose.position.y = 0.0;
      inspected.pose.position.z = 0.0;
      inspected.pose.orientation.x = 0.0;
      inspected.pose.orientation.y = 0.0;
      inspected.pose.orientation.z = 0.0;
      inspected.pose.orientation.w = 1.0;
      inspected.scale.x = 1.0;
      inspected.scale.y = 1.0;
      inspected.scale.z = 1.0;
      visualization_msgs::Marker uninspected = inspected;
      uninspected.header.seq++;
      uninspected.id++;
      uninspected.ns = "meshUninspected";
      mesh_->assembleMarkerArray(inspected, uninspected);
      if (inspected.points.size() > 0) {
        params_.inspectionPath_.publish(inspected);
      }
      if (uninspected.points.size() > 0) {
        params_.inspectionPath_.publish(uninspected);
      }*/
      ROS_WARN_THROTTLE(
          1.0, "planner<set_state_from_pose_msg>: Mesh case unimplemented!!!");
    }
  }
}

void vsExploration::RrtTree::setStateFromOdometryMsg(
    const nav_msgs::Odometry &pose) {
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id,
                             pose.header.stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[0] >
      params_.inspection_throttle_) {
    inspectionThrottleTime_[0] += params_.inspection_throttle_;
    if (mesh_) {
      /*geometry_msgs::Pose poseTransformed;
      tf::poseTFToMsg(transform * poseTF, poseTransformed);
      mesh_->setPeerPose(poseTransformed, 0);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
      // Publish the mesh marker for visualization in rviz
      visualization_msgs::Marker inspected;
      inspected.ns = "meshInspected";
      inspected.id = 0;
      inspected.header.seq = inspected.id;
      inspected.header.stamp = pose.header.stamp;
      inspected.header.frame_id = params_.navigationFrame_;
      inspected.type = visualization_msgs::Marker::TRIANGLE_LIST;
      inspected.lifetime = ros::Duration(10);
      inspected.action = visualization_msgs::Marker::ADD;
      inspected.pose.position.x = 0.0;
      inspected.pose.position.y = 0.0;
      inspected.pose.position.z = 0.0;
      inspected.pose.orientation.x = 0.0;
      inspected.pose.orientation.y = 0.0;
      inspected.pose.orientation.z = 0.0;
      inspected.pose.orientation.w = 1.0;
      inspected.scale.x = 1.0;
      inspected.scale.y = 1.0;
      inspected.scale.z = 1.0;
      visualization_msgs::Marker uninspected = inspected;
      uninspected.header.seq++;
      uninspected.id++;
      uninspected.ns = "meshUninspected";
      mesh_->assembleMarkerArray(inspected, uninspected);
      if (inspected.points.size() > 0) {
        params_.inspectionPath_.publish(inspected);
      }
      if (uninspected.points.size() > 0) {
        params_.inspectionPath_.publish(uninspected);
      }*/
      ROS_WARN_THROTTLE(
          1.0,
          "planner<set_state_from_odometry_msg>: Mesh case unimplemented!!!");
    }
  }
}

void vsExploration::RrtTree::setPeerStateFromPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped &pose, int n_peer) {
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id,
                             pose.header.stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  geometry_msgs::Pose poseTransformed;
  tf::poseTFToMsg(transform * poseTF, poseTransformed);
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[n_peer] >
      params_.inspection_throttle_) {
    inspectionThrottleTime_[n_peer] += params_.inspection_throttle_;
    if (mesh_) {
      mesh_->setPeerPose(poseTransformed, n_peer);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, n_peer);
    }
  }
}

bool vsExploration::RrtTree::resampleFirstVertex(int numRuns) {

  vsExploration::Node<StateVec> *targetNode = bestNode_;
  if (targetNode->parent_ == NULL)
    return false;

  while (targetNode->parent_ != rootNode_ && targetNode->parent_ != NULL) {
    targetNode = targetNode->parent_;
  }
  vsExploration::Node<StateVec> *sourceNode = targetNode->parent_;

  StateVec targetState = targetNode->state_;
  double targetState_originalYaw = targetState[3];

  // std::cout << targetNode->state_[3]/0.017453293 << " Original Gain: " <<
  // targetNode->gain_ << std::endl;

  bool reOrient_success = false;
  const int it_reOrient = 8;
  const double disc_reOrient = (2.0 * M_PI) / it_reOrient;
  for (int i = -it_reOrient / 2; i < it_reOrient / 2;
       ++i) { // TODO: start from i=1, original gain is already known, no need
              // for recalculation
    targetState[3] = targetState_originalYaw + disc_reOrient * i;
    // std::cout << targetState[3]/0.017453293;
    if (targetState[3] > M_PI)
      targetState[3] -= 2.0 * M_PI;
    else if (targetState[3] < -M_PI)
      targetState[3] += 2.0 * M_PI;

    auto targetNodeGain = gain(targetState);
    double target_node_gain;
    if (numRuns < params_.degressiveSwitchoffLoops_)
      target_node_gain =
          sourceNode->gain_ +
          (targetNodeGain)*exp(
              -params_.degressiveCoeff_ *
              targetNode->distance_); // limit behavior to initial runs
    else
      target_node_gain = sourceNode->gain_ + (targetNodeGain);

    // std::cout << "gain: " << target_node_gain << std::endl;
    if (target_node_gain > targetNode->gain_) {
      reOrient_success = true;
      targetNode->state_ = targetState;
      targetNode->gain_ = target_node_gain;
    }
  }

  if (reOrient_success) {
    rePublishNode(targetNode, vsExploration::NBVP_PLANLEVEL);
  }

  return reOrient_success;
}

void vsExploration::RrtTree::rePublishNode(
    Node<StateVec> *node, vsExploration::PlanningLevel planninglevel,
    int nodeorder) {
  const float eps = 0.001; // std::numeric_limits<float>::epsilon();
  for (std::vector<visualization_msgs::Marker>::iterator it =
           params_.planningPath_markers_.markers.begin();
       it != params_.planningPath_markers_.markers.end(); ++it) {
    if (it->ns == "vp_orientations" &&
        std::abs(it->pose.position.x - node->state_[0]) <= eps &&
        std::abs(it->pose.position.y - node->state_[1]) <= eps &&
        std::abs(it->pose.position.z - node->state_[2]) <= eps) {
      // tf::Quaternion
      // quat_original(it->pose.orientation.x,it->pose.orientation.y,it->pose.orientation.z,it->pose.orientation.w);
      // tfScalar roll_original, pitch_original, yaw_original;
      // tf::Matrix3x3(quat_original).getRPY(roll_original, pitch_original,
      // yaw_original);
      // std::cout<< "Modifying Node [" << node->state_[0] << "," <<
      // node->state_[1] << "," << node->state_[2] << "]: "<<
      // yaw_original/0.017453293 << "->" << node->state_[3]/0.017453293 <<
      // std::endl;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, node->state_[3]);
      it->pose.orientation.x = quat.x();
      it->pose.orientation.y = quat.y();
      it->pose.orientation.z = quat.z();
      it->pose.orientation.w = quat.w();
      break;
    }
  }
}

void vsExploration::RrtTree::iterate(int numRuns, int plannerMode) {
  // In this function a new configuration is sampled and added to the tree.
  StateVec newState;

  // Sample over a sphere with the radius of the maximum diagonal of the
  // exploration space.
  // Throw away samples outside the sampling region it exiting is not allowed
  // by the corresponding parameter. This method is to not bias the tree towards
  // the center of the exploration space.
  double radius = sqrt(SQ(params_.minX_ - params_.maxX_) +
                       SQ(params_.minY_ - params_.maxY_) +
                       SQ(params_.minZ_ - params_.maxZ_));
  bool solutionFound_pos = false;
  int whileThres_pos = 10000;
  while (!solutionFound_pos && whileThres_pos--) {
    for (int i = 0; i < 3; i++) {
      newState[i] =
          2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
      continue;
    // Offset new state by root
    newState += rootNode_->state_;
    if (!params_.softBounds_) {
      if (newState.x() + params_.boundingBoxOffset_.x() <
          params_.minX_ + 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() + params_.boundingBoxOffset_.y() <
                 params_.minY_ + 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() + params_.boundingBoxOffset_.z() <
                 params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
        continue;
      } else if (newState.x() + params_.boundingBoxOffset_.x() >
                 params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() + params_.boundingBoxOffset_.y() >
                 params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() + params_.boundingBoxOffset_.z() >
                 params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
        continue;
      }
    }
    // Sample ONLY in FREE MAPPED space
    if (volumetric_mapping::OctomapManager::CellStatus::kFree !=
        manager_->getCellStatusBoundingBox(
            Eigen::Vector3d(newState[0], newState[1], newState[2]) +
                params_.boundingBoxOffset_,
            params_.boundingBox_))
      continue;
    solutionFound_pos = true;
  }
  if (!solutionFound_pos) {
    return;
  }

  // Find nearest neighbour
  kdres *nearest =
      kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }

  vsExploration::Node<StateVec> *newParent =
      (vsExploration::Node<StateVec> *)kd_res_item_data(nearest);
  kd_res_free(nearest);

  // Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1],
                         newParent->state_[2]);
  Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                            newState[2] - origin[2]);
  if (direction.norm() > params_.extensionRange_) {
    direction = params_.extensionRange_ * direction.normalized();
  }
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];
  if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
      manager_->getLineStatusBoundingBox(
          origin + params_.boundingBoxOffset_,
          origin + params_.boundingBoxOffset_ + direction +
              direction.normalized() * params_.dOvershoot_,
          params_.boundingBox_)) {
    // Sample the new orientation
    newState[3] = params_.yaw_sampling_limit_ *
                  (((double)rand()) / ((double)RAND_MAX) - 0.5); // 2.0 * M_PI
    // Create new node and insert into tree
    vsExploration::Node<StateVec> *newNode = new vsExploration::Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newNode->id_ = counter_ + 1; //0 is root node
    newNode->children_.clear();
    newParent->children_.push_back(newNode);

    if (plannerMode == 2)
      ROS_INFO("Add a node with [%d] to parents [%d] with [%d] children",
             newNode->id_, newParent->id_, newParent->children_.size());

    double new_gain = 0;
    // JAH: planner_mode: switch
    if (plannerMode == 2) {
      // To be calculated after forming the whole tree
      newNode->marker_ = 0;
      // new_gain = curiousGain(newNode->state_);
      // newNode->gain_ = newParent->gain_ +
      //     new_gain * exp(-params_.degressiveCoeff_ * newNode->distance_);
    } else {
      new_gain = gain(newNode->state_);
      if (numRuns < params_.degressiveSwitchoffLoops_) {
        newNode->gain_ =
            newParent->gain_ +
            new_gain * exp(-params_.degressiveCoeff_ * newNode->distance_);
      } else {
        newNode->gain_ = newParent->gain_ + new_gain;
      }
    }
    kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

    // Display new node
    // publishNode(newNode);
    if (plannerMode == 2)
      publishNode(newNode, vsExploration::SAL_PLANLEVEL);
    else
      publishNode(newNode, vsExploration::NBVP_PLANLEVEL);

    // Update best IG and node if applicable
    if (newNode->gain_ > bestGain_) {
      bestGain_ = newNode->gain_;
      bestNode_ = newNode;
    }
    counter_++;

    if (new_gain > bestSingleGain_) {
      bestSingleGain_ = new_gain;
    }
  }
}

void vsExploration::RrtTree::initialize(int numRuns) {
  // This function is to initialize the tree, including insertion of remainder
  // of previous best branch.
  g_ID_ = 0;
  g_ID_S_ = 0;
  g_ID_r_ = 0;
  g_ID_Sr_ = 0;
  n_ID_ = 0;
  n_ID_r_ = 0;

  bestSingleGain_ = 0;

  // Initialize kd-tree with root node and prepare log file
  kdTree_ = kd_create(3);
  kdSubTree_ = kd_create(3);

  if (params_.log_) {
    if (fileTree_.is_open()) {
      fileTree_.close();
    }
    fileTree_.open(
        (logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt")
            .c_str(),
        std::ios::out);
  }

  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = NULL;
  rootNode_->id_ = 0;
  rootNode_->marker_ = 0;

  if (params_.exact_root_) {
    if (iterationCount_ <= 1) {
      exact_root_ = root_;
    }
    rootNode_->state_ = exact_root_;
  } else {
    rootNode_->state_ = root_;
  }
  kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(),
             rootNode_->state_.z(), rootNode_);
  iterationCount_++;

  // Insert all nodes of the remainder of the previous best branch, checking for
  // collisions and recomputing the gain.
  for (typename std::vector<StateVec>::reverse_iterator iter =
           bestBranchMemory_.rbegin();
       iter != bestBranchMemory_.rend(); ++iter) {
    StateVec newState = *iter;
    kdres *nearest =
        kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      continue;
    }
    vsExploration::Node<StateVec> *newParent =
        (vsExploration::Node<StateVec> *)kd_res_item_data(nearest);
    kd_res_free(nearest);

    // Check for collision
    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1],
                           newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                              newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_) {
      direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];
    if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
        manager_->getLineStatusBoundingBox(
            origin + params_.boundingBoxOffset_,
            origin + params_.boundingBoxOffset_ + direction +
                direction.normalized() * params_.dOvershoot_,
            params_.boundingBox_)) {
      // Create new node and insert into tree
      vsExploration::Node<StateVec> *newNode =
          new vsExploration::Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      double new_gain = 0;
      new_gain = gain(newNode->state_);
      if (numRuns < params_.degressiveSwitchoffLoops_) {
        newNode->gain_ =
            newParent->gain_ +
            new_gain * exp(-params_.degressiveCoeff_ * newNode->distance_);
      } else {
        newNode->gain_ = newParent->gain_ + new_gain;
      }
      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      // Display new node
      publishNode(newNode, vsExploration::NBVP_PLANLEVEL);
      // publishNode(newNode);

      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
  }

  // Publish visualization of total exploration area
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
  p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
  p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = params_.maxX_ - params_.minX_;
  p.scale.y = params_.maxY_ - params_.minY_;
  p.scale.z = params_.maxZ_ - params_.minZ_;
  p.color.r = 200.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 0.25;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

bool vsExploration::RrtTree::resampleBestEdge(double ext_ratio) {
  // double ext_ratio = 0;
  // plannerEvaluate(ext_ratio);

  // if ((params_.planner_state_ == PSTATE_EXPL_ONLY) ||
  //     (params_.planner_state_ == PSTATE_EXPL_SAL)){
  if (params_.planner_state_ == PSTATE_EXPL_ONLY) {
    ROS_INFO("Skip the 2nd layer");
    return false;
  }

  vsExploration::Node<StateVec> *targetNode = bestNode_;
  if (targetNode->parent_ == NULL)
    return false;

  while (targetNode->parent_ != rootNode_ && targetNode->parent_ != NULL) {
    targetNode = targetNode->parent_;
  }
  vsExploration::Node<StateVec> *sourceNode = targetNode->parent_;

  double dist = targetNode->distance_ - sourceNode->distance_;
  double dyaw = targetNode->state_[3] - sourceNode->state_[3];
  if (dyaw < -M_PI)
    dyaw += 2 * M_PI;
  else if (dyaw > M_PI)
    dyaw -= 2 * M_PI;
  dyaw = abs(dyaw);

  // dt is cancelled out
  double t_exec = std::max(dyaw / params_.dyaw_max_, dist / params_.v_max_);
  double dist_allow = t_exec * params_.v_max_;

  ext_ratio = ext_ratio * dist_allow / dist;
  ext_ratio = (ext_ratio <= params_.extend_ratio_max_)
                  ? ext_ratio
                  : params_.extend_ratio_max_;

  ROS_INFO("Extend ratio (after checking the time budget) : %f", ext_ratio);
  return connect(sourceNode, targetNode, ext_ratio);
}

bool vsExploration::RrtTree::moveToNextNode(Node<StateVec> * &node) {
  int num_child = node->children_.size();
  if (num_child > 0) {
    ROS_INFO("Has [%d] children, should go deeper", num_child);
    // Has children, should go deeper
    while (num_child > 0) {
      num_child--;
      if (node->children_[num_child]->marker_ == 0) {
        node = node->children_[num_child];
        ROS_INFO("Pick children at [%d] with id", num_child, (int)node->id_);
        return true;
      }
    }
  }

  // All children are explored OR no children,
  // Mark this as an explored node
  // Back to parent
  ROS_INFO("No children or leaf node");
  node->marker_ = 1;
  if (node->parent_ != NULL) {
    node = node->parent_;
    return true;
  } else {
    return false;
  }
}

bool vsExploration::RrtTree::computeBestEntropyBranch() {
  // From the root node
  // Trace until the end of each branch
  // Then cmpute D-entropy for each branch
  float best_e_gain = 0;
  vsExploration::Node<StateVec> *best_e_node = rootNode_;
  vsExploration::Node<StateVec> *node = rootNode_;

  // Scan the whole tree
  int count = 0;
  while (moveToNextNode(node) && (count++ < 100)) {
    ROS_INFO("At node [%d]", node->id_);
    if (node->children_.size() == 0) {
      // End node
      // Computer the entropy_current
      ROS_INFO("Found an end node");
      float gain_tmp = entropyGain(node);
      if (gain_tmp > best_e_gain) {
        best_e_gain = gain_tmp;
        best_e_node = node;
      }
    }
  }

  ROS_INFO("Best gain: %f", best_e_gain);
  if (best_e_gain > 0) {
    bestGain_ = best_e_gain;
    bestNode_ = best_e_node;
    return true;
  } else {
    return false;
  }
}

void vsExploration::RrtTree::plannerEvaluate(double &extend_ratio) {
  /*------------------------------------
  * Compute the acceptable extend_ratio (testing)
  -----------------------------------*/
  extend_ratio = params_.extend_ratio_fixed_;
  // double per = manager_->getVolumePercentage(bestSingleGain_);
  // ROS_INFO("NBVP max single gain: %f", per);

  if (params_.extend_enable_) { // compute extension from the exploration rate
    extend_ratio = 1.0;         // default number
    // trigger the exploration rate computation
    double exp_rate[3] = {0, 0, 0};
    manager_->getExplorationRate(exp_rate);

    double t_require = 0;
    double t_remain = 0;
    double t_budget = params_.time_budget_;
    double e_percentage = exp_rate[0];
    double e_rate = exp_rate[1];
    double t_passed = exp_rate[2];

    // Low-pass filter
    if (e_rate > 1.0) {
      e_rate = 0; // something wrong
    }
    erate_buf_.push_back(e_rate);
    int len = erate_buf_.size();
    if (len >= params_.exp_filter_window_) {
      e_rate = 0;
      for (int i = 1; i <= params_.exp_filter_window_; i++) {
        e_rate += erate_buf_[len - i];
      }
      e_rate = e_rate / (double)(params_.exp_filter_window_);
      erate_buf_[len - 1] = e_rate; // put back
    }

    ROS_INFO(
        "Exploration state: percentage:%f, rate:%f (after avg: %f), time:%f",
        exp_rate[0], exp_rate[1], e_rate, exp_rate[2]);
    //
    switch (params_.planner_state_) {
    case PSTATE_INITIALIZED:
      ROS_WARN("Switched to PSTATE_EXPL_ONLY");
      params_.planner_state_ = PSTATE_EXPL_ONLY;
      break;
    case PSTATE_EXPL_ONLY:
      if (e_percentage > params_.exp_lower_bound_) {
        params_.planner_state_ = PSTATE_EXPL_FULL;
        ROS_WARN("Switched to PSTATE_EXPL_FULL");
      } else
        return; // do nothing
      break;
    case PSTATE_EXPL_FULL:
      if (e_percentage > params_.exp_upper_bound_) {
        params_.planner_state_ = PSTATE_EXPL_SAL;
        ROS_WARN("Switched to PSTATE_EXPL_SAL");
      }
      break;
    case PSTATE_EXPL_SAL:
      break;
    default:
      params_.planner_state_ = PSTATE_EXPL_ONLY;
      break;
    }

    if (e_rate) {
      t_require = (1 - e_percentage) / e_rate;
    } else {
      t_require = t_budget; // require all budget
    }
    ROS_INFO("Min required time  : %f", t_require);
    t_remain = t_budget - t_passed;
    ROS_INFO("Time remaining     : %f", t_remain);

    if (t_remain <= 0) {
      ROS_WARN("Out of time budget  : STOP ?");
      ROS_WARN("Out of time budget...");
      // return false;
    } else if (t_remain > t_require) {
      extend_ratio = 1.0 + (t_remain - t_require) / t_require;
    }
    extend_ratio = (extend_ratio <= params_.extend_ratio_max_)
                       ? extend_ratio
                       : params_.extend_ratio_max_; // limit at 2.0

    // LOG FILE FOR evaluation
    int planner_mode = params_.planner_state_;

    if (((params_.cuEnable_) && (t_remain <= 0)) ||
        ((!params_.cuEnable_) && (e_percentage > params_.exp_upper_bound_))) {
      ROS_WARN("Finished. Going to shutdown now....");
      params_.planner_state_ = PSTATE_EXPL_FINISH;
      // ros::shutdown();
    }
  }
  ROS_INFO("Extend ratio       : %f", extend_ratio);
}

void vsExploration::RrtTree::setHardTarget(std::vector<double> val) {
  vsExploration::Node<StateVec> *targetNode = new vsExploration::Node<StateVec>;

  targetNode->parent_ = rootNode_;
  targetNode->gain_ = 0;
  targetNode->distance_ = 0;
  targetNode->children_.clear();
  targetNode->state_[0] = val[0];
  targetNode->state_[1] = val[1];
  targetNode->state_[2] = val[2];
  targetNode->state_[3] = val[3];
  rootNode_->children_.push_back(targetNode);
  publishNode(targetNode, vsExploration::NBVP_PLANLEVEL);

  bestNode_ = targetNode;
}
int vsExploration::RrtTree::getPlannerState(void) {
  return params_.planner_state_;
}

bool vsExploration::RrtTree::getBestVertex(std::vector<double> &vertex) {
  vsExploration::Node<StateVec> *targetNode;
  targetNode = bestNode_;
  if (targetNode->parent_ == NULL)
    return false;

  while (targetNode->parent_ != rootNode_ && targetNode->parent_ != NULL) {
    targetNode = targetNode->parent_;
  }
  vertex.push_back(targetNode->state_[0]);
  vertex.push_back(targetNode->state_[1]);
  vertex.push_back(targetNode->state_[2]);
  vertex.push_back(targetNode->state_[3]);
  return true;
}

// // span a tree to search for feasible paths
// bool vsExploration::RrtTree::connect(vsExploration::Node<StateVec>
// *sourceNode,
//                                      vsExploration::Node<StateVec>
//                                      *targetNode,
//                                      double extend_ratio){
//   //ROS_WARN("DEBUG: %f %f %f %f",  sourceNode->distance_, sourceNode->gain_,
//   targetNode->distance_, targetNode->gain_);
//   /*--------------------
//   * Database
//   ---------------------*/
//   // Re-intialize a root node
//   sourceNode->parent_ = NULL;
//   sourceNode->gain_ = 0;
//   sourceNode->distance_ = 0;
//   sourceNode->children_.clear();
//   // Endpoints of feasible branches
//   std::vector< vsExploration::Node<StateVec>* > feasible_branch_ep;
//   feasible_branch_ep.clear();
//   // Tree data structure
//   kdSubTree_ = kd_create(3);
//   // Add first node to the tree
//   kd_insert3(kdSubTree_, sourceNode->state_.x(),
//                          sourceNode->state_.y(),
//                          sourceNode->state_.z(), sourceNode);
//
//   /*---------------------------------
//   * Initialize parameters for a tree
//   ----------------------------------*/
//   // Sampling space in the ellipse
//   Eigen::Vector3f shortest_path(targetNode->state_.x() -
//   sourceNode->state_.x(),
//                                 targetNode->state_.y() -
//                                 sourceNode->state_.y(),
//                                 targetNode->state_.z() -
//                                 sourceNode->state_.z());
//   Eigen::Vector3f
//   ellipse_center((targetNode->state_.x()+sourceNode->state_.x())/2.0,
//                                  (targetNode->state_.y()+sourceNode->state_.y())/2.0,
//                                  (targetNode->state_.z()+sourceNode->state_.z())/2.0);
//   double shortest_dist = shortest_path.norm();
//   float dist_limit = extend_ratio * shortest_dist;
//   if (dist_limit < params_.vseReplanningDistanceMin_) {
//     ROS_WARN("Too short for 2nd layer, just ignore: %f(m)", dist_limit);
//     return false;
//   }
//
//   Eigen::Vector3f ellipse_radius(0.5 * shortest_dist * extend_ratio,
//                                  0.5 * shortest_dist * sqrt(extend_ratio *
//                                  extend_ratio - 1),
//                                  0.5 * shortest_dist * sqrt(extend_ratio *
//                                  extend_ratio - 1) ); // x: r*d/2, y,z:
//                                  d/2*sqrt(r^2-1)
//   tf::Vector3 tf_e_center(ellipse_center.x(), ellipse_center.y(),
//   ellipse_center.z());
//   Eigen::Quaternion<float> q;
//   Eigen::Vector3f init(1.0, 0.0, 0.0); //x-axis
//   Eigen::Vector3f dir(shortest_path.x(), shortest_path.y(),
//   shortest_path.z()); // AB
//   q.setFromTwoVectors(init, dir);
//   q.normalize();
//   tf::Quaternion tf_e_orient(q.x(),q.y(),q.z(),q.w());
//   tf::Transform tf_e_full(tf_e_orient, tf_e_center);
//
//   //Bsp: Publish visualization of replanning ellipsoid
//   visualization_msgs::Marker p;
//   p.header.stamp = ros::Time::now();
//   p.header.seq = 0;
//   p.header.frame_id = params_.navigationFrame_;
//   p.id = 0;
//   p.ns = "exp_workspace";
//   p.type = visualization_msgs::Marker::SPHERE;
//   p.action = visualization_msgs::Marker::ADD;
//   p.pose.position.x = ellipse_center.x();
//   p.pose.position.y = ellipse_center.y();
//   p.pose.position.z = ellipse_center.z();
//   p.pose.orientation.x = tf_e_orient.x();
//   p.pose.orientation.y = tf_e_orient.y();
//   p.pose.orientation.z = tf_e_orient.z();
//   p.pose.orientation.w = tf_e_orient.w();
//   p.scale.x = 2 * ellipse_radius.x();
//   p.scale.y = 2 * ellipse_radius.y();
//   p.scale.z = 2 * ellipse_radius.z();
//   p.color.r = 200.0 / 255.0;
//   p.color.g = 200.0 / 255.0;
//   p.color.b = 0;
//   p.color.a = 0.5;
//   p.lifetime = ros::Duration(10.0);
//   p.frame_locked = false;
//   params_.planningWorkspace_.publish(p);
//
//   // ROS_INFO("Searching path (D:%f/ExtD:%f) between [%f,%f,%f] and
//   [%f,%f,%f]",
//   //                         shortest_path.norm(),
//   //                         dist_limit,
//   //                         sourceNode->state_.x(),
//   //                         sourceNode->state_.y(),
//   //                         sourceNode->state_.z(),
//   //                         targetNode->state_.x(),
//   //                         targetNode->state_.y(),
//   //                         targetNode->state_.z());
//
//   /*-------------------------------
//   * Loop to find feasible branches
//   --------------------------------*/
//   int count_true = 0, count_false = 0;
//   int sampling_node = 0;
//   int node_counter = 0;
//   int feasible_branch_counter = 0;
//   bool stop_spanning_tree = false;
//   while ((!stop_spanning_tree) &&
//          (feasible_branch_counter < params_.branch_max_) &&
//          (node_counter < params_.node_max_)){
//     /*-------------------------------
//     * Sampling a random point
//     --------------------------------*/
//     StateVec rand_state;
//     bool found_rand_state = false;
//     int sampling_count = 0;
//     while (!found_rand_state) {
//       // an ellipse
//       double r_rand = 2 * (((double) rand()) / ((double) RAND_MAX) - 0.5);
//       //[-1, 1]
//       double alpha = M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
//       //[-pi/2, pi/2]
//       double beta = 2 * M_PI * (((double) rand()) / ((double) RAND_MAX) -
//       0.5); //[-pi, pi]
//       rand_state[0] = r_rand * ellipse_radius.x() * cos(alpha) * cos(beta);
//       rand_state[1] = r_rand * ellipse_radius.y() * cos(alpha) * sin(beta);
//       rand_state[2] = r_rand * ellipse_radius.z() * sin(alpha);
//
//       // // a cube
//       // // rand_state[0] = ellipse_radius.x() * (((double) rand()) /
//       ((double) RAND_MAX) - 0.5); //[-r,+r]
//       // // rand_state[1] = ellipse_radius.y() * (((double) rand()) /
//       ((double) RAND_MAX) - 0.5);
//       // // rand_state[2] = ellipse_radius.z() * (((double) rand()) /
//       ((double) RAND_MAX) - 0.5);
//       // // sampling only in the sphere within the extended distance
//       tf::Vector3 state_tf = tf_e_full * tf::Vector3(rand_state[0],
//       rand_state[1], rand_state[2]);
//       rand_state[0] = state_tf.x();
//       rand_state[1] = state_tf.y();
//       rand_state[2] = state_tf.z();
//
//       // // Sampling in the whole bounding box for cases that we need to
//       perform second layer only
//       // for (int i = 0; i < 3; i++) {
//       //   rand_state[i] = ((double) rand()) / ((double) RAND_MAX);
//       // }
//       // rand_state[0] = params_.minX_ + rand_state[0] * (params_.maxX_ -
//       params_.minX_);
//       // rand_state[1] = params_.minY_ + rand_state[1] * (params_.maxY_ -
//       params_.minY_);
//       // rand_state[2] = params_.minZ_ + rand_state[2] * (params_.maxZ_ -
//       params_.minZ_);
//
//       //Compare with the limit distance to remove useless points right in
//       this stage to save time
//       Eigen::Vector3f forward_vec(rand_state[0] - sourceNode->state_[0],
//                                   rand_state[1] - sourceNode->state_[1],
//                                   rand_state[2] - sourceNode->state_[2]);
//       Eigen::Vector3f backward_vec(rand_state[0] - targetNode->state_[0],
//                                   rand_state[1] - targetNode->state_[1],
//                                   rand_state[2] - targetNode->state_[2]);
//       if ((forward_vec.norm() + backward_vec.norm()) > dist_limit){
//         // in the ellipse sampling case, it must always sastisfy, shouldn't
//         reach here
//         count_false++;
//         continue;
//       }else{
//         count_true++;
//       }
//
//       sampling_count++;
//       if (sampling_count > 1000) {
//         break;
//         ROS_WARN("Can not sample any new point --> Stop spanning a tree");
//       }
//
//       // Check inside the bounding box
//       if (!params_.softBounds_) {
//         if (rand_state.x() + params_.boundingBoxOffset_.x()< params_.minX_ +
//         0.5 * params_.boundingBox_.x()) {
//           continue;
//         } else if (rand_state.y() + params_.boundingBoxOffset_.y()<
//         params_.minY_ + 0.5 * params_.boundingBox_.y()) {
//           continue;
//         } else if (rand_state.z() + params_.boundingBoxOffset_.z()<
//         params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
//           continue;
//         } else if (rand_state.x() + params_.boundingBoxOffset_.x()>
//         params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
//           continue;
//         } else if (rand_state.y() + params_.boundingBoxOffset_.y()>
//         params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
//           continue;
//         } else if (rand_state.z() + params_.boundingBoxOffset_.z()>
//         params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
//           continue;
//         }
//       }
//
//       // only sample points in free voxels
//       if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
//         manager_->getCellStatusBoundingBox(Eigen::Vector3d(rand_state[0],
//                                                            rand_state[1],
//                                                            rand_state[2])+params_.boundingBoxOffset_,
//                                                            params_.boundingBox_)){
//         found_rand_state = true;
//         // ROS_INFO("Sample point[%d,%d]: [%f,%f,%f]", sampling_count,
//         sampling_node, newState[0], newState[1], newState[2] );
//       }
//     }
//     //ROS_INFO("RRT number of sampling points True: %d, False: %d",
//     count_true, count_false);
//     if (count_true > 10000){
//       // this is because the robot is outside its workspace and can not find
//       a feasible path to come back
//       // just execute the first layer
//       ROS_WARN("Cannot resample new path. Status: [%d] nodes and [%d]
//       branches", node_counter, feasible_branch_counter);
//       return false;
//     }
//     /*------------------------------------
//     * Add new node from new sampling point
//     -------------------------------------*/
//     if (!found_rand_state){
//       // stop spanning a tree since we can not sample any new point
//       stop_spanning_tree = true;
//     }else{
//       // Find nearest node as parent
//       kdres * nearest = kd_nearest3(kdSubTree_, rand_state.x(),
//       rand_state.y(), rand_state.z());
//       if (kd_res_size(nearest) <= 0) {
//         kd_res_free(nearest);
//         continue;
//       }
//       vsExploration::Node<StateVec> * parent_node = NULL;
//       parent_node = (vsExploration::Node<StateVec> *)
//       kd_res_item_data(nearest);
//       kd_res_free(nearest);
//       Eigen::Vector3d edge(rand_state[0] - parent_node->state_[0],
//                            rand_state[1] - parent_node->state_[1],
//                            rand_state[2] - parent_node->state_[2]);
//       double replanning_extension_range =
//       params_.vseReplanningExtensionRatio_* 2 * ellipse_radius.x();
//       if (edge.norm() > replanning_extension_range) {
//         edge = replanning_extension_range * edge.normalized();
//       }
//
//       // Compute new vertex
//       StateVec vertex;
//       vertex[0] = parent_node->state_[0] + edge[0];
//       vertex[1] = parent_node->state_[1] + edge[1];
//       vertex[2] = parent_node->state_[2] + edge[2];
//       vertex[3] = 0;
//
//       // Verify the total length of this branch
//       Eigen::Vector3f best_edge(vertex[0] - targetNode->state_[0],
//                                 vertex[1] - targetNode->state_[1],
//                                 vertex[2] - targetNode->state_[2]);
//       float dist_vertex2src = parent_node->distance_ + edge.norm();
//       float dist_vertex2tgt = dist_vertex2src + best_edge.norm();
//       if (dist_vertex2tgt > dist_limit){
//         //ROS_WARN("Oop: %f %f %f", newParent->distance_, direction.norm(),
//         forward_dir.norm());
//         continue;
//       }
//
//       // Check collision along this edge
//       Eigen::Vector3d start(parent_node->state_.x(),
//                             parent_node->state_.y(),
//                             parent_node->state_.z());
//       Eigen::Vector3d end = start + edge + edge.normalized() *
//       params_.dOvershoot_;
//       if (volumetric_mapping::OctomapManager::CellStatus::kFree !=
//           manager_->getLineStatusBoundingBox(start+params_.boundingBoxOffset_,
//                                              end+params_.boundingBoxOffset_,
//                                              params_.boundingBox_)) {
//         continue;
//       }
//
//       // Found one more qualified vertex
//       vsExploration::Node<StateVec> * new_node = new
//       vsExploration::Node<StateVec>;
//       new_node->state_ = vertex;
//       new_node->parent_ = parent_node;
//       new_node->distance_ = parent_node->distance_ + edge.norm();
//       new_node->gain_ = -1; // haven't assigned gain
//       parent_node->children_.push_back(new_node); // make a copy
//       node_counter++;
//
//       // Check target reaching
//       Eigen::Vector3d target_radius(targetNode->state_[0]-vertex[0],
//                                     targetNode->state_[1]-vertex[1],
//                                     targetNode->state_[2]-vertex[2]);
//       if (target_radius.norm() < 0.4 ){ //  voxel resolution
//         // Reach target, but have to check the clearance
//         // Check collision along this node to target node
//         Eigen::Vector3d node_start(vertex[0], vertex[1], vertex[2]);
//         Eigen::Vector3d node_end = node_start + target_radius +
//         target_radius.normalized()* params_.dOvershoot_;
//         if (volumetric_mapping::OctomapManager::CellStatus::kFree !=
//             manager_->getLineStatusBoundingBox(node_start+params_.boundingBoxOffset_,
//                                                node_end+params_.boundingBoxOffset_,
//                                                params_.boundingBox_)) {
//           continue;
//         }else{
//           feasible_branch_ep.push_back(new_node);
//           feasible_branch_counter++;
//           //ROS_WARN("RRT: reached target");
//         }
//       }else{
//         kd_insert3(kdSubTree_, vertex.x(), vertex.y(), vertex.z(), new_node);
//       }
//       // Display new node
//       // int nodeorder=0;
//       // publishNode(new_node, vsExploration::BSP_PLANLEVEL, nodeorder);
//       //publishNode(new_node);
//       //publishNode(new_node, vsExploration::BSP_PLANLEVEL);
//     }
//   }
//
//   if (feasible_branch_counter == 0) {
//     ROS_WARN("Can not find any feasible path!");
//     return false;
//   }else{
//     ROS_INFO("Found %d feasible branches", feasible_branch_counter);
//   }
//
//   // ROS_INFO("RRT number of sampling points True: %d, False: %d",
//   count_true, count_false);
//   /*------------------------------------------------
//   * Evaluate the gain information along all branches
//   --------------------------------------------------*/
//   int feasible_branch_valid_counter = 0;
//   int feasible_branch_zero_gain = 0;
//   int feasible_constraint_invalid = 0;
//   int feasible_branch_node_counter = 0;
//   vsExploration::Node<StateVec>* best_branch_ep;
//   double best_branch_gain = 0;
//
//   for (int i=0; i < feasible_branch_counter; i++ ){
//     // ROS_WARN("Branch: %d----------------------------------------------",
//     i);
//     /*----------------------------------------------------
//     * Trace backward the tree to evaluate each node
//     ------------------------------------------------------*/
//     int counter = 0;
//     vsExploration::Node<StateVec> * end_node = feasible_branch_ep[i]; // end
//     point of this branch
//     // link to target first
//     Eigen::Vector3d link_vertex(targetNode->state_[0] - end_node->state_[0],
//                                 targetNode->state_[1] - end_node->state_[1],
//                                 targetNode->state_[2] - end_node->state_[2]);
//     vsExploration::Node<StateVec> * tgt_node = new
//     vsExploration::Node<StateVec>;
//     tgt_node->state_ = targetNode->state_;
//     tgt_node->parent_ = end_node;
//     tgt_node->distance_ = end_node->distance_ + link_vertex.norm();
//     tgt_node->gain_ = 0; // zero gain in target node means we don't care gain
//     in this node
//     //kd_insert3(kdSubTree_, tgt_node->state_.x(), tgt_node->state_.y(),
//     tgt_node->state_.z(), tgt_node);
//     tgt_node->parent_->children_.push_back(tgt_node); // make a copy
//     publishNode(tgt_node, vsExploration::BSP_PLANLEVEL);
//     //ROS_INFO("Target: %f %f %f %f", tgt_node->state_[0],
//     tgt_node->state_[1], tgt_node->state_[2], tgt_node->state_[3]);
//     vsExploration::Node<StateVec> * node = tgt_node;
//
//     vsExploration::Node<StateVec> * node_parent;
//     bool found_informative_branch = true;
//     bool stop_tracing = false;
//     // traceback step-by-step
//     while ((node->parent_ != NULL) && (found_informative_branch) &&
//     (!stop_tracing)){
//       if (node->parent_){
//         node_parent = node->parent_;
//         if (node_parent->gain_ == -1){
//           // haven't assigned any orientation
//           // sample 5 orientation
//           if (node_parent->parent_) {
//             // still in the branch
//             if (node_parent->parent_->parent_){
//               // sample new nodes and that is all
//               double best_gain = 0;
//               double best_yaw = node->state_[3];
//               int cnt = 5;
//               while (cnt){
//                 double rand_num = 2.0 * (((double) rand()) / ((double)
//                 RAND_MAX) - 0.5); //[-1.1] (-1 + 2 * cnt / 5)
//                 double dist = node->distance_ - node_parent->distance_;
//                 double rand_yaw = rand_num * params_.dyaw_max_ * dist /
//                 params_.v_max_;
//                 node_parent->state_[3] = node->state_[3] + rand_yaw;
//                 if (node_parent->state_[3] < -M_PI) node_parent->state_[3] +=
//                 2*M_PI;
//                 else if (node_parent->state_[3] > M_PI)
//                 node_parent->state_[3] -= 2*M_PI;
//
//                 if (cnt == 5) best_yaw = node_parent->state_[3]; // just
//                 choose a random orientation rather than choose the same as
//                 its children
//
//                 double gain_sampled = curiousGain(node_parent->state_);
//                 if (gain_sampled > best_gain) {
//                   best_gain = gain_sampled;
//                   best_yaw = node_parent->state_[3];
//                 }
//                 cnt--;
//               }
//               node_parent->state_[3] = best_yaw;
//               node_parent->gain_  = best_gain;
//               publishNode(node_parent, vsExploration::BSP_PLANLEVEL);
//               // ROS_INFO("Node %d: %f %f %f %f", counter,
//               node_parent->state_[0], node_parent->state_[1],
//               node_parent->state_[2], node_parent->state_[3]);
//             }else{
//               // opp: this is a special node since it requires constraints
//               from both side
//               double best_gain = 0;
//               double best_yaw = 0;
//               bool found_orientation = false;
//               double d1 = node->distance_ - node_parent->distance_;
//               Eigen::Vector3d link_vertex(sourceNode->state_[0] -
//               node_parent->state_[0],
//                                           sourceNode->state_[1] -
//                                           node_parent->state_[1],
//                                           sourceNode->state_[2] -
//                                           node_parent->state_[2]);
//               double d2 = link_vertex.norm();
//               double delta1 = params_.dyaw_max_ * d1 / params_.v_max_;
//               double delta2 = params_.dyaw_max_ * d2 / params_.v_max_;
//               int cnt = 20;
//
//               // relax set
//               bool found_orientation_relax = false;
//               double best_yaw_relax = 0;
//               double best_gain_relax = 0;
//               double delta1_relax = 1.5 * delta1; // increase to sastify
//               constraint
//               double delta2_relax = 1.5 * delta2;
//
//               while(cnt--){
//                 // uniformaly sampling
//                 //-M_PI + cnt * M_PI / 10;
//                 double rand_yaw = 2.0 * M_PI * (((double) rand()) / ((double)
//                 RAND_MAX) - 0.5); //[-pi,pi]
//
//                 double a1 = rand_yaw - node->state_[3];
//                 if (a1 < -M_PI) a1 += 2*M_PI;
//                 else if (a1 > M_PI) a1 -= 2*M_PI;
//
//                 double a2 = sourceNode->state_[3] - rand_yaw;
//                 if (a2 < -M_PI) a2 += 2*M_PI;
//                 else if (a2 > M_PI) a2 -= 2*M_PI;
//
//                 // Test yaw angle within the feasible motion of robot
//                 if ( (abs(a1) <= delta1) && (abs(a2) <= delta2)){
//                   // sastify constraints
//                   node_parent->state_[3] = rand_yaw;
//                   double gain_sampled = curiousGain(node_parent->state_);
//                   if (gain_sampled > best_gain) {
//                     best_gain = gain_sampled;
//                     best_yaw = node_parent->state_[3];
//                   }else if (!found_orientation){
//                     best_yaw = node_parent->state_[3]; // feasible
//                     orientation even it provides zero-gain
//                   }
//                   found_orientation = true;
//                }else if( (best_branch_gain == 0) &&  // execute when there is
//                no non-zero branch or havent found any solution
//                          ((!found_orientation) || (best_gain)) &&
//                          (abs(a1) <= delta1_relax) && (abs(a2) <=
//                          delta2_relax)){
//                  node_parent->state_[3] = rand_yaw;
//                  double gain_sampled = curiousGain(node_parent->state_);
//                  if (gain_sampled > best_gain_relax) {
//                    best_gain_relax = gain_sampled;
//                    best_yaw_relax = node_parent->state_[3];
//                  }else if (!found_orientation_relax){
//                    best_yaw_relax = node_parent->state_[3]; // feasible
//                    orientation even it provides zero-gain
//                  }
//                  found_orientation_relax = true;
//                }
//               }
//               // ROS_INFO("Node %d [s]: %f %f %f %f", counter,
//               node_parent->state_[0], node_parent->state_[1],
//               node_parent->state_[2], node_parent->state_[3]);
//
//               if ((!found_orientation) && (found_orientation_relax)){
//                 best_gain = best_gain_relax;
//                 found_orientation = true;
//                 best_yaw = best_yaw_relax;
//                 // ROS_WARN("Found a relax solution");
//               }
//               // Assign the best orientation
//               if (found_orientation){
//                 node_parent->state_[3] = best_yaw;
//                 node_parent->gain_  = best_gain; // not accumulate
//                 publishNode(node_parent, vsExploration::BSP_PLANLEVEL);
//                 stop_tracing = true;
//               } else{
//                 found_informative_branch = false;
//               }
//             }
//           }
//           node = node_parent;
//         } else {
//           if (node_parent->parent_) // not close to the root node
//           {
//             // already sampling
//             // let check if it sastifies constraints
//             double d = node->distance_ - node_parent->distance_;
//             double delta_d = params_.dyaw_max_ * d / params_.v_max_;
//             double delta_ori = node_parent->state_[3] - node->state_[3];
//             if (delta_ori < -M_PI) delta_ori += 2*M_PI;
//             else if (delta_ori > M_PI) delta_ori -= 2*M_PI;
//             if (abs(delta_ori) <= delta_d){
//               // good case
//               // ROS_INFO("Node %d [evaluated - g = %f]: %f %f %f %f",
//               counter, node_parent->gain_, node_parent->state_[0],
//               node_parent->state_[1], node_parent->state_[2],
//               node_parent->state_[3]);
//               stop_tracing = true;
//             }else{
//               // bad case: break a new branch
//               vsExploration::Node<StateVec> * node_break = new
//               vsExploration::Node<StateVec>;
//               node_break->state_ = node_parent->state_; //same state
//               node_break->parent_ = node_parent->parent_; //same parent
//               node_break->distance_ = node_parent->distance_;
//               node_break->gain_ = -1; // to initilize a new node in this
//               branch
//               //kd_insert3(kdSubTree_, node_break->state_.x(),
//               node_break->state_.y(), node_break->state_.z(), node_break);
//               node_break->parent_->children_.push_back(node_break); // make a
//               copy
//
//               node->parent_ = node_break;  // assign new node
//               // ROS_INFO("Node %d [re-eval]: %f %f %f %f", counter,
//               node_parent->state_[0], node_parent->state_[1],
//               node_parent->state_[2], node_parent->state_[3]);
//             }
//           }else{
//             // skip this special node
//             stop_tracing = true;
//           }
//         }
//       }
//       counter++;
//       feasible_branch_node_counter++;
//     }
//     if (found_informative_branch){
//       node = tgt_node;
//       double gain_accumulate = 0;
//       while(node->parent_){
//         gain_accumulate += node->gain_;
//         node = node->parent_;
//       }
//       if (gain_accumulate > 0) {
//         // ROS_INFO("Gain: %f", gain_accumulate);
//         feasible_branch_valid_counter++;
//         // ROS_WARN("Success with gain: %f", gain_accumulate);
//       }
//       else {
//         feasible_branch_zero_gain++;
//         // ROS_WARN("Success but zero gain");
//       }
//       tgt_node->gain_ = gain_accumulate;
//       if (gain_accumulate > best_branch_gain){
//         best_branch_gain = gain_accumulate;
//         best_branch_ep = tgt_node;
//       }
//     }else{
//       // infeasible path: discard it
//       feasible_constraint_invalid++;
//     }
//   }
//
//   /*ROS_INFO("RRT: %d nodes with %d branches [%d nodes] \n[Valid: %d;
//   Valid-but-zero-gain: %d; Invalid-due-to-constraint: %d]",
//             node_counter,
//             feasible_branch_counter,
//             feasible_branch_node_counter,
//             feasible_branch_valid_counter,
//             feasible_branch_zero_gain,
//             feasible_constraint_invalid );
//  */
//   if (best_branch_gain > 0){
//     bestNode_ = best_branch_ep;
//     bestReGain_ = bestNode_->gain_;
//     // Display new node
//     int nodeorder=2;
//     publishNode(bestNode_, vsExploration::BSP_PLANLEVEL, nodeorder);
//     ROS_INFO("The best branch with gain %f :)", bestNode_->gain_);
//     return true;
//   }else if (feasible_branch_valid_counter){
//     ROS_WARN("St wrong: best branch gain %f", best_branch_gain);
//     return false;
//   }else{
//     ROS_INFO("Can not find any non-zero branch :(");
//     return false;
//   }
// }

// span a tree to search for feasible paths
bool vsExploration::RrtTree::connect(vsExploration::Node<StateVec> *sourceNode,
                                     vsExploration::Node<StateVec> *targetNode,
                                     double extend_ratio) {
  // ROS_WARN("DEBUG: %f %f %f %f",  sourceNode->distance_, sourceNode->gain_,
  // targetNode->distance_, targetNode->gain_);
  /*--------------------
  * Database
  ---------------------*/
  // Re-intialize a root node
  sourceNode->parent_ = NULL;
  sourceNode->gain_ = 0;
  sourceNode->distance_ = 0;
  sourceNode->children_.clear();
  // Endpoints of feasible branches
  std::vector<vsExploration::Node<StateVec> *> feasible_branch_ep;
  feasible_branch_ep.clear();
  // Tree data structure
  kdSubTree_ = kd_create(3);
  // Add first node to the tree
  kd_insert3(kdSubTree_, sourceNode->state_.x(), sourceNode->state_.y(),
             sourceNode->state_.z(), sourceNode);

  /*---------------------------------
  * Initialize parameters for a tree
  ----------------------------------*/
  // Sampling space in the ellipse
  Eigen::Vector3f shortest_path(targetNode->state_.x() - sourceNode->state_.x(),
                                targetNode->state_.y() - sourceNode->state_.y(),
                                targetNode->state_.z() -
                                    sourceNode->state_.z());
  Eigen::Vector3f ellipse_center(
      (targetNode->state_.x() + sourceNode->state_.x()) / 2.0,
      (targetNode->state_.y() + sourceNode->state_.y()) / 2.0,
      (targetNode->state_.z() + sourceNode->state_.z()) / 2.0);
  double shortest_dist = shortest_path.norm();
  float dist_limit = extend_ratio * shortest_dist;
  if (dist_limit < params_.vseReplanningDistanceMin_) {
    ROS_WARN("Too short for 2nd layer, just ignore: %f(m)", dist_limit);
    return false;
  }

  Eigen::Vector3f ellipse_radius(
      0.5 * shortest_dist * extend_ratio,
      0.5 * shortest_dist * sqrt(extend_ratio * extend_ratio - 1),
      0.5 * shortest_dist * sqrt(extend_ratio * extend_ratio -
                                 1)); // x: r*d/2, y,z: d/2*sqrt(r^2-1)
  tf::Vector3 tf_e_center(ellipse_center.x(), ellipse_center.y(),
                          ellipse_center.z());
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0); // x-axis
  Eigen::Vector3f dir(shortest_path.x(), shortest_path.y(),
                      shortest_path.z()); // AB
  q.setFromTwoVectors(init, dir);
  q.normalize();
  tf::Quaternion tf_e_orient(q.x(), q.y(), q.z(), q.w());
  tf::Transform tf_e_full(tf_e_orient, tf_e_center);

  // Bsp: Publish visualization of replanning ellipsoid
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "exp_workspace";
  p.type = visualization_msgs::Marker::SPHERE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = ellipse_center.x();
  p.pose.position.y = ellipse_center.y();
  p.pose.position.z = ellipse_center.z();
  p.pose.orientation.x = tf_e_orient.x();
  p.pose.orientation.y = tf_e_orient.y();
  p.pose.orientation.z = tf_e_orient.z();
  p.pose.orientation.w = tf_e_orient.w();
  p.scale.x = 2 * ellipse_radius.x();
  p.scale.y = 2 * ellipse_radius.y();
  p.scale.z = 2 * ellipse_radius.z();
  p.color.r = 200.0 / 255.0;
  p.color.g = 200.0 / 255.0;
  p.color.b = 0;
  p.color.a = 0.5;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.planningWorkspace_.publish(p);

  // ROS_INFO("Searching path (D:%f/ExtD:%f) between [%f,%f,%f] and [%f,%f,%f]",
  //                         shortest_path.norm(),
  //                         dist_limit,
  //                         sourceNode->state_.x(),
  //                         sourceNode->state_.y(),
  //                         sourceNode->state_.z(),
  //                         targetNode->state_.x(),
  //                         targetNode->state_.y(),
  //                         targetNode->state_.z());

  /*-------------------------------
  * Loop to find feasible branches
  --------------------------------*/
  int count_true = 0, count_false = 0;
  int sampling_node = 0;
  int node_counter = 0;
  int feasible_branch_counter = 0;
  bool stop_spanning_tree = false;
  while ((!stop_spanning_tree) &&
         (feasible_branch_counter < params_.branch_max_) &&
         (node_counter < params_.node_max_)) {
    /*-------------------------------
    * Sampling a random point
    --------------------------------*/
    StateVec rand_state;
    bool found_rand_state = false;
    int sampling_count = 0;
    while (!found_rand_state) {
      // an ellipse
      double r_rand =
          2 * (((double)rand()) / ((double)RAND_MAX) - 0.5); //[-1, 1]
      double alpha =
          M_PI * (((double)rand()) / ((double)RAND_MAX) - 0.5); //[-pi/2, pi/2]
      double beta =
          2 * M_PI * (((double)rand()) / ((double)RAND_MAX) - 0.5); //[-pi, pi]
      rand_state[0] = r_rand * ellipse_radius.x() * cos(alpha) * cos(beta);
      rand_state[1] = r_rand * ellipse_radius.y() * cos(alpha) * sin(beta);
      rand_state[2] = r_rand * ellipse_radius.z() * sin(alpha);

      // // a cube
      // // rand_state[0] = ellipse_radius.x() * (((double) rand()) / ((double)
      // RAND_MAX) - 0.5); //[-r,+r]
      // // rand_state[1] = ellipse_radius.y() * (((double) rand()) / ((double)
      // RAND_MAX) - 0.5);
      // // rand_state[2] = ellipse_radius.z() * (((double) rand()) / ((double)
      // RAND_MAX) - 0.5);
      // // sampling only in the sphere within the extended distance
      tf::Vector3 state_tf =
          tf_e_full * tf::Vector3(rand_state[0], rand_state[1], rand_state[2]);
      rand_state[0] = state_tf.x();
      rand_state[1] = state_tf.y();
      rand_state[2] = state_tf.z();

      // // Sampling in the whole bounding box for cases that we need to perform
      // second layer only
      // for (int i = 0; i < 3; i++) {
      //   rand_state[i] = ((double) rand()) / ((double) RAND_MAX);
      // }
      // rand_state[0] = params_.minX_ + rand_state[0] * (params_.maxX_ -
      // params_.minX_);
      // rand_state[1] = params_.minY_ + rand_state[1] * (params_.maxY_ -
      // params_.minY_);
      // rand_state[2] = params_.minZ_ + rand_state[2] * (params_.maxZ_ -
      // params_.minZ_);

      // Compare with the limit distance to remove useless points right in this
      // stage to save time
      Eigen::Vector3f forward_vec(rand_state[0] - sourceNode->state_[0],
                                  rand_state[1] - sourceNode->state_[1],
                                  rand_state[2] - sourceNode->state_[2]);
      Eigen::Vector3f backward_vec(rand_state[0] - targetNode->state_[0],
                                   rand_state[1] - targetNode->state_[1],
                                   rand_state[2] - targetNode->state_[2]);
      if ((forward_vec.norm() + backward_vec.norm()) > dist_limit) {
        // in the ellipse sampling case, it must always sastisfy, shouldn't
        // reach here
        count_false++;
        continue;
      } else {
        count_true++;
      }

      sampling_count++;
      if (sampling_count > 1000) {
        break;
        ROS_WARN("Can not sample any new point --> Stop spanning a tree");
      }

      // Check inside the bounding box
      if (!params_.softBounds_) {
        if (rand_state.x() + params_.boundingBoxOffset_.x() <
            params_.minX_ + 0.5 * params_.boundingBox_.x()) {
          continue;
        } else if (rand_state.y() + params_.boundingBoxOffset_.y() <
                   params_.minY_ + 0.5 * params_.boundingBox_.y()) {
          continue;
        } else if (rand_state.z() + params_.boundingBoxOffset_.z() <
                   params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
          continue;
        } else if (rand_state.x() + params_.boundingBoxOffset_.x() >
                   params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
          continue;
        } else if (rand_state.y() + params_.boundingBoxOffset_.y() >
                   params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
          continue;
        } else if (rand_state.z() + params_.boundingBoxOffset_.z() >
                   params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
          continue;
        }
      }

      // only sample points in free voxels
      if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
          manager_->getCellStatusBoundingBox(
              Eigen::Vector3d(rand_state[0], rand_state[1], rand_state[2]) +
                  params_.boundingBoxOffset_,
              params_.boundingBox_)) {
        found_rand_state = true;
        // ROS_INFO("Sample point[%d,%d]: [%f,%f,%f]", sampling_count,
        // sampling_node, newState[0], newState[1], newState[2] );
      }
    }
    // ROS_INFO("RRT number of sampling points True: %d, False: %d", count_true,
    // count_false);
    if (count_true > 10000) {
      // this is because the robot is outside its workspace and can not find a
      // feasible path to come back
      // just execute the first layer
      ROS_WARN("Cannot resample new path. Status: [%d] nodes and [%d] branches",
               node_counter, feasible_branch_counter);
      return false;
    }
    /*------------------------------------
    * Add new node from new sampling point
    -------------------------------------*/
    if (!found_rand_state) {
      // stop spanning a tree since we can not sample any new point
      stop_spanning_tree = true;
    } else {
      // Find nearest node as parent
      kdres *nearest = kd_nearest3(kdSubTree_, rand_state.x(), rand_state.y(),
                                   rand_state.z());
      if (kd_res_size(nearest) <= 0) {
        kd_res_free(nearest);
        continue;
      }
      vsExploration::Node<StateVec> *parent_node = NULL;
      parent_node = (vsExploration::Node<StateVec> *)kd_res_item_data(nearest);
      kd_res_free(nearest);
      Eigen::Vector3d edge(rand_state[0] - parent_node->state_[0],
                           rand_state[1] - parent_node->state_[1],
                           rand_state[2] - parent_node->state_[2]);
      double replanning_extension_range =
          params_.vseReplanningExtensionRatio_ * 2 * ellipse_radius.x();
      if (edge.norm() > replanning_extension_range) {
        edge = replanning_extension_range * edge.normalized();
      }

      // Compute new vertex
      StateVec vertex;
      vertex[0] = parent_node->state_[0] + edge[0];
      vertex[1] = parent_node->state_[1] + edge[1];
      vertex[2] = parent_node->state_[2] + edge[2];
      vertex[3] = 0;

      // Verify the total length of this branch
      Eigen::Vector3f best_edge(vertex[0] - targetNode->state_[0],
                                vertex[1] - targetNode->state_[1],
                                vertex[2] - targetNode->state_[2]);
      float dist_vertex2src = parent_node->distance_ + edge.norm();
      float dist_vertex2tgt = dist_vertex2src + best_edge.norm();
      if (dist_vertex2tgt > dist_limit) {
        // ROS_WARN("Oop: %f %f %f", newParent->distance_, direction.norm(),
        // forward_dir.norm());
        continue;
      }

      // Check collision along this edge
      Eigen::Vector3d start(parent_node->state_.x(), parent_node->state_.y(),
                            parent_node->state_.z());
      Eigen::Vector3d end =
          start + edge + edge.normalized() * params_.dOvershoot_;
      if (volumetric_mapping::OctomapManager::CellStatus::kFree !=
          manager_->getLineStatusBoundingBox(start + params_.boundingBoxOffset_,
                                             end + params_.boundingBoxOffset_,
                                             params_.boundingBox_)) {
        continue;
      }

      // Found one more qualified vertex
      vsExploration::Node<StateVec> *new_node =
          new vsExploration::Node<StateVec>;
      new_node->state_ = vertex;
      new_node->parent_ = parent_node;
      new_node->distance_ = parent_node->distance_ + edge.norm();
      new_node->gain_ = -1;                       // haven't assigned gain
      parent_node->children_.push_back(new_node); // make a copy
      node_counter++;

      // Check target reaching
      Eigen::Vector3d target_radius(targetNode->state_[0] - vertex[0],
                                    targetNode->state_[1] - vertex[1],
                                    targetNode->state_[2] - vertex[2]);
      if (target_radius.norm() < 0.4) { //  voxel resolution
        // Reach target, but have to check the clearance
        // Check collision along this node to target node
        Eigen::Vector3d node_start(vertex[0], vertex[1], vertex[2]);
        Eigen::Vector3d node_end =
            node_start + target_radius +
            target_radius.normalized() * params_.dOvershoot_;
        if (volumetric_mapping::OctomapManager::CellStatus::kFree !=
            manager_->getLineStatusBoundingBox(
                node_start + params_.boundingBoxOffset_,
                node_end + params_.boundingBoxOffset_, params_.boundingBox_)) {
          continue;
        } else {
          feasible_branch_ep.push_back(new_node);
          feasible_branch_counter++;
          // ROS_WARN("RRT: reached target");
        }
      } else {
        kd_insert3(kdSubTree_, vertex.x(), vertex.y(), vertex.z(), new_node);
      }
      // Display new node
      // int nodeorder=0;
      // publishNode(new_node, vsExploration::BSP_PLANLEVEL, nodeorder);
      // publishNode(new_node);
      // publishNode(new_node, vsExploration::BSP_PLANLEVEL);
    }
  }

  if (feasible_branch_counter == 0) {
    ROS_WARN("Can not find any feasible path!");
    return false;
  } else {
    ROS_INFO("Found %d feasible branches", feasible_branch_counter);
  }

  // ROS_INFO("RRT number of sampling points True: %d, False: %d", count_true,
  // count_false);
  /*------------------------------------------------
  * Evaluate the gain information along all branches
  --------------------------------------------------*/
  int feasible_branch_valid_counter = 0;
  int feasible_branch_zero_gain = 0;
  int feasible_constraint_invalid = 0;
  int feasible_branch_node_counter = 0;
  vsExploration::Node<StateVec> *best_branch_ep;
  double best_branch_gain = 0;

  double entropy_gain_best = 0;
  double phi_best[50] = {0};
  int phi_best_size = 0;

  for (int i = 0; i < feasible_branch_counter; i++) {
    // ROS_WARN("Branch: %d----------------------------------------------", i);
    /*----------------------------------------------------
    * Trace backward the tree to evaluate each node
    ------------------------------------------------------*/
    int counter = 0;
    vsExploration::Node<StateVec> *end_node =
        feasible_branch_ep[i]; // end point of this branch
    // link to target first
    Eigen::Vector3d link_vertex(targetNode->state_[0] - end_node->state_[0],
                                targetNode->state_[1] - end_node->state_[1],
                                targetNode->state_[2] - end_node->state_[2]);
    vsExploration::Node<StateVec> *tgt_node = new vsExploration::Node<StateVec>;
    tgt_node->state_ = targetNode->state_;
    tgt_node->parent_ = end_node;
    tgt_node->distance_ = end_node->distance_ + link_vertex.norm();
    tgt_node->gain_ = 0; // zero gain means we don't care gain in this node
    // kd_insert3(kdSubTree_, tgt_node->state_.x(), tgt_node->state_.y(),
    // tgt_node->state_.z(), tgt_node);
    tgt_node->parent_->children_.push_back(tgt_node); // make a copy
    publishNode(tgt_node, vsExploration::BSP_PLANLEVEL);
    // ROS_INFO("Target: %f %f %f %f", tgt_node->state_[0], tgt_node->state_[1],
    // tgt_node->state_[2], tgt_node->state_[3]);
    vsExploration::Node<StateVec> *node = tgt_node;

    // MATLAB built C++ sampling function
    heading_sample_initialize();

    double dphi_max = params_.dyaw_max_ / params_.dt_;
    double v_max = params_.v_max_ / params_.dt_;
    //@TODO: tung, the t allow time, assume v_min = v_max/1.5
    double t_allow = dist_limit / (v_max / 1.5);
    double dphi_lim_min = -params_.dyaw_sampling_limit_;
    double dphi_lim_max = params_.dyaw_sampling_limit_;
    double phi_0 = sourceNode->state_[3];
    double phi_n = targetNode->state_[3];
    double m = 10;
    double max_iter = 50;

    double d[50] = {0};
    int d_size[1] = {0};

    emxArray_real_T *dphi;

    double res = 0;
    double n = 0; /// because of MATLAB
    //
    emxInitArray_real_T(&dphi, 2);

    vsExploration::Node<StateVec> *node_tmp = node;

    while (node_tmp->parent_ != NULL) {
      node_tmp = node_tmp->parent_;
      n = n + 1;
    }
    // n = n - 1;
    int n_int = (int)n;
    node_tmp = node;
    for (int itmp = (n_int - 1); itmp >= 0; itmp--) {
      d[itmp] = node_tmp->distance_ - node_tmp->parent_->distance_;
      node_tmp = node_tmp->parent_;
    }

    bool found_informative_branch = false;

    res = 0;
    // Call the entry-point 'heading_sample'.
    clock_t t2 = clock();
    clock_t t1 = clock();
    m = 5;
    max_iter = 200;
    // quick and dirty hack; this should be implemented in the MATLAB code
    if ((n > 1) && (n < 50))
      heading_sample(dphi_max, v_max, t_allow, dphi_lim_min, dphi_lim_max,
                     phi_0, phi_n, n, m, d, d_size, max_iter, &res, dphi);
    if (res == 1) {
      ROS_INFO("\nSample: Success");
      found_informative_branch = true;
    } else {
      ROS_INFO("\nSample: Fail (Error code: %f)", res);
    }
    t2 = clock();
    float diff = ((float)t2 - (float)t1);
    ROS_INFO("Time to sample: %f (sec) ", diff / CLOCKS_PER_SEC);
    //////////////////
    int sample_best_ind = 0;
    bool found_better_path = false;
    int branch_best_ind = 0;
    double branch_best_entropy = 0;
    vsExploration::Node<StateVec> *node_parent;
    if (found_informative_branch) {
      int dphi_ind = 0;
      for (int sample_iter = 0; sample_iter < (int)m; sample_iter++) {
        // Set the sampled yaw angles
        bool valid_sample = true;
        node_parent = node;
        for (int ni = (n - 1); ni > 0; ni--) {
          double dphi_tmp = dphi->data[sample_iter + ni * (int)m];
          // @TODO: check again the limit of d-phi here, this should be inside
          // MATLAB function
          if ((dphi_tmp < dphi_lim_min) || (dphi_tmp > dphi_lim_max)) {
            valid_sample = false;
          }
          node_parent->parent_->state_[3] =
              node_parent->state_[3] - dphi_tmp; // sample_iter*(int)n + ni];
          // std::cout << node_parent->parent_->state_[3] << ",";
          if (node_parent->parent_->state_[3] < -M_PI)
            node_parent->parent_->state_[3] += 2 * M_PI;
          else if (node_parent->parent_->state_[3] > M_PI)
            node_parent->parent_->state_[3] -= 2 * M_PI;
          node_parent = node_parent->parent_;
        }
        double dphi_tmp = dphi->data[sample_iter];
        // @TODO: check again the limit of d-phi here, this should be inside
        // MATLAB function
        if ((dphi_tmp < dphi_lim_min) || (dphi_tmp > dphi_lim_max)) {
          valid_sample = false;
          std::cout << "Opp..." << dphi_tmp
                    << "; Root angle: " << node_parent->state_[3] - dphi_tmp
                    << std::endl;
        }

        if (valid_sample) {
          // ROS_INFO("Compute entropy: ");
          // sample sets of yaw angles for all nodes
          double entropy_gain = entropyGain(node);
          // std::cout << "D-Entropy gain: " << entropy_gain << std::endl;

          // Update the best one
          if (entropy_gain > entropy_gain_best) {
            found_better_path = true;
            sample_best_ind = sample_iter;
            entropy_gain_best = entropy_gain;
          }

          if (entropy_gain > branch_best_entropy) {
            branch_best_ind = sample_iter;
            branch_best_entropy = entropy_gain;
          }
          // dphi_ind += n_int;
        }
      }

      // Update the best sample set
      if (found_better_path) {
        node_parent = node;
        // NOTICE THAT, TRACE BACKWARD
        std::cout << "Current best path is branch: " << i
                  << "; E-gain: " << entropy_gain_best
                  << "; Angles: " << node_parent->state_[3] * 180 / M_PI << ",";
        for (int ni = (n - 1); ni > 0; ni--) {
          double dphi_tmp = dphi->data[sample_best_ind + ni * (int)m];
          // double dphi_tmp = dphi->data[n_int*(sample_best_ind+1) - 1 - ni];
          // //wrong column and row
          node_parent->parent_->state_[3] = node_parent->state_[3] - dphi_tmp;
          if (node_parent->parent_->state_[3] < -M_PI)
            node_parent->parent_->state_[3] += 2 * M_PI;
          else if (node_parent->parent_->state_[3] > M_PI)
            node_parent->parent_->state_[3] -= 2 * M_PI;
          phi_best[ni] =
              node_parent->parent_->state_[3]; // save for the final solution
          std::cout << "[" << dphi_tmp * 180 / M_PI << "]"
                    << node_parent->parent_->state_[3] * 180 / M_PI << ",";
          publishNode(node_parent, vsExploration::BSP_PLANLEVEL);
          node_parent = node_parent->parent_;
        }
        double dphi_tmp1 = dphi->data[sample_best_ind];
        // double dphi_tmp1 = dphi->data[n_int*sample_best_ind]; //wroing
        std::cout << "[" << dphi_tmp1 * 180 / M_PI << "]"
                  << node_parent->parent_->state_[3] * 180 / M_PI << ",";
        std::cout << std::endl;
        phi_best_size = n_int - 1;
        bestNode_ = node;
      }

      // Visualize the best one in each branch only
      if ((branch_best_entropy > 0) &&
          (branch_best_entropy < entropy_gain_best)) {
        // Display the best of this branch for debug purpose
        std::cout << "Best for branch: " << i
                  << "; E-Gain: " << branch_best_entropy << std::endl;
        node_parent = node;
        for (int ni = (n - 1); ni > 0; ni--) {
          double dphi_tmp = dphi->data[branch_best_ind + ni * (int)m];
          node_parent->parent_->state_[3] = node_parent->state_[3] - dphi_tmp;
          if (node_parent->parent_->state_[3] < -M_PI)
            node_parent->parent_->state_[3] += 2 * M_PI;
          else if (node_parent->parent_->state_[3] > M_PI)
            node_parent->parent_->state_[3] -= 2 * M_PI;
          publishNode(node_parent, vsExploration::BSP_PLANLEVEL);
          node_parent = node_parent->parent_;
        }
      }
    }
    emxDestroyArray_real_T(dphi);
    // emxDestroyArray_real_T(d);
  }

  if (entropy_gain_best > 0) {
    // Set the sample for the best one
    vsExploration::Node<StateVec> *node_tmp = bestNode_->parent_;
    for (int ni = 0; ni < phi_best_size; ni++) {
      node_tmp->state_[3] = phi_best[ni];
      node_tmp = node_tmp->parent_;
    }
    ROS_INFO("Found the best path with d-entropy: %f", entropy_gain_best);
    return true;
  } else {
    ROS_WARN("Can not find any path with nonzero entropy");
  }
  return false;
}

void vsExploration::RrtTree::publishBestPath(
    vsExploration::PlanningLevel planninglevel) {
  vsExploration::Node<StateVec> *node = bestNode_;
  while (node != rootNode_ && node->parent_ != NULL) {
    const ros::Duration lifetime(params_.bestPlanMarker_lifetime_);
    visualization_msgs::Marker p;

    p.header.stamp = ros::Time::now();
    if ((planninglevel == vsExploration::NBVP_PLANLEVEL) ||
        (planninglevel == vsExploration::SAL_PLANLEVEL)) {
      p.header.seq = g_ID_;
    } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
      p.header.seq = g_ID_r_;
    } else {
      ROS_WARN("planner(publishNode): Invalid vsExploration::PlanningLevel.");
      return;
    }
    p.header.frame_id = params_.navigationFrame_;

    unsigned int markerlevel = static_cast<unsigned int>(planninglevel);

    // Orientations
    if ((planninglevel == vsExploration::NBVP_PLANLEVEL) ||
        (planninglevel == vsExploration::SAL_PLANLEVEL)) {
      p.id = g_ID_;
      g_ID_++;
    } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
      p.id = g_ID_r_;
      g_ID_r_++;
    }
    p.ns = "vp_orientations_best";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2];
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, node->state_[3]);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    p.scale.x = 0.3;
    // p.scale.x = std::max(node->gain_ / 200.0, 0.15);
    p.scale.y = 1.25 * 0.03 / (0.35 * markerlevel + 1);
    p.scale.z = 1.25 * 0.03 / (0.35 * markerlevel + 1);
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    if (planninglevel == vsExploration::NBVP_PLANLEVEL) {
      p.color.r = 0.75;
      p.color.g = 0.75;
      p.color.b = 0.0;
      params_.bestPlanningPath_markers_.markers.push_back(p);
    } else if (planninglevel == vsExploration::SAL_PLANLEVEL) {
      p.color.r = 0.0;
      p.color.g = 0.75;
      p.color.b = 0.0;
      params_.bestPlanningPath_markers_.markers.push_back(p);
    } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
      p.color.r = 0.0;
      p.color.g = 0.75;
      p.color.b = 0.0;
      params_.bestRePlanningPath_markers_.markers.push_back(p);
    }

    if (!node->parent_)
      break;

    // Positions
    if ((planninglevel == vsExploration::NBVP_PLANLEVEL) ||
        (planninglevel == vsExploration::SAL_PLANLEVEL)) {
      p.id = g_ID_;
      g_ID_++;
    } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
      p.id = g_ID_r_;
      g_ID_r_++;
    }
    p.ns = "vp_positions_best";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->parent_->state_[0];
    p.pose.position.y = node->parent_->state_[1];
    p.pose.position.z = node->parent_->state_[2];
    Eigen::Quaternion<float> q;
    Eigen::Vector3f init(1.0, 0.0, 0.0);
    Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                        node->state_[1] - node->parent_->state_[1],
                        node->state_[2] - node->parent_->state_[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.scale.x = dir.norm();
    p.scale.y = 1.25 * 0.03 / (0.35 * markerlevel + 1);
    p.scale.z = 1.25 * 0.03 / (0.35 * markerlevel + 1);
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    if (planninglevel == vsExploration::NBVP_PLANLEVEL) {
      p.color.r = 0.0;
      p.color.g = 0.75;
      p.color.b = 0.75;
      params_.bestPlanningPath_markers_.markers.push_back(p);
    } else if (planninglevel == vsExploration::SAL_PLANLEVEL) {
      p.color.r = 0.5;
      p.color.g = 0.0;
      p.color.b = 0;
      params_.bestPlanningPath_markers_.markers.push_back(p);
    } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
      p.color.r = 0.5;
      p.color.g = 0.0;
      p.color.b = 0;
      params_.bestRePlanningPath_markers_.markers.push_back(p);
    }

    node = node->parent_;
  }

  if ((planninglevel == vsExploration::NBVP_PLANLEVEL) ||
      (planninglevel == vsExploration::SAL_PLANLEVEL)) {
    params_.bestPlanningPath_.publish(params_.bestPlanningPath_markers_);
  } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
    params_.bestRePlanningPath_.publish(params_.bestRePlanningPath_markers_);
  }
}

std::vector<geometry_msgs::Pose>
vsExploration::RrtTree::getBestBranch(std::string targetFrame) {
  // Bsp: Returns the complete best branch
  // ROS_INFO("Best branch:");
  std::vector<geometry_msgs::Pose> ret, ret_edge;
  vsExploration::Node<StateVec> *current = bestNode_;
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      ret_edge =
          samplePath(current->parent_->state_, current->state_, targetFrame);
      ret.insert(ret.begin(), ret_edge.begin(), ret_edge.end());
      current = current->parent_;
    }
    ret_edge =
        samplePath(current->parent_->state_, current->state_, targetFrame);
    ret.insert(ret.begin(), ret_edge.begin(), ret_edge.end());
    // history_.push(current->parent_->state_);
    exact_root_ = bestNode_->state_; // last node
  }
  return ret;
}

void vsExploration::RrtTree::setCamModel(
    image_geometry::PinholeCameraModel &camInfo) {
  if (cam_model_ready_)
    return;
  ROS_WARN("got camera model..");
  cam_model_ = camInfo;
  std::cout << cam_model_.cameraInfo() << std::endl;
  std::cout << cam_model_.fx() << "," << cam_model_.fy() << std::endl;
  cam_model_ready_ = true;
}

double vsExploration::RrtTree::curiousGain(StateVec state) {
  // This function computes the gain
  double gain = 0.0;
  const double disc = manager_->getResolution();
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  double cam_range = params_.curious_range_; //
  double rangeSq = pow(cam_range, 2.0);
  int count = 0;

  // Iterate over all nodes within the allowed distance
  for (vec[0] = std::max(state[0] - cam_range, params_.minX_);
       vec[0] < std::min(state[0] + cam_range, params_.maxX_); vec[0] += disc) {
    for (vec[1] = std::max(state[1] - cam_range, params_.minY_);
         vec[1] < std::min(state[1] + cam_range, params_.maxY_);
         vec[1] += disc) {
      for (vec[2] = std::max(state[2] - cam_range,
                             params_.explorationMinZ_); /*params_.minZ_);*/
           vec[2] < std::min(state[2] + cam_range, params_.explorationMaxZ_);
           vec[2] += disc) {
        Eigen::Vector3d dir = vec - origin;
        // Skip if distance is too large
        if (dir.transpose().dot(dir) > rangeSq) {
          continue;
        }
        bool insideAFieldOfView = false;
        // Check that voxel center is inside one of the fields of view.
        for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator
                 itCBN = params_.camBoundNormals_.begin();
             itCBN != params_.camBoundNormals_.end(); itCBN++) {
          bool inThisFieldOfView = true;
          for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                   itCBN->begin();
               itSingleCBN != itCBN->end(); itSingleCBN++) {
            Eigen::Vector3d normal =
                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                (*itSingleCBN);
            double val = dir.dot(normal.normalized());
            if (val < SQRT2 * disc) {
              inThisFieldOfView = false;
              break;
            }
          }
          if (inThisFieldOfView) {
            insideAFieldOfView = true;
            break;
          }
        }
        if (!insideAFieldOfView) {
          continue;
        }
        count++;
        // Check cell status and add to the gain considering the corresponding
        // factor.
        double curious_gain;
        volumetric_mapping::OctomapManager::CellStatus node =
            manager_->getCuriousGain(vec, &curious_gain);
        if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
          // Rayshooting to evaluate inspectability of cell
          if (this->manager_->getVisibility(origin, vec, true) !=
              volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
            Eigen::Vector3d normal_vec(cos(state[3]), sin(state[3]), 0);
            Eigen::Vector3d dist_vec(0, 0, 0);
            dist_vec = vec - origin;
            float z = dist_vec.norm(); // dir.dot(normal_vec.normalized());
            float f = manager_->getAreaOverPixel(z);
            gain += curious_gain * exp(-f * params_.curious_coefficient_);
          }
        }
      }
    }
  }
  // Scale with volume
  // gain *= pow(disc, 3.0);
  // std::cout << "Curious: " << count << std::endl;
  return gain;
}

double vsExploration::RrtTree::entropyGain(Node<StateVec> *node) {
  // node: is an end node of a branch
  // Compute a potential difference entropy gain of this branch
  // Consider the whole path
  // Here we don't need to take into account the root node and the end node
  // Since they are the same for every branch
  double gain = 0.0;
  const double disc = manager_->getResolution();
  Eigen::Vector3d origin;
  Eigen::Vector3d vec;
  double cam_range = params_.curious_range_; //
  double rangeSq = pow(cam_range, 2.0);
  int count = 0;

  std::vector<double> vx;
  std::vector<double> vy;
  std::vector<double> vz;
  std::vector<double> vyaw;

  // ROS_INFO("Get params of viewpoints first: ");
  vsExploration::Node<StateVec> *currentNode = node->parent_;
  if (currentNode->parent_ != NULL) {
    while ((currentNode->parent_ != rootNode_) &&
           (currentNode->parent_ != NULL)) {
      vx.push_back(currentNode->state_[0]);
      vy.push_back(currentNode->state_[1]);
      vz.push_back(currentNode->state_[2]);
      vyaw.push_back(currentNode->state_[3]);
      currentNode = currentNode->parent_;
    }
  }

  int numberofNodes = vx.size();
  if (numberofNodes == 0) return 0;
  double xmin = *std::min_element(vx.begin(), vx.end());
  double xmax = *std::max_element(vx.begin(), vx.end());
  double ymin = *std::min_element(vy.begin(), vy.end());
  double ymax = *std::max_element(vy.begin(), vy.end());
  double zmin = *std::min_element(vz.begin(), vz.end());
  double zmax = *std::max_element(vz.begin(), vz.end());

  xmin = std::max(xmin - cam_range, params_.minX_);
  xmax = std::min(xmax + cam_range, params_.maxX_);
  ymin = std::max(ymin - cam_range, params_.minY_);
  ymax = std::min(ymax + cam_range, params_.maxY_);
  zmin = std::max(zmin - cam_range, params_.explorationMinZ_);
  zmax = std::min(zmax + cam_range, params_.explorationMaxZ_);

  // ROS_INFO("Params: %f %f %f %f %f %f", xmin, xmax, ymin, ymax, zmin, zmax);
  std::vector<Eigen::Vector3d> inFov_Origins;
  // ROS_INFO("Loop: ");
  // Iterate over all nodes within the allowed distance
  // Check each voxel inside FOV of each viewpoint or not
  for (vec[0] = xmin; vec[0] < xmax; vec[0] += disc) {
    for (vec[1] = ymin; vec[1] < ymax; vec[1] += disc) {
      for (vec[2] = zmin; vec[2] < zmax; vec[2] += disc) {
        // Check if it is anomaly voxel
        if (manager_->isAbnormalVoxel(vec)) {
          // printf("abnormal" );
          // std::cout << "Number of Nodes: " << numberofNodes << std::endl;
          // check each FOV here
          inFov_Origins.clear();
          for (int i = 0; i < numberofNodes; i++) {
            bool insideAFieldOfView = false;
            origin.x() = vx[i];
            origin.y() = vy[i];
            origin.z() = vz[i];

            Eigen::Vector3d dir = vec - origin;
            // Skip if distance is too large
            if (dir.transpose().dot(dir) > rangeSq) {
              continue;
            }

            // Check that voxel center is inside one of the fields of view.
            for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator
                     itCBN = params_.camBoundNormals_.begin();
                 itCBN != params_.camBoundNormals_.end(); itCBN++) {
              bool inThisFieldOfView = true;
              for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                       itCBN->begin();
                   itSingleCBN != itCBN->end(); itSingleCBN++) {
                Eigen::Vector3d normal =
                    Eigen::AngleAxisd(vyaw[i], Eigen::Vector3d::UnitZ()) *
                    (*itSingleCBN);
                double val = dir.dot(normal.normalized());
                if (val < SQRT2 * disc) {
                  inThisFieldOfView = false;
                  break;
                }
              }
              if (inThisFieldOfView) {
                insideAFieldOfView = true;
                break;
              }
            }

            if (insideAFieldOfView) {
              // add this to the list
              inFov_Origins.push_back(origin);
            }
          }
          // std::cout << "Inside FOV: " << inFov_Origins.size() << " voxels"<<
          // std::endl;
          if (inFov_Origins.size()) {
            // double gt = 0;
            // manager_->getEstEntropy(inFov_Origins[0], vec, 3, &gt) ;
            // gain += gt;
            gain += manager_->getDiffEntropyFromPath(inFov_Origins, vec);
          }
        }
      }
    }
  }
  return gain;
}

void vsExploration::RrtTree::updateToEval(ros::Time time_stamp) {
  clock_t begin_time = clock();

  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    // listener.waitForTransform(params_.navigationFrame_, params_.bodyFrame_,
    // img->header.stamp, ros::Duration(1) );
    listener.lookupTransform(params_.navigationFrame_, params_.bodyFrame_,
                             time_stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Can not lookup %s", ex.what());
    return;
  }

  tf::Vector3 position = transform.getOrigin();
  tf::Quaternion quat = transform.getRotation();
  double state[4];
  // pose
  state[0] = position.x();
  state[1] = position.y();
  state[2] = position.z();
  state[3] = tf::getYaw(quat);

  const double disc = manager_->getResolution();
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  double cam_range = params_.curious_range_; //
  double rangeSq = pow(cam_range, 2.0);
  int count = 0;

  // Iterate over all nodes within the allowed distance
  for (vec[0] = std::max(state[0] - cam_range,
                         params_.minX_ - params_.explorationExtensionX_);
       vec[0] < std::min(state[0] + cam_range,
                         params_.maxX_ + params_.explorationExtensionX_);
       vec[0] += disc) {
    for (vec[1] = std::max(state[1] - cam_range,
                           params_.minY_ - params_.explorationExtensionY_);
         vec[1] < std::min(state[1] + cam_range,
                           params_.maxY_ + params_.explorationExtensionY_);
         vec[1] += disc) {
      for (vec[2] = std::max(state[2] - cam_range,
                             params_.explorationMinZ_); /*params_.minZ_);*/
           vec[2] < std::min(state[2] + cam_range, params_.explorationMaxZ_);
           vec[2] += disc) {
        Eigen::Vector3d dir = vec - origin;
        // Skip if distance is too large
        if (dir.transpose().dot(dir) > rangeSq) {
          continue;
        }
        bool insideAFieldOfView = false;
        // Check that voxel center is inside one of the fields of view.
        for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator
                 itCBN = params_.camBoundNormals_.begin();
             itCBN != params_.camBoundNormals_.end(); itCBN++) {
          bool inThisFieldOfView = true;
          for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                   itCBN->begin();
               itSingleCBN != itCBN->end(); itSingleCBN++) {
            Eigen::Vector3d normal =
                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                (*itSingleCBN);
            double val = dir.dot(normal.normalized());
            if (val < SQRT2 * disc) {
              inThisFieldOfView = false;
              break;
            }
          }
          if (inThisFieldOfView) {
            insideAFieldOfView = true;
            break;
          }
        }
        if (!insideAFieldOfView) {
          continue;
        }

        Eigen::Vector3d dist_vec(0, 0, 0);
        dist_vec = vec - origin;
        float z = dist_vec.norm(); // dir.dot(normal_vec.normalized());
        manager_->setVoxelToEval(origin, vec, z);
        count++;
      }
    }
  }
  // std::cout << "Viewpoint: " << count << std::endl;
}

std::vector<geometry_msgs::Pose>
vsExploration::RrtTree::getBestEdge(std::string targetFrame) {
  // This function returns the first edge of the best branch
  std::vector<geometry_msgs::Pose> ret;
  vsExploration::Node<StateVec> *current = bestNode_;
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      current = current->parent_;
    }
    ret = samplePath(current->parent_->state_, current->state_, targetFrame);
    history_.push(current->parent_->state_);
    exact_root_ = current->state_;
  }
  return ret;
}

double vsExploration::RrtTree::gain(StateVec state) {
  // This function computes the gain
  double gain = 0.0;
  const double disc = manager_->getResolution();
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  double rangeSq = pow(params_.gainRange_, 2.0);
  // Iterate over all nodes within the allowed distance

  for (vec[0] = std::max(state[0] - params_.gainRange_,
                         params_.minX_ - params_.explorationExtensionX_);
       vec[0] < std::min(state[0] + params_.gainRange_,
                         params_.maxX_ + params_.explorationExtensionX_);
       vec[0] += disc) {
    for (vec[1] = std::max(state[1] - params_.gainRange_,
                           params_.minY_ - params_.explorationExtensionY_);
         vec[1] < std::min(state[1] + params_.gainRange_,
                           params_.maxY_ + params_.explorationExtensionY_);
         vec[1] += disc) {
      for (vec[2] = std::max(state[2] - params_.gainRange_,
                             params_.explorationMinZ_);
           vec[2] < std::min(state[2] + params_.gainRange_,
                             params_.maxZ_ + params_.explorationMaxZ_);
           vec[2] += disc) {
        Eigen::Vector3d dir = vec - origin;
        // Skip if distance is too large
        if (dir.transpose().dot(dir) > rangeSq) {
          continue;
        }
        bool insideAFieldOfView = false;
        // Check that voxel center is inside one of the fields of view.
        for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator
                 itCBN = params_.camBoundNormals_.begin();
             itCBN != params_.camBoundNormals_.end(); itCBN++) {
          bool inThisFieldOfView = true;
          for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN =
                   itCBN->begin();
               itSingleCBN != itCBN->end(); itSingleCBN++) {
            Eigen::Vector3d normal =
                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                (*itSingleCBN);
            double val = dir.dot(normal.normalized());
            if (val < SQRT2 * disc) {
              inThisFieldOfView = false;
              break;
            }
          }
          if (inThisFieldOfView) {
            insideAFieldOfView = true;
            break;
          }
        }
        if (!insideAFieldOfView) {
          continue;
        }
        // Check cell status and add to the gain considering the corresponding
        // factor.
        double probability;
        volumetric_mapping::OctomapManager::CellStatus node =
            manager_->getCellProbabilityPoint(vec, &probability);
        if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
              this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igUnmapped_;
            // TODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ *
            // PROBABILISTIC_MODEL(probability);
          }
        } else if (node ==
                   volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
              this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igOccupied_;
            // TODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ *
            // PROBABILISTIC_MODEL(probability);
          }
        } else {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
              this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igFree_;
            // TODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ *
            // PROBABILISTIC_MODEL(probability);
          }
        }
      }
    }
  }
  // Scale with volume
  gain *= pow(disc, 3.0);
  // Check the gain added by inspectable surface
  if (mesh_) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state.x(), state.y(), state.z()));
    tf::Quaternion quaternion;
    quaternion.setEuler(0.0, 0.0, state[3]);
    transform.setRotation(quaternion);
    gain += params_.igArea_ * mesh_->computeInspectableArea(transform);
  }
  return gain;
}

std::vector<geometry_msgs::Pose>
vsExploration::RrtTree::getPathBackToPrevious(std::string targetFrame) {
  std::vector<geometry_msgs::Pose> ret;
  if (history_.empty()) {
    return ret;
  }
  ret = samplePath(root_, history_.top(), targetFrame);
  history_.pop();
  return ret;
}

void vsExploration::RrtTree::memorizeBestBranch() {
  bestBranchMemory_.clear();
  Node<StateVec> *current = bestNode_;
  while (current->parent_ && current->parent_->parent_) {
    bestBranchMemory_.push_back(current->state_);
    current = current->parent_;
  }
}

void vsExploration::RrtTree::clear() {
  delete rootNode_;
  rootNode_ = NULL;

  counter_ = 0;
  bestGain_ = params_.zero_gain_;
  bestReGain_ = 0;
  bestNode_ = NULL;

  kd_free(kdTree_);

  // Clear visualization MarkerArrays
  params_.planningPath_markers_.markers.clear();
  params_.planningPathStats_markers_.markers.clear();
  params_.bestPlanningPath_markers_.markers.clear();
  params_.rePlanningPath_markers_.markers.clear();
  params_.rePlanningPathStats_markers_.markers.clear();
  params_.bestRePlanningPath_markers_.markers.clear();
}

void vsExploration::RrtTree::publishPath(
    vsExploration::PlanningLevel planninglevel) {
  if ((planninglevel == vsExploration::PlanningLevel::NBVP_PLANLEVEL) ||
      (planninglevel == vsExploration::PlanningLevel::SAL_PLANLEVEL)) {
    params_.planningPath_.publish(params_.planningPath_markers_);
    params_.planningPathStats_.publish(params_.planningPathStats_markers_);
  } else if (planninglevel == vsExploration::PlanningLevel::BSP_PLANLEVEL) {
    params_.rePlanningPath_.publish(params_.rePlanningPath_markers_);
    params_.rePlanningPathStats_.publish(params_.rePlanningPathStats_markers_);
  } else {
    ROS_WARN("planner(publishPath): Invalid vsExploration::PlanningLevel.");
  }
}

void vsExploration::RrtTree::publishNode_old(Node<StateVec> *node) {
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::min(node->gain_ / 100.0, 0.02);
  ; // std::max(node->gain_ / 100.0, 0.05);
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0 / 255.0;
  p.color.g = 167.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);

  if (!node->parent_)
    return;

  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_branches";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->parent_->state_[0];
  p.pose.position.y = node->parent_->state_[1];
  p.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 0.03;
  p.scale.z = 0.03;
  p.color.r = 100.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.7;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);

  if (params_.log_) {
    for (int i = 0; i < node->state_.size(); i++) {
      fileTree_ << node->state_[i] << ",";
    }
    fileTree_ << node->gain_ << ",";
    for (int i = 0; i < node->parent_->state_.size(); i++) {
      fileTree_ << node->parent_->state_[i] << ",";
    }
    fileTree_ << node->parent_->gain_ << "\n";
  }
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
  std::ostringstream out;
  out << std::setprecision(n) << std::fixed << a_value;
  return out.str();
}

void vsExploration::RrtTree::publishNode(
    Node<StateVec> *node, vsExploration::PlanningLevel planninglevel,
    int nodeorder) {
  const ros::Duration lifetime(params_.planMarker_lifetime_);
  visualization_msgs::Marker p;

  p.header.stamp = ros::Time::now();
  if ((planninglevel == vsExploration::NBVP_PLANLEVEL) ||
      (planninglevel == vsExploration::SAL_PLANLEVEL)) {
    p.header.seq = g_ID_;
  } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
    p.header.seq = g_ID_r_;
  } else {
    ROS_WARN("planner(publishNode): Invalid vsExploration::PlanningLevel.");
    return;
  }
  p.header.frame_id = params_.navigationFrame_;

  unsigned int markerlevel = static_cast<unsigned int>(planninglevel);

  // Orientations
  if ((planninglevel == vsExploration::NBVP_PLANLEVEL) ||
      (planninglevel == vsExploration::SAL_PLANLEVEL)) {
    p.id = g_ID_;
    g_ID_++;
  } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
    p.id = g_ID_r_;
    g_ID_r_++;
  }
  p.ns = "vp_orientations";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = 0.3;
  p.scale.y = 0.03 / (0.35 * markerlevel + 1);
  p.scale.z = 0.03 / (0.35 * markerlevel + 1);
  p.color.a = 1.0;
  p.lifetime = lifetime;
  p.frame_locked = false;
  if (planninglevel == vsExploration::NBVP_PLANLEVEL) {
    p.color.r = 0.75;
    p.color.g = 0.75;
    p.color.b = 0.0;
    params_.planningPath_markers_.markers.push_back(p);
  } else if (planninglevel == vsExploration::SAL_PLANLEVEL) {
    p.color.r = 0.0;
    p.color.g = 0.75;
    p.color.b = 0.0;
    params_.planningPath_markers_.markers.push_back(p);
  } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
    p.color.r = 0.0;
    p.color.g = 0.75;
    p.color.b = 0.0;
    params_.rePlanningPath_markers_.markers.push_back(p);
  }

  double p_text_scale = 0.1;
  if ((planninglevel == vsExploration::NBVP_PLANLEVEL) ||
      (planninglevel == vsExploration::SAL_PLANLEVEL)) {
    // Numbers
    unsigned int p_tot =
        node->unmapped_cnt_ + node->occupied_cnt_ + node->free_cnt_;

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_connection"; // connection:=WhiteAlpha
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2];
    p.color.r = 1.0;
    p.color.g = 1.0;
    p.color.b = 1.0;
    p.color.a = 0.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(nodeorder) + ":";
    if (!node->parent_)
      p.text += std::string("-1");
    else
      p.text += std::to_string(node->parent_->id_);
    p.text += std::string(",") + std::to_string(node->id_);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_gain"; // gain:=Black
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = 1.25 * p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] + p.scale.z * 2.5;
    p.color.r = 0.0;
    p.color.g = 0.0;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = to_string_with_precision((float)node->gain_, 2);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_unmapped_gain"; // kUnmapped:=Blue
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] + p.scale.z * 2;
    p.color.r = 0.0;
    p.color.g = 0.0;
    p.color.b = 0.75;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = to_string_with_precision((float)node->unmapped_gain_, 2);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_occupied_gain"; // kOccupied:=Green
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] + p.scale.z * 1;
    p.color.r = 0.0;
    p.color.g = 0.75;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = to_string_with_precision((float)node->occupied_gain_, 2);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_unmapped_cnt"; // visibility:=Alpha
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] - p.scale.z * 1;
    p.color.r = 0.0;
    p.color.g = 0.75;
    p.color.b = 0.75;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(node->unmapped_cnt_);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_occupied_cnt"; // kOccupied:=Yellow
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] - p.scale.z * 2;
    p.color.r = 0.0;
    p.color.g = 0.75;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(node->occupied_cnt_);
    params_.planningPathStats_markers_.markers.push_back(p);
  } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
    p.id = g_ID_Sr_;
    g_ID_Sr_++;
    p.ns = "vp_reconnection"; // reconnection:=WhiteAlpha
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2];
    p.color.r = 1.0;
    p.color.g = 1.0;
    p.color.b = 1.0;
    p.color.a = 0.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(nodeorder) + ":";
    if (!node->parent_)
      p.text += std::string("-1");
    else
      p.text += std::to_string(node->parent_->id_);
    p.text += std::string(",") + std::to_string(node->id_);
    params_.rePlanningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_Sr_;
    g_ID_Sr_++;
    p.ns = "gain";
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = 1.25 * 0.75 * p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] + 1.25 * p.scale.z;
    p.color.r = 0.0;
    p.color.g = 0.0;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(node->gain_);
    params_.rePlanningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_Sr_;
    g_ID_Sr_++;
    p.ns = "vp_distance";
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = 0.75 * p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2] - p.scale.z * 1;
    p.color.r = 1.0;
    p.color.g = 1.0;
    p.color.b = 1.0;
    p.color.a = 0.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(node->distance_);
    params_.rePlanningPathStats_markers_.markers.push_back(p);
  }

  if (!node->parent_)
    return;

  // Positions
  if ((planninglevel == vsExploration::NBVP_PLANLEVEL) ||
      (planninglevel == vsExploration::SAL_PLANLEVEL)) {
    p.id = g_ID_;
    g_ID_++;
  } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
    p.id = g_ID_r_;
    g_ID_r_++;
  }
  p.ns = "vp_positions";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->parent_->state_[0];
  p.pose.position.y = node->parent_->state_[1];
  p.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 0.03 / (0.35 * planninglevel + 1);
  p.scale.z = 0.03 / (0.35 * planninglevel + 1);
  p.color.a = 1.0;
  p.lifetime = lifetime;
  p.frame_locked = false;
  if (planninglevel == vsExploration::NBVP_PLANLEVEL) {
    p.color.r = 0.0;
    p.color.g = 0.0;
    p.color.b = 0.75;
    params_.planningPath_markers_.markers.push_back(p);
  } else if (planninglevel == vsExploration::SAL_PLANLEVEL) {
    p.color.r = 0.75;
    p.color.g = 0.0;
    p.color.b = 0.0;
    params_.planningPath_markers_.markers.push_back(p);
  } else if (planninglevel == vsExploration::BSP_PLANLEVEL) {
    p.color.r = 0.75;
    p.color.g = 0.0;
    p.color.b = 0.0;
    params_.rePlanningPath_markers_.markers.push_back(p);
  }
}

std::vector<geometry_msgs::Pose>
vsExploration::RrtTree::samplePath(StateVec start, StateVec end,
                                   std::string targetFrame) {
  std::vector<geometry_msgs::Pose> ret;
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(targetFrame, params_.navigationFrame_,
                             ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return ret;
  }
  Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1],
                           end[2] - start[2]);
  double yaw_direction = end[3] - start[3];
  if (yaw_direction > M_PI) {
    yaw_direction -= 2.0 * M_PI;
  }
  if (yaw_direction < -M_PI) {
    yaw_direction += 2.0 * M_PI;
  }

  bool useConstraints = false;
  if (useConstraints) {
    geometry_msgs::Pose pose;
    StateVec state = start;
    for (unsigned int i = 0; i < 2; ++i, state = end) {
      tf::Vector3 origin(state[0], state[1], state[2]);
      double yaw = state[3];
      if (yaw > M_PI)
        yaw -= 2.0 * M_PI;
      else if (yaw < -M_PI)
        yaw += 2.0 * M_PI;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      origin = transform * origin;
      quat = transform * quat;
      tf::Pose poseTF(quat, origin);
      tf::poseTFToMsg(poseTF, pose);
      ret.push_back(pose);
    }
    tf::Vector3 origin_end(end[0], end[1], end[2]);
  } else {
    double disc =
        std::min(params_.dt_ * params_.v_max_ / distance.norm(),
                 params_.dt_ * params_.dyaw_max_ / abs(yaw_direction));
    assert(disc > 0.0);
    for (double it = 0.0; it <= 1.0; it += disc) {
      tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                         (1.0 - it) * start[1] + it * end[1],
                         (1.0 - it) * start[2] + it * end[2]);
      double yaw = start[3] + yaw_direction * it;
      if (yaw > M_PI)
        yaw -= 2.0 * M_PI;
      if (yaw < -M_PI)
        yaw += 2.0 * M_PI;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      origin = transform * origin;
      quat = transform * quat;
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      ret.push_back(pose);
    }
  }
  return ret;
}

// std::vector<geometry_msgs::Pose> vsExploration::RrtTree::samplePath(StateVec
// start, StateVec end,
//                                                                     std::string
//                                                                     targetFrame)
// {
//   std::vector<geometry_msgs::Pose> ret;
//   static tf::TransformListener listener;
//   tf::StampedTransform transform;
//   try {
//     listener.lookupTransform(targetFrame, params_.navigationFrame_,
//     ros::Time(0), transform);
//   } catch (tf::TransformException ex) {
//     ROS_ERROR("%s", ex.what());
//     return ret;
//   }
//   Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1], end[2] -
//   start[2]);
//   double yaw_direction = end[3] - start[3];
//   if (yaw_direction > M_PI) {
//     yaw_direction -= 2.0 * M_PI;
//   }
//   if (yaw_direction < -M_PI) {
//     yaw_direction += 2.0 * M_PI;
//   }
//   double disc = std::min(params_.dt_ * params_.v_max_ / distance.norm(),
//                          params_.dt_ * params_.dyaw_max_ /
//                          abs(yaw_direction));
//   assert(disc > 0.0);
//   for (double it = 0.0; it <= 1.0; it += disc) {
//     tf::Vector3 origin((1.0 - it) * start[0] + it * end[0], (1.0 - it) *
//     start[1] + it * end[1],
//                        (1.0 - it) * start[2] + it * end[2]);
//     double yaw = start[3] + yaw_direction * it;
//     if (yaw > M_PI)
//       yaw -= 2.0 * M_PI;
//     if (yaw < -M_PI)
//       yaw += 2.0 * M_PI;
//     tf::Quaternion quat;
//     quat.setEuler(0.0, 0.0, yaw);
//     origin = transform * origin;
//     quat = transform * quat;
//     tf::Pose poseTF(quat, origin);
//     geometry_msgs::Pose pose;
//     tf::poseTFToMsg(poseTF, pose);
//     ret.push_back(pose);
//     if (params_.log_) {
//       filePath_ << poseTF.getOrigin().x() << ",";
//       filePath_ << poseTF.getOrigin().y() << ",";
//       filePath_ << poseTF.getOrigin().z() << ",";
//       filePath_ << tf::getYaw(poseTF.getRotation()) << "\n";
//     }
//   }
//   return ret;
// }

#endif
