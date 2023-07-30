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

#ifndef NBVP_HPP_
#define NBVP_HPP_

#include <fstream>
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <iostream>


#include <visualization_msgs/Marker.h>

#include <vseplanner/vsep.h>

// Convenience macro to get the absolute yaw difference
#define ANGABS(x) (fmod(fabs(x),2.0*M_PI)<M_PI?fmod(fabs(x),2.0*M_PI):2.0*M_PI-fmod(fabs(x),2.0*M_PI))

using namespace Eigen;

template<typename stateVec>
vsExploration::vsePlanner<stateVec>::vsePlanner(const ros::NodeHandle& nh,
                                                const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{

  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);

  params_.planningWorkspace_ = nh_.advertise<visualization_msgs::Marker>("planningWorkspace", 1000);
  params_.planningPath_ = nh_.advertise<visualization_msgs::MarkerArray>("planningPath", 1000);
  params_.planningPathStats_ = nh_.advertise<visualization_msgs::MarkerArray>("planningPathStats", 1000);
  params_.bestPlanningPath_ = nh_.advertise<visualization_msgs::MarkerArray>("bestPlanningPath", 1000);
  params_.rePlanningPath_ = nh_.advertise<visualization_msgs::MarkerArray>("rePlanningPath", 1000);
  params_.rePlanningPathStats_ = nh_.advertise<visualization_msgs::MarkerArray>("rePlanningPathStats", 1000);
  params_.bestRePlanningPath_ = nh_.advertise<visualization_msgs::MarkerArray>("bestRePlanningPath", 1000);

  // Set up the topics and services
  params_.inspectionPath_ = nh_.advertise<visualization_msgs::Marker>("inspectionPath", 1000);
  plannerService_ = nh_.advertiseService("vseplanner",
                                         &vsExploration::vsePlanner<stateVec>::plannerCallback,
                                         this);
  saveWaypointService_ = nh_.advertiseService("save_waypoint",
                                         &vsExploration::vsePlanner<stateVec>::saveWaypointCallback,
                                         this);
  saveMapClient_ = nh_.serviceClient<volumetric_msgs::SaveMap> ("vsePlanner/save_map");

  plannerPub_ = nh_.advertise<nav_msgs::Path>("planner_path", 1000);

  posClient_ = nh_.subscribe("pose", 10, &vsExploration::vsePlanner<stateVec>::posCallback, this);
  odomClient_ = nh_.subscribe("odometry", 10, &vsExploration::vsePlanner<stateVec>::odomCallback, this);

  pointcloud_sub_ = nh_.subscribe("pointcloud_throttled", 1,
                                  &vsExploration::vsePlanner<stateVec>::insertPointcloudWithTf,
                                  this);
  pointcloud_sub_cam_up_ = nh_.subscribe(
      "pointcloud_throttled_up", 1,
      &vsExploration::vsePlanner<stateVec>::insertPointcloudWithTfCamUp, this);
  pointcloud_sub_cam_down_ = nh_.subscribe(
      "pointcloud_throttled_down", 1,
      &vsExploration::vsePlanner<stateVec>::insertPointcloudWithTfCamDown, this);

  image_transport::ImageTransport it(nh_);
  subCam = it.subscribe("image_saliency", 1,  &vsExploration::vsePlanner<stateVec>::camCallback, this );
  subCamInfo = nh_.subscribe("image_info", 2, &vsExploration::vsePlanner<stateVec>::camInfoCallback, this );

  if (!setParams()) {
    ROS_ERROR("Could not start the planner. Parameters missing!");
  }

  // Precompute the camera field of view boundaries. The normals of the separating hyperplanes are stored
  for (int i = 0; i < params_.camPitch_.size(); i++) {
    double pitch = M_PI * params_.camPitch_[i] / 180.0;
    double camTop = (pitch - M_PI * params_.camVertical_[i] / 360.0) + M_PI / 2.0;
    double camBottom = (pitch + M_PI * params_.camVertical_[i] / 360.0) - M_PI / 2.0;
    double side = M_PI * (params_.camHorizontal_[i]) / 360.0 - M_PI / 2.0;
    Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
    Vector3d top(cos(camTop), 0.0, -sin(camTop));
    Vector3d right(cos(side), sin(side), 0.0);
    Vector3d left(cos(side), -sin(side), 0.0);
    AngleAxisd m = AngleAxisd(pitch, Vector3d::UnitY());
    Vector3d rightR = m * right;
    Vector3d leftR = m * left;
    rightR.normalize();
    leftR.normalize();
    std::vector<Eigen::Vector3d> camBoundNormals;
    camBoundNormals.push_back(bottom);
    // ROS_INFO("bottom: (%2.2f, %2.2f, %2.2f)", bottom[0], bottom[1], bottom[2]);
    camBoundNormals.push_back(top);
    // ROS_INFO("top: (%2.2f, %2.2f, %2.2f)", top[0], top[1], top[2]);
    camBoundNormals.push_back(rightR);
    // ROS_INFO("rightR: (%2.2f, %2.2f, %2.2f)", rightR[0], rightR[1], rightR[2]);
    camBoundNormals.push_back(leftR);
    // ROS_INFO("leftR: (%2.2f, %2.2f, %2.2f)", leftR[0], leftR[1], leftR[2]);
    params_.camBoundNormals_.push_back(camBoundNormals);
  }

  // Load mesh from STL file if provided.
  std::string ns = ros::this_node::getName();
  std::string stlPath = "";
  mesh_ = NULL;
  if (ros::param::get(ns + "/stl_file_path", stlPath)) {
    if (stlPath.length() > 0) {
      if (ros::param::get(ns + "/mesh_resolution", params_.meshResolution_)) {
        std::fstream stlFile;
        stlFile.open(stlPath.c_str());
        if (stlFile.is_open()) {
          mesh_ = new mesh::StlMesh(stlFile);
          mesh_->setResolution(params_.meshResolution_);
          mesh_->setOctomapManager(manager_);
          mesh_->setCameraParams(params_.camPitch_, params_.camHorizontal_, params_.camVertical_,
                                 params_.gainRange_);
        } else {
          ROS_WARN("Unable to open STL file");
        }
      } else {
        ROS_WARN("STL mesh file path specified but mesh resolution parameter missing!");
      }
    }
  }
  // Initialize the tree instance.
  tree_ = new RrtTree(mesh_, manager_);
  tree_->setParams(params_);
  peerPosClient1_ = nh_.subscribe("peer_pose_1", 10,
                                  &vsExploration::RrtTree::setPeerStateFromPoseMsg1, tree_);
  peerPosClient2_ = nh_.subscribe("peer_pose_2", 10,
                                  &vsExploration::RrtTree::setPeerStateFromPoseMsg2, tree_);
  peerPosClient3_ = nh_.subscribe("peer_pose_3", 10,
                                  &vsExploration::RrtTree::setPeerStateFromPoseMsg3, tree_);
  // Not yet initialized.
  params_.planner_state_ = PSTATE_NONE;
  // Not yet ready. Needs a position message first.
  ready_ = false;
  num_runs_ = 0;
}

template<typename stateVec>
vsExploration::vsePlanner<stateVec>::~vsePlanner()
{
  if (manager_) {
    delete manager_;
  }
  if (mesh_) {
    delete mesh_;
  }
}

template<typename stateVec>
void vsExploration::vsePlanner<stateVec>::posCallback(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  tree_->setStateFromPoseMsg(pose);
  // Setup state in intial freespace-clearance-around-agent mode.
  if (!ready_) {
    params_.planner_state_ = PSTATE_BBXCLEARANCE;
  }
  // Planner is now ready to plan.
  ready_ = true;
}

template<typename stateVec>
void vsExploration::vsePlanner<stateVec>::odomCallback(
    const nav_msgs::Odometry& pose)
{
  tree_->setStateFromOdometryMsg(pose);
  // Setup state in intial freespace-clearance-around-agent mode.
  if (!ready_) {
    params_.planner_state_ = PSTATE_BBXCLEARANCE;
  }
  // Planner is now ready to plan.
  ready_ = true;
}

template<typename stateVec>
bool vsExploration::vsePlanner<stateVec>::saveWaypointCallback(std_srvs::Empty::Request& req, 
                                                               std_srvs::Empty::Response& res)
{
  tree_->saveWaypoint();
  return true;
}

template<typename stateVec>
bool vsExploration::vsePlanner<stateVec>::plannerCallback(vsep_msgs::vsep_srv::Request& req,
                                                          vsep_msgs::vsep_srv::Response& res)
{
  ros::Time computationTime = ros::Time::now();
  // Check that planner is ready to compute path.
  if (!ros::ok()) {
    ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
    return true;
  }

  if (!ready_) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Planner not ready!");
    return true;
  }
  if (manager_ == NULL) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
    return true;
  }
  if (manager_->getMapSize().norm() <= 0.0) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Octomap is empty!");
    return true;
  }
  if (tree_->getPlannerState() == PSTATE_EXPL_FINISH) {
    return true;
  }
  res.path.clear();

  // Clear old tree and reinitialize.
  tree_->clear();
  tree_->initialize(num_runs_);
  vector_t path;
  // Iterate the tree construction method.
  bool brkflag = false;
  int loopCount = 0;

  int planner_mode = 1;
  static int target_counter = 0;
  if (tree_->getPlannerState() == PSTATE_EXPL_SAL){
    // only case we switch to saliency only, and turn off the second layer
    planner_mode = 2;
    ROS_WARN("Switched to saliency based exploration");
    //ROS_WARN("cuEnable: %s", params_.cuEnable_?"true":"false");
  }

  while ((!tree_->gainFound() || tree_->getCounter() < params_.initIterations_) && ros::ok()) {
    if (tree_->getCounter() > params_.cutoffIterations_) {
      ROS_INFO("No gain found, shutting down");
      // ros::shutdown();
      brkflag = true;
      break;
    }
    if (loopCount > 1000 * (tree_->getCounter() + 1)) {
      ROS_WARN("planner<iterate>: Exceeding maximum failed iterations, return to previous point!");
      res.path = tree_->getPathBackToPrevious(req.header.frame_id);
      brkflag =  true;
      break;
    }
    tree_->iterate(num_runs_, planner_mode);
    loopCount++;
  }
  if (brkflag) return true;

  // evaluate current situation first
  double ext_ratio = 0;
  tree_->plannerEvaluate(ext_ratio);

  // Resample first vertex of NBVP planning level
  if (planner_mode != 2){
    if (tree_->resampleFirstVertex(num_runs_)){
      ROS_INFO_STREAM("planner(resampleFirstVertex): SUCCESS");
    }
    else{
      ROS_INFO_STREAM("planner(resampleFirstVertex): Failed to improve...");
    }
  }

  if (planner_mode == 2) {
    tree_->publishPath(SAL_PLANLEVEL);
    tree_->publishBestPath(SAL_PLANLEVEL);
  }
  else {
    tree_->publishPath(NBVP_PLANLEVEL);
    tree_->publishBestPath(NBVP_PLANLEVEL); // NBVP path
  }

  res.path = tree_->getBestEdge(req.header.frame_id);
  ROS_INFO("Finished 1st layer: path computation lasted %2.3fs with %d nodes",
            (ros::Time::now() - computationTime).toSec(), tree_->getCounter());

  if (!res.path.empty()){
    ++num_runs_; //successful NBVP planning steps
  }

  computationTime = ros::Time::now();
  if ((params_.cuEnable_) && (planner_mode != 2)){
    bool res_resampleBestEdge = tree_->resampleBestEdge(ext_ratio);

    //Bsp: BSP planning level done, publish entire MarkerArray (full nested replanning tree)
    tree_->publishPath(BSP_PLANLEVEL);
    if (res_resampleBestEdge){
      //Bsp: Extract the best branch
      res.path = tree_->getBestBranch(req.header.frame_id);
      if (!res.path.empty()){
        // ++num_reruns_; //successful BSP planning steps
      }
      //Bsp: Publish extracted path (best nested branch)
      tree_->publishBestPath(BSP_PLANLEVEL); // VSEP path
      ROS_INFO("Finished 2nd layer: path computation lasted %2.3fs",
                (ros::Time::now() - computationTime).toSec());
    }else{
      tree_->updateVSEPWaypoint(); // append latest Exploration waypoints to VSEP waypoint list here
      ROS_WARN("Failed to find a path in 2nd layer (%2.3fs)",
                (ros::Time::now() - computationTime).toSec());
    }
  }

  nav_msgs::Path planner_path;
  planner_path.header.stamp = ros::Time::now();
  planner_path.header.frame_id = params_.navigationFrame_;
  for (int i=0;i<res.path.size();++i){
    geometry_msgs::PoseStamped posestamped_i;
    posestamped_i.header.stamp = ros::Time::now()+ros::Duration(1.0);
    posestamped_i.header.frame_id = params_.navigationFrame_;
    posestamped_i.pose = res.path.at(i);
    planner_path.poses.push_back(posestamped_i);
  }

  if (!planner_path.poses.empty()){
    if (params_.planner_state_ == PSTATE_BBXCLEARANCE) {
      // cancel updating the clearance box
      params_.planner_state_ = PSTATE_INITIALIZED;
    }
    plannerPub_.publish( planner_path );
  }else{
    ROS_WARN("vsePlanner: No path computed...");
  }

  return true;
}

template<typename stateVec>
void vsExploration::vsePlanner<stateVec>::camInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& camInfoIn)
{
  if (CamInfoReady) return;

  CamInfo = *camInfoIn;
  CamModel.fromCameraInfo (camInfoIn);
  CamInfoReady = true;
  tree_->setCamModel(CamModel);
  tree_->setCamInfo(CamModel);

}

template<typename stateVec>
void vsExploration::vsePlanner<stateVec>::camCallback(
    const sensor_msgs::ImageConstPtr& camIn)
{
  if (!CamInfoReady) return;

  if ((params_.sal_wait_for_planner_ && (params_.planner_state_ >= PSTATE_INITIALIZED)) ||
      (!params_.sal_wait_for_planner_)){
    params_.sal_wait_for_planner_ = false; // auto update from now

    // insert a salient image into the octomap
    tree_->insertSaliencyImgWithTf(camIn);

    // for evaluation
    tree_->updateToEval(camIn->header.stamp);

  }
}

template<typename stateVec>
bool vsExploration::vsePlanner<stateVec>::setParams()
{
  std::string ns = ros::this_node::getName();
  bool ret = true;
  params_.v_max_ = 0.25;
  if (!ros::param::get(ns + "/system/v_max", params_.v_max_)) {
    ROS_WARN("No maximal system speed specified. Looking for %s. Default is 0.25.",
             (ns + "/system/v_max").c_str());
  }
  params_.dyaw_max_ = 0.5;
  if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_)) {
    ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.",
             (ns + "/system/yaw_max").c_str());
  }
  params_.camPitch_ = {15.0};
  if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_)) {
    ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.",
             (ns + "/system/camera/pitch").c_str());
  }
  params_.camHorizontal_ = {90.0};
  if (!ros::param::get(ns + "/system/camera/horizontal", params_.camHorizontal_)) {
    ROS_WARN("No camera horizontal opening specified. Looking for %s. Default is 90deg.",
             (ns + "/system/camera/horizontal").c_str());
  }
  params_.camVertical_ = {60.0};
  if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_)) {
    ROS_WARN("No camera vertical opening specified. Looking for %s. Default is 60deg.",
             (ns + "/system/camera/vertical").c_str());
  }
  if(params_.camPitch_.size() != params_.camHorizontal_.size() ||params_.camPitch_.size() != params_.camVertical_.size() ){
    ROS_WARN("Specified camera fields of view unclear: Not all parameter vectors have same length! Setting to default.");
    params_.camPitch_.clear();
    params_.camPitch_ = {15.0};
    params_.camHorizontal_.clear();
    params_.camHorizontal_ = {90.0};
    params_.camVertical_.clear();
    params_.camVertical_ = {60.0};
  }
  params_.igProbabilistic_ = 0.0;
  if (!ros::param::get(ns + "/vsep/gain/probabilistic", params_.igProbabilistic_)) {
    ROS_WARN(
        "No gain coefficient for probability of cells specified. Looking for %s. Default is 0.0.",
        (ns + "/vsep/gain/probabilistic").c_str());
  }
  params_.igFree_ = 0.0;
  if (!ros::param::get(ns + "/vsep/gain/free", params_.igFree_)) {
    ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.",
             (ns + "/vsep/gain/free").c_str());
  }
  params_.igOccupied_ = 0.0;
  if (!ros::param::get(ns + "/vsep/gain/occupied", params_.igOccupied_)) {
    ROS_WARN("No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.",
             (ns + "/vsep/gain/occupied").c_str());
  }
  params_.igUnmapped_ = 1.0;
  if (!ros::param::get(ns + "/vsep/gain/unmapped", params_.igUnmapped_)) {
    ROS_WARN("No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.",
             (ns + "/vsep/gain/unmapped").c_str());
  }
  params_.igArea_ = 1.0;
  if (!ros::param::get(ns + "/vsep/gain/area", params_.igArea_)) {
    ROS_WARN("No gain coefficient for mesh area specified. Looking for %s. Default is 1.0.",
             (ns + "/vsep/gain/area").c_str());
  }
  params_.degressiveSwitchoffLoops_ = 5;
  if (!ros::param::get(ns + "/vsep/gain/degressive_switchoffLoops", params_.degressiveSwitchoffLoops_)) {
    ROS_WARN("No Loop counts for degressive factor for gain accumulation specified. Looking for %s. Default is 5.",
             (ns + "/vsep/gain/degressive_switchoffLoops").c_str());
  }
  params_.degressiveCoeff_ = 0.25;
  if (!ros::param::get(ns + "/vsep/gain/degressive_coeff", params_.degressiveCoeff_)) {
    ROS_WARN(
        "No degressive factor for gain accumulation specified. Looking for %s. Default is 0.25.",
        (ns + "/vsep/gain/degressive_coeff").c_str());
  }
  params_.extensionRange_ = 1.0;
  if (!ros::param::get(ns + "/vsep/tree/extension_range", params_.extensionRange_)) {
    ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
             (ns + "/vsep/tree/extension_range").c_str());
  }
  params_.initIterations_ = 15;
  if (!ros::param::get(ns + "/vsep/tree/initial_iterations", params_.initIterations_)) {
    ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
             (ns + "/vsep/tree/initial_iterations").c_str());
  }
  params_.dt_ = 0.1;
  if (!ros::param::get(ns + "/vsep/dt", params_.dt_)) {
    ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
             (ns + "/vsep/dt").c_str());
  }
  params_.gainRange_ = 1.0;
  if (!ros::param::get(ns + "/vsep/gain/range", params_.gainRange_)) {
    ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
             (ns + "/vsep/gain/range").c_str());
  }

  params_.yaw_sampling_limit_ = 2 * M_PI; // 2pi: no limit
  if (!ros::param::get(ns + "/vsep/yaw_sampling_limit", params_.yaw_sampling_limit_)) {
    ROS_WARN("No yaw sampling limit specified. Looking for %s. Default is 2*PI.",
             (ns + "/vsep/yaw_sampling_limit").c_str());
  }

  if (!ros::param::get(ns + "/bbx/minX", params_.minX_)) {
    ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minY", params_.minY_)) {
    ROS_WARN("No y-min value specified. Looking for %s", (ns + "/bbx/minY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_)) {
    ROS_WARN("No z-min value specified. Looking for %s", (ns + "/bbx/minZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_)) {
    ROS_WARN("No x-max value specified. Looking for %s", (ns + "/bbx/maxX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_)) {
    ROS_WARN("No y-max value specified. Looking for %s", (ns + "/bbx/maxY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_)) {
    ROS_WARN("No z-max value specified. Looking for %s", (ns + "/bbx/maxZ").c_str());
    ret = false;
  }


  if (!ros::param::get(ns + "/bbx/plusX", params_.plusX_)) {
  	ROS_WARN("No plusX value specified. Looking for %s", (ns + "/bbx/plusX").c_str());
  	ret = false;
  }

  if (!ros::param::get(ns + "/bbx/plusY", params_.plusY_)) {
    ROS_WARN("No plusY value specified. Looking for %s", (ns + "/bbx/plusY").c_str());
    ret = false;
  }

  params_.softBounds_ = false;
  if (!ros::param::get(ns + "/bbx/softBounds", params_.softBounds_)) {
    ROS_WARN(
        "Not specified whether scenario bounds are soft or hard. Looking for %s. Default is false",
        (ns + "/bbx/softBounds").c_str());
  }
  params_.boundingBox_[0] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0])) {
    ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/x").c_str());
  }
  params_.boundingBox_[1] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1])) {
    ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/y").c_str());
  }
  params_.boundingBox_[2] = 0.3;
  if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2])) {
    ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
             (ns + "/system/bbx/z").c_str());
  }

  params_.boundingBoxOffset_[0] = 0.0;
  if (!ros::param::get(ns + "/system/bbx/x_offset", params_.boundingBoxOffset_[0])) {
    ROS_WARN("No x offset value specified. Looking for %s. Default is 0.0m.", (ns + "/system/bbx/x_offset").c_str());
  }
  params_.boundingBoxOffset_[1] = 0.0;
  if (!ros::param::get(ns + "/system/bbx/y_offset", params_.boundingBoxOffset_[1])) {
    ROS_WARN("No y offset value specified. Looking for %s. Default is 0.0m.", (ns + "/system/bbx/y_offset").c_str());
  }
  params_.boundingBoxOffset_[2] = 0.0;
  if (!ros::param::get(ns + "/system/bbx/z_offset", params_.boundingBoxOffset_[2])) {
    ROS_WARN("No z offset value specified. Looking for %s. Default is 0.0m.", (ns + "/system/bbx/z_offset").c_str());
  }
  params_.cutoffIterations_ = 200;
  if (!ros::param::get(ns + "/vsep/tree/cutoff_iterations", params_.cutoffIterations_)) {
    ROS_WARN("No cutoff iterations value specified. Looking for %s. Default is 200.",
             (ns + "/vsep/tree/cutoff_iterations").c_str());
  }
  params_.zero_gain_ = 0.0;
  if (!ros::param::get(ns + "/vsep/gain/zero", params_.zero_gain_)) {
    ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
             (ns + "/vsep/gain/zero").c_str());
  }
  params_.dOvershoot_ = 0.5;
  if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_)) {
    ROS_WARN(
        "No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.",
        (ns + "/system/bbx/overshoot").c_str());
  }
  params_.log_ = false;
  if (!ros::param::get(ns + "/vsep/log/on", params_.log_)) {
    ROS_WARN("Logging is off by default. Turn on with %s: true", (ns + "/vsep/log/on").c_str());
  }
  params_.log_throttle_ = 0.5;
  if (!ros::param::get(ns + "/vsep/log/throttle", params_.log_throttle_)) {
    ROS_WARN("No throttle time for logging specified. Looking for %s. Default is 0.5s.",
             (ns + "/vsep/log/throttle").c_str());
  }
  params_.navigationFrame_ = "world";
  if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_)) {
    ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.",
             (ns + "/tf_frame").c_str());
  }

  params_.bodyFrame_ = "body";
  if (!ros::param::get(ns + "/body_frame", params_.bodyFrame_)) {
    ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.",
             (ns + " ").c_str());
  }

  params_.pcl_throttle_ = 0.333;
  if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_)) {
    ROS_WARN(
        "No throttle time constant for the point cloud insertion specified. Looking for %s. Default is 0.333.",
        (ns + "/pcl_throttle").c_str());
  }
  params_.inspection_throttle_ = 0.25;
  if (!ros::param::get(ns + "/inspection_throttle", params_.inspection_throttle_)) {
    ROS_WARN(
        "No throttle time constant for the inspection view insertion specified. Looking for %s. Default is 0.1.",
        (ns + "/inspection_throttle").c_str());
  }
  params_.exact_root_ = true;
  if (!ros::param::get(ns + "/vsep/tree/exact_root", params_.exact_root_)) {
    ROS_WARN("No option for exact root selection specified. Looking for %s. Default is true.",
             (ns + "/vsep/tree/exact_root").c_str());
  }

  params_.softStart_ = true;
  if (!ros::param::get(ns + "/vsep/softStart", params_.softStart_)) {
    ROS_WARN("No option for soft starting (clearing a flight space of free cells until first successful iteration) specified. Looking for %s. Default is true.",
             (ns + "/vsep/softStart").c_str());
  }
  params_.softStartMinZ_ = 0.0;
  if (!ros::param::get(ns + "/vsep/softStartMinZ", params_.softStartMinZ_)) {
    ROS_WARN("No option for soft starting MinZ (clearing a flight space of free cells only over this MinZ value) specified. Looking for %s. Default is 0.0.",
             (ns + "/vsep/softStartMinZ").c_str());
  }

  params_.cuEnable_ = false;
  if (!ros::param::get(ns + "/vsep/curious_enable", params_.cuEnable_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.curious_range_ = 1.0;
  if (!ros::param::get(ns + "/vsep/curious_range", params_.curious_range_)) {
    ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
             (ns + "/vsep/gain/range").c_str());
  }

  params_.curious_coefficient_ = 1.0;
  if (!ros::param::get(ns + "/vsep/curious_coefficient", params_.curious_coefficient_)) {
    ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
             (ns + "/vsep/gain/range").c_str());
  }

  params_.branch_max_ = 10;
  if (!ros::param::get(ns + "/vsep/branch_max", params_.branch_max_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.node_max_ = 1000;
  if (!ros::param::get(ns + "/vsep/node_max", params_.node_max_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.extend_ratio_fixed_ = 1.5;
  if (!ros::param::get(ns + "/vsep/extend_ratio_fixed", params_.extend_ratio_fixed_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.extend_ratio_max_ = 1.5;
  if (!ros::param::get(ns + "/vsep/extend_ratio_max", params_.extend_ratio_max_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.exp_lower_bound_ = 0.2;
  if (!ros::param::get(ns + "/vsep/exp_lower_bound", params_.exp_lower_bound_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.exp_upper_bound_ = 0.9;
  if (!ros::param::get(ns + "/vsep/exp_upper_bound", params_.exp_upper_bound_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.exp_filter_window_ = 3;
  if (!ros::param::get(ns + "/vsep/exp_filter_window", params_.exp_filter_window_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.extend_enable_= false;
  if (!ros::param::get(ns + "/vsep/extend_enable", params_.extend_enable_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.time_budget_ = 1000.0;
  if (!ros::param::get(ns + "/vsep/time_budget", params_.time_budget_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.sal_ground_remove_level_ = -10.0;
  if (!ros::param::get(ns + "/vsep/sal_ground_remove_level", params_.sal_ground_remove_level_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.sal_wait_for_planner_  = false;
  if (!ros::param::get(ns + "/vsep/sal_wait_for_planner", params_.sal_wait_for_planner_)) {
    ROS_WARN("Not specified whether activate 2nd layer or not. Looking for %s. Default is false", (ns + "vsep/curious_enable").c_str());
  }

  params_.vseReplanningDistanceMin_ = 0.5;
  if (!ros::param::get(ns + "/vsep/replanning_distance_min", params_.vseReplanningDistanceMin_)) {
    ROS_WARN("No min distance threshold for Replanning step. Looking for %s. Default is 0.25.", (ns + "/vse/replanning_distance_min").c_str());
  }

  params_.vseReplanningExtensionRatio_ = 0.5;
  if (!ros::param::get(ns + "/vsep/replanning_extension_ratio", params_.vseReplanningExtensionRatio_)) {
    ROS_WARN("No ratio for max extension range in Replanning step. Looking for %s. Default is 0.5.", (ns + "/vse/replanning_extension_ratio").c_str());
  }

  params_.planMarker_lifetime_ = 15.0;
  if (!ros::param::get(ns + "/markers/planMarker_lifetime", params_.planMarker_lifetime_)) {
    ROS_WARN("No lifetime for planning markers specified. Looking for %s. Default is 15.0s.", (ns + "/markers/planMarker_lifetime").c_str());
  }
  params_.bestPlanMarker_lifetime_ = 15.0;
  if (!ros::param::get(ns + "/markers/bestPlanMarker_lifetime", params_.bestPlanMarker_lifetime_)) {
    ROS_WARN("No lifetime for Best planning markers specified. Looking for %s. Default is 15.0s.", (ns + "/markers/bestPlanMarker_lifetime").c_str());
  }
  params_.replanMarker_lifetime_ = 15.0;
  if (!ros::param::get(ns + "/markers/replanMarker_lifetime", params_.replanMarker_lifetime_)) {
    ROS_WARN("No lifetime for replanning markers specified. Looking for %s. Default is 15.0s.", (ns + "/markers/replanMarker_lifetime").c_str());
  }
  params_.bestReplanMarker_lifetime_ = 15.0;
  if (!ros::param::get(ns + "/markers/bestReplanMarker_lifetime", params_.bestReplanMarker_lifetime_)) {
    ROS_WARN("No lifetime for Best replanning markers specified. Looking for %s. Default is 15.0s.", (ns + "/markers/bestReplanMarker_lifetime").c_str());
  }

  if (!ros::param::get(ns + "/bbx/explorationExtensionX", params_.explorationExtensionX_)) {
    ROS_WARN("No BBx x-extension value for IG collection (exploration space) specified. Looking for %s", (ns + "/bbx/explorationExtensionX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/explorationExtensionY", params_.explorationExtensionY_)) {
    ROS_WARN("No BBx y-extension value for IG collection (exploration space) specified. Looking for %s", (ns + "/bbx/explorationExtensionY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/explorationMinZ", params_.explorationMinZ_)) {
    ROS_WARN("No BBx z-min value for IG collection (exploration space) specified. Looking for %s", (ns + "/bbx/explorationMinZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/explorationMaxZ", params_.explorationMaxZ_)) {
    ROS_WARN("No BBx z-max value for IG collection (exploration space) specified. Looking for %s", (ns + "/bbx/explorationMaxZ").c_str());
    ret = false;
  }

  return ret;
}

template<typename stateVec>
void vsExploration::vsePlanner<stateVec>::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;

    if (params_.softStart_&& params_.planner_state_<=PSTATE_BBXCLEARANCE){
      Eigen::Vector3d boundingBoxSize(params_.boundingBox_[0]+5.0*params_.dOvershoot_,params_.boundingBox_[1]+5.0*params_.dOvershoot_,params_.boundingBox_[2]+5.0*params_.dOvershoot_);
      //const double minZ = 2.0*manager_->getResolution();
      tree_->clearRootStateBBX(boundingBoxSize,params_.softStartMinZ_); //minZ);
    }


  }
}

template<typename stateVec>
void vsExploration::vsePlanner<stateVec>::insertPointcloudWithTfCamUp(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}

template<typename stateVec>
void vsExploration::vsePlanner<stateVec>::insertPointcloudWithTfCamDown(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}


#endif // NBVP_HPP_
