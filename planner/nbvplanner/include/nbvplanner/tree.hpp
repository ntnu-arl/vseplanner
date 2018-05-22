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

#ifndef TREE_HPP_
#define TREE_HPP_

#include <nbvplanner/tree.h>

template<typename stateVec>
nbvInspection::Node<stateVec>::Node()
{
  parent_ = NULL;
  distance_ = DBL_MAX;
  gain_ = 0.0;
}

template<typename stateVec>
nbvInspection::Node<stateVec>::~Node()
{
  for (typename std::vector<Node<stateVec> *>::iterator it = children_.begin();
      it != children_.end(); it++) {
    delete (*it);
    (*it) = NULL;
  }
}

template<typename stateVec>
nbvInspection::TreeBase<stateVec>::TreeBase()
{
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
}

template<typename stateVec>
nbvInspection::TreeBase<stateVec>::TreeBase(mesh::StlMesh * mesh,
                                            volumetric_mapping::OctomapManager * manager)
{
  mesh_ = mesh;
  manager_ = manager;
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
}

template<typename stateVec>
nbvInspection::TreeBase<stateVec>::~TreeBase()
{
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::setPeerStateFromPoseMsg1(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  setPeerStateFromPoseMsg(pose, 1);
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::setPeerStateFromPoseMsg2(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  setPeerStateFromPoseMsg(pose, 2);
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::setPeerStateFromPoseMsg3(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  setPeerStateFromPoseMsg(pose, 3);
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::setParams(Params params)
{
  params_ = params;
  // set size of the workspace (JAH)
  manager_->setWorkspaceBox(params.minX_ - params.plusX_,
                            params.minY_ - params.plusY_,
                            params.explorationMinZ_,
                            params.maxX_ + params_.plusX_,
                            params.maxY_ + params_.plusY_,
                            params.maxZ_); //explorationMaxZ_: NOTE: can not reach to explorationMaxZ_
  ROS_INFO("Set workspace [%f,%f,%f,%f,%f,%f]", params.minX_ - params.plusX_,
                            										params.minY_ - params.plusY_,
                            										params.explorationMinZ_,
                           											params.maxX_ + params_.plusX_,
                            										params.maxY_ + params_.plusY_,
                            										params.maxZ_);
}

template<typename stateVec>
int nbvInspection::TreeBase<stateVec>::getCounter()
{
  return counter_;
}

template<typename stateVec>
bool nbvInspection::TreeBase<stateVec>::gainFound()
{
  return bestGain_ > params_.zero_gain_;
}

template<typename stateVec>
bool nbvInspection::TreeBase<stateVec>::reGainFound()
{
  return bestReGain_ > initReGain_;
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  manager_->insertPointcloudWithTf(pointcloud);
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::insertSaliencyImgWithTf(
    const sensor_msgs::ImageConstPtr& img)
{
  manager_->insertSaliencyImgWithTf(img);
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::setCamInfo(
    image_geometry::PinholeCameraModel& camInfo)
{
  manager_->setCamInfo(camInfo);
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::clearTakeoffBBX(
    Eigen::Vector3d& boundingBox_
    )
{
  //manager_->clearBBX(Eigen::Vector3d(root_[0],root_[1],0.5*boundingBox_[2]),boundingBox_); //Will only work on already existent nodes
  manager_->setFree(Eigen::Vector3d(root_[0],root_[1],0.5*boundingBox_[2]),boundingBox_); //Will use virtual points to create required nodes
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::clearRootStateBBX(
    Eigen::Vector3d& boundingBox_,
    const double& minZ
    )
{
  const double epsilon = manager_->getResolution()/2;
  Eigen::Vector3d boundingBoxOffset(0.0,0.0,0.0);
  if (root_[2]-boundingBox_[2]/2<minZ){
    boundingBoxOffset[2] = -(root_[2]-boundingBox_[2]/2-minZ)/2 + epsilon;
    boundingBox_[2] -= boundingBoxOffset[2];
  }
  //manager_->clearBBX(Eigen::Vector3d(root_[0],root_[1],root_[2]), boundingBox_, bounddingBoxOffset); //Will only work on already existent nodes
  manager_->setFree(Eigen::Vector3d(root_[0],root_[1],root_[2]), boundingBox_, boundingBoxOffset); //Will use virtual points to create required nodes
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::getRootState(
    Eigen::Vector3d& rootState_
    )
{
  rootState_[0] = root_[0];
  rootState_[1] = root_[1];
  rootState_[2] = root_[2];
}


#endif
