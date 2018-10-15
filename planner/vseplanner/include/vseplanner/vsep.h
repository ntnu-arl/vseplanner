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

#ifndef VSEP_H_
#define VSEP_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>

#include <octomap_world/octomap_manager.h>
#include <vseplanner/vsep_srv.h>
#include <vseplanner/octomap_srv.h>
#include <vseplanner/mesh_structure.h>
#include <vseplanner/tree.hpp>
#include <vseplanner/rrt.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>

//Customization

// #define SQ(x) ((x)*(x))
// #define SQRT2 0.70711

namespace vsExploration {

template<typename stateVec>
class vsePlanner
{

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber posClient_;
  ros::Subscriber odomClient_;
  ros::Subscriber peerPosClient1_;
  ros::Subscriber peerPosClient2_;
  ros::Subscriber peerPosClient3_;
  ros::ServiceServer plannerService_;

  ros::ServiceClient plannerPubServiceClient_;
  ros::ServiceClient saveMapClient_;
  ros::Publisher plannerPub_;

  ros::Subscriber pointcloud_sub_;
  ros::Subscriber pointcloud_sub_cam_up_;
  ros::Subscriber pointcloud_sub_cam_down_;

  image_transport::Subscriber subCam;
  ros::Subscriber subCamInfo;

  Params params_;
  mesh::StlMesh * mesh_;
  volumetric_mapping::OctomapManager * manager_;

  bool ready_;
  int num_runs_;

  sensor_msgs::CameraInfo CamInfo;
  bool CamInfoReady = false;
  image_geometry::PinholeCameraModel CamModel;
  bool octree_update_ = false;
 public:
  typedef std::vector<stateVec> vector_t;
  TreeBase<stateVec> * tree_;

  vsePlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~vsePlanner();
  bool setParams();
  void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void odomCallback(const nav_msgs::Odometry& pose);
  bool plannerCallback(vseplanner::vsep_srv::Request& req, vseplanner::vsep_srv::Response& res);
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  void insertPointcloudWithTfCamUp(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  void insertPointcloudWithTfCamDown(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);

  void camCallback(const sensor_msgs::ImageConstPtr& camIn);
  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camInfoIn);

};
}

#endif // NBVP_H_
