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

#ifndef TREE_H_
#define TREE_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_world/octomap_manager.h>
#include <vseplanner/mesh_structure.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>

// planner states
#define PSTATE_NONE          -1
#define PSTATE_BBXCLEARANCE   0
#define PSTATE_INITIALIZED    1
#define PSTATE_EXPL_ONLY      2
#define PSTATE_EXPL_FULL      3
#define PSTATE_EXPL_SAL       4
#define PSTATE_EXPL_FINISH    5

namespace vsExploration {

enum PlanningLevel { NBVP_PLANLEVEL, BSP_PLANLEVEL, SAL_PLANLEVEL};

struct Params
{
  //BSP service client
  ros::ServiceClient vseServiceClient_;

  // state
  int planner_state_;

  //BSP params
  bool vseEnable_;
  double vseReplanningDistanceMin_;
  double vseReplanningExtensionRatio_;

  //Visualization params
  double planMarker_lifetime_;
  double bestPlanMarker_lifetime_;
  double replanMarker_lifetime_;
  double bestReplanMarker_lifetime_;
  visualization_msgs::MarkerArray planningPath_markers_;
  visualization_msgs::MarkerArray planningPathStats_markers_;
  visualization_msgs::MarkerArray bestPlanningPath_markers_;
  visualization_msgs::MarkerArray rePlanningPath_markers_;
  visualization_msgs::MarkerArray rePlanningPathStats_markers_;
  visualization_msgs::MarkerArray bestRePlanningPath_markers_;

  std::vector<double> camPitch_;
  std::vector<double> camHorizontal_;
  std::vector<double> camVertical_;
  std::vector<std::vector<Eigen::Vector3d> > camBoundNormals_;

  double igProbabilistic_;
  double igFree_;
  double igOccupied_;
  double igUnmapped_;
  double igArea_;
  double gainRange_;
  double degressiveCoeff_;
  double zero_gain_;

  double yaw_sampling_limit_;
  double v_max_;
  double dyaw_max_;
  double dOvershoot_;
  double extensionRange_;
  bool exact_root_;
  int initIterations_;
  int cutoffIterations_;
  double dt_;

  double minX_;
  double minY_;
  double minZ_;
  double maxX_;
  double maxY_;
  double maxZ_;
  double plusX_;
  double plusY_;
  double explorationExtensionX_;
  double explorationExtensionY_;
  double explorationMinZ_;
  double explorationMaxZ_;
  bool softBounds_;
  Eigen::Vector3d boundingBox_;

  //Exploratory behavior modifiers

  int degressiveSwitchoffLoops_;
  Eigen::Vector3d boundingBoxOffset_;

  //Publishers
  ros::Publisher planningWorkspace_;
  ros::Publisher planningPath_;
  ros::Publisher planningPathStats_;
  ros::Publisher bestPlanningPath_;
  ros::Publisher rePlanningPath_;
  ros::Publisher rePlanningPathStats_;
  ros::Publisher bestRePlanningPath_;


  bool cuEnable_;
  double curious_range_;
  double curious_coefficient_;
  int branch_max_;
  int node_max_;
  float extend_ratio_fixed_;
  float extend_ratio_max_;
  bool extend_enable_;
  float time_budget_;
  float exp_lower_bound_;
  float exp_upper_bound_;
  int exp_filter_window_;
  float sal_ground_remove_level_;
  bool sal_wait_for_planner_;

  double meshResolution_;

  ros::Publisher inspectionPath_;
  std::string navigationFrame_;
  std::string bodyFrame_;

  bool log_;
  double log_throttle_;
  double pcl_throttle_;
  double inspection_throttle_;

  bool softStart_;
  double softStartMinZ_;

  bool explorationMode_;

};

template<typename stateVec>
class Node
{
 public:
  Node();
  ~Node();
  stateVec state_;
  Node * parent_;
  std::vector<Node*> children_;
  double gain_;
  double distance_;

  int id_;
  bool leafNode_;

  unsigned int unmapped_cnt_;
  unsigned int occupied_cnt_;
  unsigned int free_cnt_;
  double unmapped_gain_;
  double occupied_gain_;
  double free_gain_;
};

template<typename stateVec>
class TreeBase
{
 protected:
  Params params_;
  int counter_;
  double bestGain_;
  double bestReGain_;
  double initReGain_;
  double bestSingleGain_;

  Node<stateVec> * bestNode_;
  Node<stateVec> * rootNode_;
  mesh::StlMesh * mesh_;
  volumetric_mapping::OctomapManager * manager_;
  stateVec root_;
  stateVec exact_root_;
  std::vector<std::vector<Eigen::Vector3d>*> segments_;
  std::vector<std::string> agentNames_;
 public:
  TreeBase();
  TreeBase(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~TreeBase();

  void clearTakeoffBBX(Eigen::Vector3d& boundingBox_);
  void clearRootStateBBX(Eigen::Vector3d& boundingBox_, const double& minZ = -std::numeric_limits<double>::infinity());
  void getRootState(Eigen::Vector3d& rootState_);

  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose) = 0;
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose) = 0;
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer) = 0;
  void setPeerStateFromPoseMsg1(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void setPeerStateFromPoseMsg2(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void setPeerStateFromPoseMsg3(const geometry_msgs::PoseWithCovarianceStamped& pose);
  virtual void iterate(int numRuns, int plannerMode) = 0;
  virtual void buildGraph() = 0;
  virtual void evaluateGraph() = 0;
  virtual void initialize(int numRuns) = 0;
  virtual bool resampleBestEdge(double ext_ratio) = 0;
  virtual bool resampleFirstVertex(int numRuns) = 0;
  virtual bool connect(Node<stateVec> *sourceNode,Node<stateVec> * targetNode, double extend_ratio) = 0;
  virtual int getPlannerState(void) = 0;
  virtual void setHardTarget(std::vector<double> val) = 0;
  virtual bool getBestVertex(std::vector<double> &vertex) = 0;

  virtual void setCamModel(image_geometry::PinholeCameraModel& camInfo) = 0;
  virtual void plannerEvaluate(double &extend_ratio) = 0;
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame) = 0;

  virtual void clear() = 0;

  virtual void updateToEval(ros::Time time_stamp) = 0;
  virtual std::vector<geometry_msgs::Pose> getBestBranch(std::string targetFrame) = 0;
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame) = 0;
  virtual void memorizeBestBranch() = 0;
  void setParams(Params params);

  //Publishing
  virtual void publishPath(vsExploration::PlanningLevel planninglevel=NBVP_PLANLEVEL) = 0;
  virtual void publishBestPath(vsExploration::PlanningLevel planninglevel=NBVP_PLANLEVEL) = 0;

  int getCounter();
  bool gainFound();
  bool reGainFound();
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  void insertSaliencyImgWithTf(const sensor_msgs::ImageConstPtr& img);
  void setCamInfo(image_geometry::PinholeCameraModel& camInfo);
};

}

#endif
