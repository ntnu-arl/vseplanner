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

#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>
#include <vseplanner/tree.h>
#include <vseplanner/mesh_structure.h>
#include <vseplanner/graph.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace vsExploration {

class RrtTree : public TreeBase<Eigen::Vector4d>
{
 public:
  typedef Eigen::Vector4d StateVec;

  RrtTree();
  RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~RrtTree();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer);
  virtual void initialize(int numRuns);
  virtual void iterate(int numRuns, int plannerMode);
  virtual void buildGraph();
  virtual void evaluateGraph();
  virtual bool resampleBestEdge(double ext_ratio);
  virtual bool resampleFirstVertex(int numRuns);
  void rePublishNode(Node<StateVec> * node, vsExploration::PlanningLevel planninglevel=NBVP_PLANLEVEL, int nodeorder=0);
  virtual bool connect(Node<StateVec> *sourceNode, Node<StateVec> * targetNode, double extend_ratio);
  virtual int  getPlannerState(void);
  virtual void setHardTarget(std::vector<double> val);
  virtual bool getBestVertex(std::vector<double> &vertex);

  virtual void setCamModel(image_geometry::PinholeCameraModel& camInfo);
  virtual void plannerEvaluate(double &extend_ratio);
  virtual void updateToEval(ros::Time time_stamp);
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame);
  virtual std::vector<geometry_msgs::Pose> getBestBranch(std::string targetFrame);
  virtual void publishPath(vsExploration::PlanningLevel planninglevel=NBVP_PLANLEVEL);
  virtual void publishBestPath(vsExploration::PlanningLevel planninglevel=NBVP_PLANLEVEL);

  virtual void clear();
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame);
  virtual void memorizeBestBranch();
  void publishNode_old(Node<StateVec> * node);
  void publishNode(Node<StateVec> * node, vsExploration::PlanningLevel planninglevel=NBVP_PLANLEVEL, int nodeorder=0);

  double gain(StateVec state);
  double curiousGain(StateVec state);
  bool sampleVertex(StateVec &state);

  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end,
                                              std::string targetFrame);


 protected:
  kdtree * kdTree_;
  kdtree * kdSubTree_;

  int g_ID_S_;
  int g_ID_r_;
  int g_ID_Sr_;
  int n_ID_;
  int n_ID_r_;

  std::stack<StateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
  std::vector<double> inspectionThrottleTime_;

  std::vector<double> erate_buf_;
  image_geometry::PinholeCameraModel cam_model_;
  bool cam_model_ready_;

  gbplanner::Graph graph_;
  std::map<int, Node<StateVec>*> vertex_state_map_;

};
}

#endif
