#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vseplanner/vsep.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vsePlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  vsExploration::vsePlanner<Eigen::Matrix<double, 4, 1> > planner(nh, nh_private);

  ros::spin();
  return 0;
}
