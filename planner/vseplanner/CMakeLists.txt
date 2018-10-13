cmake_minimum_required(VERSION 2.8.3)
project(vseplanner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  octomap_world
  tf
  kdtree
  cv_bridge
  image_transport
  image_geometry
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

add_service_files(
  FILES
  vsep_srv.srv
  octomap_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES vseplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf kdtree
)

include_directories(
  include
  heading_sample
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

#MATLAB
add_library(headingSampling
heading_sample/inv.cpp
heading_sample/sum.cpp
heading_sample/svd.cpp
heading_sample/diag.cpp
heading_sample/main.cpp
heading_sample/norm.cpp
heading_sample/rand.cpp
heading_sample/sqrt.cpp
heading_sample/svd1.cpp
heading_sample/xaxpy.cpp
heading_sample/xdotc.cpp
heading_sample/xnrm2.cpp
heading_sample/xrotg.cpp
heading_sample/xscal.cpp
heading_sample/rdivide.cpp
heading_sample/rtGetInf.cpp
heading_sample/rtGetNaN.cpp
heading_sample/rt_nonfinite.cpp
heading_sample/heading_sample.cpp
heading_sample/parabola_fitting.cpp
heading_sample/heading_sample_data.cpp
heading_sample/heading_sample_emxAPI.cpp
heading_sample/heading_sample_verify.cpp
heading_sample/heading_sample_emxutil.cpp
heading_sample/heading_sample_terminate.cpp
heading_sample/heading_sample_initialize.cpp
heading_sample/eml_rand_mt19937ar_stateful.cpp
  )

add_library(vsePlannerLib src/mesh_structure.cpp src/vsep.cpp src/rrt.cpp src/tree.cpp)
add_dependencies(vsePlannerLib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(vsePlanner src/vseplanner_node.cpp src/mesh_structure.cpp src/vsep.cpp src/rrt.cpp src/tree.cpp)
add_dependencies(vsePlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vsePlannerLib
	headingSampling
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
target_link_libraries(vsePlanner
  headingSampling
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
