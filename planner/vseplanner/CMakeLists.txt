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
  vsep_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

add_service_files(
  FILES
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
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf kdtree vsep_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)
MESSAGE( STATUS "variable is:------------------------ " ${catkin_INCLUDE_DIRS} )
add_library(vsePlannerLib src/mesh_structure.cpp src/vsep.cpp src/rrt.cpp src/tree.cpp)
add_dependencies(vsePlannerLib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(vsePlanner src/vseplanner_node.cpp src/mesh_structure.cpp src/vsep.cpp src/rrt.cpp src/tree.cpp)
add_dependencies(vsePlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vsePlannerLib
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
target_link_libraries(vsePlanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

