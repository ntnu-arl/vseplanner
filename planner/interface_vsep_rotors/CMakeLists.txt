cmake_minimum_required(VERSION 2.8.3)
project(interface_vsep_rotors)

#add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  mav_msgs
  tf
  vsep_msgs
)

add_service_files(
  FILES
  trigger_planner.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)   

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS trajectory_msgs mav_msgs tf vsep_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(exploration src/exploration.cpp)
add_dependencies(exploration ${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS} )
target_link_libraries(exploration ${catkin_LIBRARIES})


