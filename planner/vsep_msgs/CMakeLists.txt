cmake_minimum_required(VERSION 2.8.3)
project(vsep_msgs)
find_package(catkin REQUIRED COMPONENTS 
std_msgs 
geometry_msgs
message_generation)

add_service_files(
  DIRECTORY srv
  FILES vsep_srv.srv
)

#add_message_files(
#DIRECTORY msg
#FILES )

#add_action_files(
#DIRECTORY action
#FILES 
#)


generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)

