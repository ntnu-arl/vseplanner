cmake_minimum_required(VERSION 2.8.3)
project(kdtree)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME} src/kdtree.c)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
)


cs_add_executable(kdtree_example src/kdtree_test.cpp)
target_link_libraries(kdtree_example ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()
