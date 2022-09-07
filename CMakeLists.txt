cmake_minimum_required(VERSION 3.0.2)
project(exp_assignment3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  DEPENDS OpenCV
)

###########
## Build ##
###########
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/ball_tracking.py
  src/behaviour_controller.py
  src/human_interaction_gen.py
  src/motion_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#add_dependencies(exp_assignment3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(exp_assignment3 ${catkin_LIBRARIES})

#############
## Install ##
#############

# install nodes
#install(TARGETS exp_assignment3
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# install roslaunch files
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)

  # test all launch files
#  roslaunch_add_file_check(launch)
#endif()
