cmake_minimum_required(VERSION 2.8.3)
project(tum_executioner)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs genmsg dynamic_reconfigure)

## add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/ExParams.cfg)

## Declare ROS messages and services
add_message_files(FILES ExecMsg.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ExecNode src/ExecNode.cpp)
target_link_libraries(ExecNode ${catkin_LIBRARIES})
add_dependencies(ExecNode ${PROJECT_NAME}_gencfg tum_executioner_generate_messages_cpp)
