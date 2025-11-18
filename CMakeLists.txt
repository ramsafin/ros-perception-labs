cmake_minimum_required(VERSION 3.10)
project(ros_sync_buf_lab VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## packages ##
find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED
  rospy
  roscpp
  roslib      # evaluation of the ROS-package path
  cv_bridge   # conversion between OpenCV and ROS-messages
  sensor_msgs # Image ROS-message
)

## catkin specific configuration ##
catkin_package()
