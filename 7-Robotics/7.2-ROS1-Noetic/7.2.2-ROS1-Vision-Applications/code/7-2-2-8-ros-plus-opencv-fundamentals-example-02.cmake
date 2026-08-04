cmake_minimum_required(VERSION 3.0.2)
project(ros_opencv_demo)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
