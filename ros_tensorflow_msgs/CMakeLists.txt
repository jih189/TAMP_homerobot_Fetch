cmake_minimum_required(VERSION 2.8.3)
project(ros_tensorflow_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
  Predict.srv
  ComPredict.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
