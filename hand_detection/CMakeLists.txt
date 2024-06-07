cmake_minimum_required(VERSION 3.0.2)
project(hand_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  Rectangle.msg
  ImageWithRectangles.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/hand_detection_node.py
  src/point_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
