cmake_minimum_required(VERSION 3.0.2)
project(emotion_detection_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
