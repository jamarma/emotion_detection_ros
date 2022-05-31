cmake_minimum_required(VERSION 3.0.2)
project(emotion_detection_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()
catkin_package()

install(DIRECTORY models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)