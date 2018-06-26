cmake_minimum_required(VERSION 2.8.3)
project(ros_speech2text)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

## ROS messages, services and actions

add_message_files(FILES transcript.msg event.msg start_utterance.msg)

generate_messages(DEPENDENCIES std_msgs)

## ROS package

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

## Test

if(CATKIN_ENABLE_TESTING)
  # Need to encrypt google cloud credentials before automating these tests
  # find_package(rostest REQUIRED)
  # add_rostest(test/test_ros_speech2text.test)
  catkin_add_nosetests(test/test_speech_detection.py)
endif()

## Install

install(PROGRAMS
  scripts/ros_speech2text.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
