cmake_minimum_required(VERSION 3.0.2)
project(lab_bug)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
)

add_service_files(
  FILES
  SetBugGoal.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
