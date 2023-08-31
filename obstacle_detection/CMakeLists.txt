cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs  
  message_generation
  obstacle_world
  sensor_msgs
  std_msgs
  visualization_msgs)

add_message_files(
  FILES
  Obstacles.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime obstacle_world sensor_msgs std_msgs visualization_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS})
