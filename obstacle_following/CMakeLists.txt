cmake_minimum_required(VERSION 3.0.2)
project(obstacle_following)

find_package(catkin REQUIRED COMPONENTS
  rospy geometry_msgs obstacle_detection obstacle_world)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS})
