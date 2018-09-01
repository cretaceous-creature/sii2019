cmake_minimum_required(VERSION 2.8.3)
project(charge_robot)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

find_package(gazebo 9 REQUIRED sensor_msgs)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs)

catkin_package(
 CATKIN_DEPENDS gazebo_ros
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})


install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
