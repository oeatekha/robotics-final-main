cmake_minimum_required(VERSION 1.0.0)
project(user_input)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  actionlib
  sensor_msgs
  trajectory_msgs
  std_msgs
  cv_bridge
  message_filters
  message_generation
)
add_message_files(DIRECTORY msg
  FILES
  Velocity.msg
  JoyCmd.msg
  Voxel.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  sensor_msgs
  trajectory_msgs
  std_msgs
  cv_bridge
  message_filters
  message_runtime
  message_generation
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

