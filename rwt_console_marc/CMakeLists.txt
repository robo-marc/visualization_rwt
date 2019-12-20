cmake_minimum_required(VERSION 2.8.3)
project(rwt_console_marc)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbridge_server
  rospy
  rwt_utils_3rdparty
  std_msgs
  roswww
  rwt_utils_marc
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rosbridge_server rospy std_msgs rwt_utils_3rdparty roswww rwt_utils_marc
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY www/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www
)
