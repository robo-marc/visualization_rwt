cmake_minimum_required(VERSION 2.8.3)
project(rwt_image_view_marc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roswww
  rwt_utils_3rdparty
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS rospy rwt_utils_3rdparty std_srvs roswww message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/rosbag_record_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY www/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www
)