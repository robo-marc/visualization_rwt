cmake_minimum_required(VERSION 2.8.3)
project(rwt_rosparam_marc)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbridge_server
  roscpp
  rospy
  rwt_utils_3rdparty
  rwt_utils_marc
  std_msgs
  message_generation  
  roswww
)

add_service_files(
   FILES
   DumpParams.srv
   LoadParams.srv
)
 
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rosbridge_server rospy std_msgs rwt_utils_3rdparty rwt_utils_marc roswww
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
