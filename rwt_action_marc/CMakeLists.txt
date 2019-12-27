cmake_minimum_required(VERSION 2.8.3)
project(rwt_action_marc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  rwt_utils_marc
)

add_service_files(
   FILES
   ActionList.srv
)
 
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rosbridge_server rospy std_msgs rwt_utils_3rdparty roswww rwt_utils_marc
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
