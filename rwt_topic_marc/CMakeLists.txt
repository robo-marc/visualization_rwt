cmake_minimum_required(VERSION 2.8.3)
project(rwt_topic_marc)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  rospy
  rwt_utils_3rdparty
  rwt_utils_marc
  std_msgs
  message_generation  
  roswww
)


catkin_package(
  CATKIN_DEPENDS rosbridge_server rospy std_msgs rwt_utils_3rdparty rwt_utils_marc roswww
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
