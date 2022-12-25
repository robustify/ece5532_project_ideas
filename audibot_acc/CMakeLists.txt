cmake_minimum_required(VERSION 3.0.2)
project(audibot_acc)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
    rviz
    urdf
    worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
