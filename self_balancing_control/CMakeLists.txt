cmake_minimum_required(VERSION 3.0.2)
project(self_balancing_control)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
