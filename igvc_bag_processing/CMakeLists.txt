cmake_minimum_required(VERSION 3.0.2)
project(igvc_bag_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  image_geometry
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pinhole_geometry
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pinhole_geometry
  src/PinholeGeometry.cpp
)
target_link_libraries(pinhole_geometry
  ${catkin_LIBRARIES}
)

add_executable(pinhole_geometry_example
  src/pinhole_geometry_example.cpp
)
target_link_libraries(pinhole_geometry_example
  pinhole_geometry
  ${catkin_LIBRARIES}
)