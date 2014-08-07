cmake_minimum_required(VERSION 2.8.3)
project(ply_publisher)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES
    CATKIN_DEPENDS sensor_msgs
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}
  src/ply_publisher_node.cpp
  src/PlyPublisher.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
