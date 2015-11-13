cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_io)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
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
  INCLUDE_DIRS
    include
#  LIBRARIES
  CATKIN_DEPENDS
    pcl_ros
    roscpp
    sensor_msgs
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

## Declare cpp executables
add_executable(
  read
  src/read_node.cpp
  src/Read.cpp
)
add_executable(
  write
  src/write_node.cpp
  src/Write.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(
  read
  ${catkin_LIBRARIES}
)
target_link_libraries(
  write
  ${catkin_LIBRARIES}
)
