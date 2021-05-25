cmake_minimum_required(VERSION 3.5.1)
project(point_cloud_io)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options.
option(BUILD_WITH_VTK_SUPPORT "Build package with support for VTK files." OFF)
if(BUILD_WITH_VTK_SUPPORT)
  message("point_cloud_io will be built with support for VTK files.")
  add_definitions(-DHAVE_VTK)
endif()

set(CATKIN_PACKAGE_DEPENDENCIES
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########
add_executable(read
  src/read_node.cpp
  src/Read.cpp
)
target_link_libraries(read
  ${catkin_LIBRARIES}
)

add_executable(write
  src/write_node.cpp
  src/Write.cpp
)
target_link_libraries(write
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    read
    write
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY
    examples
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    TARGETS
      read
      write
  )
endif(cmake_clang_tools_FOUND)

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )
  add_dependencies(test_${PROJECT_NAME}
    read
    write
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${catkin_LIBRARIES}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif()
endif()