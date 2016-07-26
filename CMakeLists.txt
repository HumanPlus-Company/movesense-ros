cmake_minimum_required(VERSION 2.8.3)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

## pkg-config libraries
find_package(PkgConfig REQUIRED)

## movesense libraries
find_package(MOVESENSE REQUIRED)

## YAML libraries
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)


###########
## Build ##
###########

## include
include_directories(include
  ${MOVESENSE_INCLUDE_DIR}
)

## Declare a cpp executable
## LR MODE
rosbuild_add_executable(${PROJECT_NAME}_LR_node nodes/LR_MODE.cpp)
target_link_libraries(${PROJECT_NAME}_LR_node
  ${MOVESENSE_LIBRARY}
  udev
  yaml-cpp
)

## LD MODE
rosbuild_add_executable(${PROJECT_NAME}_LD_node nodes/LD_MODE.cpp)
target_link_libraries(${PROJECT_NAME}_LD_node
  ${MOVESENSE_LIBRARY}
  udev
  yaml-cpp
)

## LRD MODE
rosbuild_add_executable(${PROJECT_NAME}_LRD_node nodes/LRD_MODE.cpp)
target_link_libraries(${PROJECT_NAME}_LRD_node
  ${MOVESENSE_LIBRARY}
  udev
  yaml-cpp
)

