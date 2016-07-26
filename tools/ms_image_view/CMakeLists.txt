cmake_minimum_required(VERSION 2.8.3)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

find_package(OpenCV REQUIRED)

###########
## Build ##
###########

## image_view
rosbuild_add_executable(image_view nodes/image_view.cpp)
target_link_libraries(image_view
${OpenCV_LIBS} )


