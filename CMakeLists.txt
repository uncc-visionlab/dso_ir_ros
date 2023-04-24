cmake_minimum_required(VERSION 2.8.12)
project(dso_ir_ros)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  sensor_msgs
  geometry_msgs
)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_VERBOSE_MAKEFILE ON)
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
#set(ROS_BUILD_TYPE Release)

#rosbuild_init()
#set(EXECUTABLE_OUTPUT_PATH bin)

set(DSO_LIBRARY dso)
set(DSO_PATH dso_ir)
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")
#add_subdirectory(dso_ir/thirdparty)
add_subdirectory(dso_ir)
find_package(Pangolin 0.8 REQUIRED)
# decide if we have pangolin
if (Pangolin_FOUND)
	message("--- found PANGOLIN, compiling dso_pangolin library.")
        include_directories(${DSO_PATH}/thirdparty/Pangolin/include)
	include_directories( ${Pangolin_INCLUDE_DIRS} ) 
	set(HAS_PANGOLIN 1)
else ()
	message("--- could not find PANGOLIN, not compiling dso_pangolin library.")
	message("    this means there will be no 3D display / GUI available for dso_dataset.")
	set(HAS_PANGOLIN 0)
endif ()

message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
#find_package(Boost COMPONENTS system thread) 
find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)

# SSE flags
#rosbuild_check_for_sse()
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)

set(SOURCE_FILES         
  src/main.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_PATH}/src
  ${DSO_PATH}/thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)  

#rosbuild_add_executable(dso_live src/main.cpp ${SOURCE_FILES})
#target_link_libraries(dso_live ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})
#rosbuild_link_boost(dso_live thread)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detect_depth_surflets
#  CATKIN_DEPENDS boost cv_bridge eigen geometry_msgs image_geometry image_transport nodelet pcl_ros std_msgs tf tf2 tf2_ros visualization_msgs
#  DEPENDS system_lib
#  DEPENDS Boost Eigen3 OpenCV 
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
        SYSTEM ${BOOST_INCLUDE_DIRS}
               ${catkin_INCLUDE_DIRS}
               ${EIGEN_INCLUDE_DIRS}
               ${OPENCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})