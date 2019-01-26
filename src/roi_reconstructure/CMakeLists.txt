cmake_minimum_required(VERSION 2.8.3)
project(roi_reconstructure)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#set(OpenCV_DIR /home/x-arm/installed-software/opencv3.4/share/OpenCV)
#set(OpenCV_DIR /home/x-arm/github/opencv/install/share/OpenCV)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  eyetracking_msgs	##to use eyetracking_msgs,add this line
  darknet_ros_msgs
  image_transport
  cv_bridge
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
## System dependencies are found with CMake's convention
find_package(OpenCV 3.3 REQUIRED)

find_package(PCL 1.2 REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)

set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "")


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roi_reconstructure
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  "/home/x-arm/macaca/src/roi_reconstructure/include/roi_reconstructure"
)

add_definitions(${PCL_DEFINITIONS})

add_executable(roi_reconstructure_node
	src/roi_reconstructure.cpp)
#link_directories(${OpenCV_LIB_DIR} ${PCL_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(roi_reconstructure_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  )
