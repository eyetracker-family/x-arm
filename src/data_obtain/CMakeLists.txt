cmake_minimum_required(VERSION 2.8.3)
project(data_obtain)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV 3.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(error_analysis_node
	src/error_analysis.cpp)
link_directories(${OpenCV_LIB_DIR})
target_link_libraries(error_analysis_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  )
