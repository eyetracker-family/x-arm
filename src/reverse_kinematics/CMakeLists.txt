cmake_minimum_required(VERSION 2.8.3)
project(reverse_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV 3.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  serial
  std_msgs
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  message_generation
)


add_message_files(
  FILES
  solution.msg
  solutions.msg
)

add_service_files(
  FILES
  querySolutions.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES communication
  CATKIN_DEPENDS message_runtime roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

set(Inverse_INCLUDE_DIRS "include/reverse_kinematics")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Inverse_INCLUDE_DIRS}
)

set(inverse_SRC 
	src/moveit_angle_compensation.cpp
	src/addObject.cpp
	src/collisionCheck.cpp)

add_executable(reverse_node ${inverse_SRC})
link_directories(${OpenCV_LIB_DIR})
target_link_libraries(reverse_node ${catkin_LIBRARIES}  ${OpenCV_LIBS})

