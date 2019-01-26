cmake_minimum_required(VERSION 2.8.3)
project(macaca_eyescene_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #ar_track_alvar
  message_generation
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  image_transport
  cv_bridge
  )

find_package(OpenCV 3.3)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


 generate_dynamic_reconfigure_options(
   cfg/ColorBlock.cfg
 )

catkin_package(
    #INCLUDE_DIRS include
    LIBRARIES macaca_eyescene_map
    CATKIN_DEPENDS message_generation roscpp rospy std_msgs eyetracking_msgs dynamic_reconfigure
    #DEPENDS system_lib
  )

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake) 

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )


add_executable(colorblock src/colorblock.cpp)
target_link_libraries(colorblock
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )

add_dependencies(colorblock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

