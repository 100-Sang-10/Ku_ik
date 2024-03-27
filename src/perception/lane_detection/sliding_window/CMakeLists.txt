cmake_minimum_required(VERSION 3.0.2)
project(sliding_window)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  carla_msgs
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  image_transport
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  carla_msgs  geometry_msgs   nav_msgs   sensor_msgs   std_msgs  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES sliding_window
  CATKIN_DEPENDS carla_msgs cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf visualization_msgs
  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(sliding_window_node src/sliding_window.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(
                sliding_window_node
                ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(sliding_window_node
                ${catkin_LIBRARIES}
                ${OpenCV_LIBRARIES})
                  

