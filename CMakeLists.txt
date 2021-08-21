cmake_minimum_required(VERSION 2.8.3)
project(terrain_treadmill)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aruco       # Minimum version 0.2.2
  aruco_msgs
  aruco_ros
  cv_camera
  rosserial_client
  rosserial_msgs
  rosserial_python
  message_generation
  geometry_msgs
  dynamic_reconfigure
  flea3
  camera_base
  image_view
  visualization_msgs
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Eulers.msg
   MotorVelocity.msg
   BallVelocity.msg
   TagState.msg
   ControlEffort.msg
)

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
   aruco_msgs
   rosserial_msgs
   geometry_msgs
   visualization_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/kalman.cfg
   cfg/motor_test.cfg
   cfg/PID.cfg
 )


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES terrain_treadmill
  CATKIN_DEPENDS aruco aruco_msgs aruco_ros cv_camera rosserial_client rosserial_msgs rosserial_python message_runtime geometry_msgs
#  DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
)
