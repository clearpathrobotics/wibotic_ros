cmake_minimum_required(VERSION 2.8.3)
project(wibotic_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Wibotic.msg
  RX.msg
  TX.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  wibotic_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS rospy std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
