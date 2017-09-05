cmake_minimum_required(VERSION 2.8.3)
project(xsens_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy roscpp std_msgs message_generation sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   XsensQuaternion.msg
   ImuSensorSample.msg
   BaroSensorSample.msg   
   GnssSensorSample.msg
   Internal.msg
   sensorSample.msg
   baroSample.msg
   gnssSample.msg
   orientationEstimate.msg
   velocityEstimate.msg
   positionEstimate.msg		   
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_py
  CATKIN_DEPENDS std_msgs sensor_msgs message_generation message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
