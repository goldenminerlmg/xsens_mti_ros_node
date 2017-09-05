cmake_minimum_required(VERSION 2.8.3)
project(xsens_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs tf sensor_msgs geometry_msgs gps_common diagnostic_msgs xsens_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_py
  CATKIN_DEPENDS rospy std_msgs tf sensor_msgs geometry_msgs gps_common diagnostic_msgs xsens_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  src/mtnode.py
  src/mtdef.py
  src/mtdevice.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

