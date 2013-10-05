cmake_minimum_required(VERSION 2.8.3)
project(robotcontrol)

# TODO: Place this somewhere top-level
if(NOT CMAKE_BUILD_TYPE)
	message(STATUS "Build type not set - using RelWithDebInfo per default")
	set(CMAKE_BUILD_TYPE "RelWithDebInfo")
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
		"Choose the type of build, options are: Debug Release
		RelWithDebInfo MinSizeRel."
		FORCE)
endif()

if(CMAKE_CXX_FLAGS STREQUAL "")
	set(CMAKE_CXX_FLAGS "-Wall" CACHE STRING
		"Flags used by the compiler during all build types."
		FORCE
	)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	genmsg timer nimbro_op_model servomodel
	roscpp pluginlib rbdl actionlib actionlib_msgs
	config_server tf plot_msgs eigen_conversions
	tf_conversions
)

find_package(cm730 REQUIRED)

set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS regex)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/robotcontrol)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  Diagnostics.msg
  Button.msg
  State.msg
  CompassHeading.msg
  ServoDiag.msg
  LEDCommand.msg
)

add_action_files(
  FILES
  FadeTorque.action
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ReadOffset.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES robotmodel
	CATKIN_DEPENDS config_server rbdl
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${cm730_INCLUDE_DIRS}
)

## Declare a cpp executable
add_library(robotmodel SHARED
	src/model/joint.cpp
	src/model/robotmodel.cpp
	src/model/singlesupportmodel.cpp
	src/model/golay.cpp
	src/model/kalmanfilter.cpp
	src/motion_utils/trunk_stabilizer.cpp
)

target_link_libraries(robotmodel
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(robotcontrol
	src/robotcontrol.cpp
	src/motionmodule.cpp
)

add_library(robotinterface
	src/hw/robotinterface.cpp
	src/hw/dynamiccommandgenerator.cpp
	src/hw/dummyinterface.cpp
	src/hw/angleestimator.cpp
	src/hw/compassfilter.cpp
)

target_link_libraries(robotinterface
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${cm730_LIBRARIES}
	robotmodel
)

## Specify libraries to link a library or executable target against
target_link_libraries(robotcontrol
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	robotmodel
)

add_dependencies(robotcontrol
	robotcontrol_gencpp
)

#############
## Install ##
#############

install(TARGETS robotmodel robotinterface robotcontrol
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_golay
	test/test_golay.cpp
	src/model/golay.cpp
)
target_link_libraries(test_golay
	${catkin_LIBRARIES}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
