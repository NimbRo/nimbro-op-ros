cmake_minimum_required(VERSION 2.8.3)
project(utilities)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rostest plot_msgs config_server)

# Find and include Eigen library (for compiling utilities_test - users of this package may or may not need Eigen3 depending on which headers they are using of this package)
find_library(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  FILES
#)

#add_action_files(
#  FILES
#)

# add_service_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  # Other packages containing msgs
#)

###################################################
## Declare things to be passed to other projects ##
###################################################

## INCLUDE_DIRS: include directories of this project that dependent projects also need
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
#	LIBRARIES
#	CATKIN_DEPENDS
#	DEPENDS
)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
)

#add_library(utilities
#	src/utilities.cpp
#)
#target_link_libraries(utilities
#	${catkin_LIBRARIES}
#)

###############
## Unit Test ##
###############

add_rostest_gtest(utilities_test
	test/utilities_test.test
	test/utilities_test.cpp
)
target_link_libraries(utilities_test
	${catkin_LIBRARIES}
	${GTEST_LIBRARIES}
)
add_dependencies(utilities_test plot_msgs_gencpp config_server_gencpp)

#############
## Install ##
#############

#install(TARGETS utilities
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
# EOF