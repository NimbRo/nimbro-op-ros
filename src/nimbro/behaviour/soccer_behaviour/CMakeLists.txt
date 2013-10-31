cmake_minimum_required(VERSION 2.8.3)
project(soccer_behaviour)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-address")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	behaviour_control state_controller test_utilities
	robotcontrol roscpp config_server tf plot_msgs gait
	head_control field_model utilities
)

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
#  std_msgs  # Or other packages containing msgs
#)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#	INCLUDE_DIRS
#	LIBRARIES
#	CATKIN_DEPENDS
#       DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_executable(soccer_behaviour
	src/soccer_behaviour.cpp
	src/soccer_manager/soccer_manager.cpp
	src/ros_interface_layer/ros_interface_layer.cpp
	src/soccer_layer/soccer_layer.cpp
	src/soccer_layer/play_soccer.cpp
	src/control_layer/control_layer.cpp
	src/control_layer/game_control.cpp
	src/control_layer/kick.cpp
	src/control_layer/scoop_kick.cpp
	src/control_layer/dribble.cpp
	src/control_layer/go_behind_ball.cpp
	src/control_layer/search_for_ball.cpp
	src/control_layer/control_head.cpp
)

target_link_libraries(soccer_behaviour
	behaviour_control
	state_controller
	test_utilities
	${catkin_LIBRARIES}

)

#############
## Install ##
#############

install(TARGETS soccer_behaviour
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
