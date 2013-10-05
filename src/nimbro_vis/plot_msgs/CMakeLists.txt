cmake_minimum_required(VERSION 2.8.3)
project(plot_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
	FILES
	PlotPoint.msg
	Plot.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

catkin_package(
#    INCLUDE_DIRS include
#  LIBRARIES plot_msgs
#  CATKIN_DEPENDS genmsg roscpp
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

