cmake_minimum_required(VERSION 2.8.3)
project(camera_v4l2)

find_package(catkin REQUIRED COMPONENTS roscpp calibration nodelet pluginlib)

catkin_package(
	LIBRARIES camera_v4l2
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(camera_v4l2
	camera_v4l2.cpp
	configmanager.cpp
)

target_link_libraries(camera_v4l2
	${catkin_LIBRARIES}
	yaml-cpp
)

install(TARGETS camera_v4l2
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
