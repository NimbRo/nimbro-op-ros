cmake_minimum_required(VERSION 2.8.3)
project(launch)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config hardware visualization complete cv behaviour motions
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
