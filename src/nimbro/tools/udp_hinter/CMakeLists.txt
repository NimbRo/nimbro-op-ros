cmake_minimum_required(VERSION 2.8.3)
project(udp_hinter)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
)

## Declare a cpp library

find_package(roscpp REQUIRED)

add_custom_command(
	OUTPUT udp_hinter_config.h
	DEPENDS ${roscpp_LIBRARIES} src/genconf.sh src/udp_hinter_dummy.cpp
	COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/src/genconf.sh ${CMAKE_CURRENT_SOURCE_DIR}/src/udp_hinter_dummy.cpp > ${CMAKE_CURRENT_BINARY_DIR}/udp_hinter_config.h
	COMMENT "Determining mangled method names"
)

set_source_files_properties(src/udp_hinter.cpp
	OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/udp_hinter_config.h
)

add_library(udp_hinter
	src/udp_hinter.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(udp_hinter
	${catkin_LIBRARIES}
	dl
)

install(TARGETS udp_hinter
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
