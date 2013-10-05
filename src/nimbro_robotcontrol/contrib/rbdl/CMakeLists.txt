

cmake_minimum_required(VERSION 2.8.3)
project(rbdl)

find_package(catkin REQUIRED COMPONENTS urdf)

add_subdirectory(source)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/source/include)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/source/CMake )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(rbdl_ros
	src/rbdl_parser.cpp
	src/treestream.cpp
)

target_link_libraries(rbdl_ros
	rbdl
)

catkin_package(
	INCLUDE_DIRS include source/include ${CMAKE_CURRENT_BINARY_DIR}/source/include ${EIGEN3_INCLUDE_DIR}
	LIBRARIES rbdl rbdl_ros
)

install(TARGETS rbdl_ros
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Add gtest based cpp test target and link libraries

configure_file(test/test_config.h.in test/test_config.h @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

catkin_add_gtest(test_rbdl_parser
	test/test_rbdl_parser.cpp
)
target_link_libraries(test_rbdl_parser
	${catkin_LIBRARIES}
	rbdl_ros
	rbdl
)
