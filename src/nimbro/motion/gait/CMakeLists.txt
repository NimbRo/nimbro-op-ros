cmake_minimum_required(VERSION 2.8.3)
project(gait)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp genmsg std_msgs robotcontrol nimbro_op_model)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

include_directories(src/gaitcode)

find_package(Qt4 REQUIRED QtCore QtXml QtOpenGL)
include(${QT_USE_FILE})

find_library(QGLVIEWER_LIBRARY qglviewer-qt4 REQUIRED)
find_path(QGLVIEWER_INCLUDE manipulatedFrame.h PATH_SUFFIXES QGLViewer REQUIRED)

find_library(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${QGLVIEWER_INCLUDE})

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	GaitPerformance.msg
    GaitCommand.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetOdom.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(gait
  src/gait.cpp
  src/gaitcode/Command.cpp
  src/gaitcode/Config.cpp
  src/gaitcode/State.cpp
  src/gaitcode/RobotModel/RobotModel.cpp
  src/gaitcode/RobotControl/RobotControl.cpp
  src/gaitcode/RobotControl/MotionLayer/DynamicGait.cpp
  src/gaitcode/RobotControl/MotionLayer/Fall.cpp
  src/gaitcode/RobotControl/MotionLayer/Halt.cpp
  src/gaitcode/RobotControl/MotionLayer/Motion.cpp
  src/gaitcode/RobotControl/MotionLayer/MotionLayer.cpp
  src/gaitcode/RobotControl/MotionLayer/MotionTest.cpp
  src/gaitcode/RobotControl/MotionLayer/MotionInterfaceTest.cpp
  #src/gaitcode/util/GLlib.cpp
  src/gaitcode/util/StopWatch_linux.cpp
  src/gaitcode/util/Vec2f.cpp
  src/gaitcode/util/Vec3f.cpp
  src/configclient/configclient.cpp
  src/configclient/configitem.cpp
)

add_dependencies(gait gait_gencpp)

## Declare a cpp executable
# add_executable(gait src/gait.cpp)

## Add dependencies to the executable
# add_dependencies(gait_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(gait
  ${catkin_LIBRARIES}
  ${QGLVIEWER_LIBRARY}
)

#############
## Install ##
#############

install(TARGETS gait
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS gait gait_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gait.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
