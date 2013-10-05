cmake_minimum_required(VERSION 2.8.3)
project(trajectory_editor)

find_package(catkin REQUIRED COMPONENTS roscpp rviz rbdl urdf motion_player nimbro_op_model)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_editor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

qt4_wrap_ui(UIFILES
	src/mainwindow.ui
	src/invkinbox.ui
)

qt4_wrap_cpp(MOCFILES
	src/mainwindow.h
	src/rvizwidget.h
	src/keyframemodel.h
	src/jointdisplay.h
	src/invkinbox.h
)

add_executable(trajectory_editor
	${UIFILES}
	${MOCFILES}
	src/main.cpp
	src/mainwindow.cpp
	src/rvizwidget.cpp
	src/robotdisplay.cpp
	src/keyframemodel.cpp
	src/jointdisplay.cpp
	src/invkinbox.cpp
	src/leg_ik.cpp
)

target_link_libraries(trajectory_editor
	${catkin_LIBRARIES}
	${QT_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trajectory_editor trajectory_editor_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory_editor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
