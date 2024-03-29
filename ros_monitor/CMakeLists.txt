cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_gtest(test_gui test/test_gui.cpp)

find_package(Qt4 REQUIRED)

set(SOURCES
  src/ros_monitor.cpp
  src/display_delegates.cpp
  src/actions.cpp
  src/rosout_model.cpp)

set(HEADERS
  include/ros_monitor/actions.h
  include/ros_monitor/ros_monitor.h
  include/ros_monitor/rosout_model.h
  include/ros_monitor/display_delegates.h)

QT4_WRAP_CPP(HEADERS_MOC ${HEADERS})

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

rosbuild_add_executable(ros_monitor ${SOURCES} ${HEADERS_MOC})
target_link_libraries(ros_monitor ${QT_LIBRARIES})
target_link_libraries(ros_monitor boost_system)
rosbuild_add_compile_flags(ros_monitor -Wno-unknown-pragmas)
