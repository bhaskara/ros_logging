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

set(ALL_WARNINGS -Wall -Werror -Wextra -Wno-unused-but-set-variable)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_library(ros_logging
  src/mongo_logger.cpp
  src/query_results.cpp)

rosbuild_add_executable(rosout_logger src/rosout_logger_node.cpp)
target_link_libraries(rosout_logger ros_logging)

rosbuild_add_executable(grep_log src/grep_log.cpp)
target_link_libraries(grep_log ros_logging)
rosbuild_link_boost(grep_log program_options)

set(TARGETS ros_logging rosout_logger grep_log)

foreach(target ${TARGETS})
  rosbuild_add_compile_flags(${target} ${ALL_WARNINGS})
endforeach(target)