cmake_minimum_required( VERSION 2.8)

project( LuoboGeometry )

set( CMAKE_BUILD_TYPE "Release")

include_directories( "/usr/include/eigen3" )

add_executable(luoboGeometry luoboGeometry.cpp)

