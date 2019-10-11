cmake_minimum_required( VERSION 2.8)

project( UseGeometry2 )

set( CMAKE_BUILD_TYPE "Release")

include_directories( "/usr/include/eigen3" )

add_executable(useGeometry2 useGeometry2.cpp)

