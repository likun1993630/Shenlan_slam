cmake_minimum_required( VERSION 2.8 )

project( linearequation )

set( CMAKE_BUILD_TYPE "Release")

include_directories( "/usr/include/eigen3" )

add_executable( LinearEquation LinearEquation.cpp )
