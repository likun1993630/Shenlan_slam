set(Sophus_DIR ~/Sophus/build)
set (CMAKE_CXX_STANDARD 11)

cmake_minimum_required ( VERSION 2.8 )

project ( trajectory )

find_package ( Sophus  REQUIRED)
find_package(Pangolin)

#include_directories ("/usr/include/eigen3")
include_directories ( ${Sophus_INCLUDE_DIRS} )
include_directories ( ${Pangolin_INCLUDE_DIRS} )

add_executable ( trajectory draw_trajectory.cpp)

target_link_libraries( trajectory ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )