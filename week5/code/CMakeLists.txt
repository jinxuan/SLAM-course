cmake_minimum_required( VERSION 2.8 )
project( week5 )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 


add_executable( computeORB computeORB.cpp )
target_link_libraries( computeORB ${OpenCV_LIBS} )
