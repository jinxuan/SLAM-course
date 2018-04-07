cmake_minimum_required( VERSION 2.8 )
project( imageBasics )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS})


add_executable( undistort_image undistort_image.cpp )
# 链接OpenCV库
target_link_libraries( undistort_image ${OpenCV_LIBS} )

add_executable(disparity disparity.cpp)

target_link_libraries(disparity ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES})