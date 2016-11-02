CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
project(EigenOpencvLearn)


set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/x/opencv-3.1.0 )

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
set(REQUIRED_LIBRARIES ${OpenCV_LIBS} )

MESSAGE(STATUS "OpenCV Version= ${OpenCV_VERSION}" )
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS= ${OpenCV_INCLUDE_DIRS}" )


find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR= ${EIGEN3_INCLUDE_DIR}" )


add_subdirectory(src)



MESSAGE(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
