cmake_minimum_required(VERSION 2.8) 
PROJECT (bsnake_lane_detection)

add_definitions ("-w")
find_package(OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set( NAME_SRC
    src/laneDetector.cpp
    src/laneDetector_utils.cpp
    src/houghP.cpp
)

set( NAME_HEADERS       
     include/laneDetector_utils.h
     include/houghP.hpp
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(lane_detector src/laneDetector.cpp src/laneDetector_utils.cpp src/houghP.cpp)
target_link_libraries(lane_detector ${OpenCV_LIBRARIES})