cmake_minimum_required(VERSION 3.17)
project(CAL_MP1_ALGO)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_STANDARD 14)

set(GRAPHVIEWERCPP_PATH lib/GraphViewerCpp)
add_subdirectory(${GRAPHVIEWERCPP_PATH})
include_directories(${GRAPHVIEWERCPP_PATH}/include)

add_executable(CAL_MP1_ALGO main.cpp client.cpp productprovider.cpp node.cpp deliveryCar.cpp utils.h utils.cpp planRoute.cpp planRoute.h)

target_link_libraries(CAL_MP1_ALGO graphviewer)