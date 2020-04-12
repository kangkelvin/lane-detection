cmake_minimum_required(VERSION 3.7)
project(lane-detection)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

find_package(OpenCV 4.1 REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Find all executables
file(GLOB project_SRCS src/*.cpp include/*.h)

# Add project executable
add_executable(lane-detection ${project_SRCS})
target_link_libraries(lane-detection ${OpenCV_LIBRARIES})
