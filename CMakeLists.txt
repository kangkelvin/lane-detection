cmake_minimum_required(VERSION 2.8.3)
project(lane-detection)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  image_transport
  cv_bridge
  roscpp
  rospy
  std_msgs
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Add project executable
# add_executable(lane-detection ${project_SRCS})
# target_link_libraries(lane-detection ${OpenCV_LIBRARIES})

add_executable(video-parser src/video-parser.cpp)
target_link_libraries(video-parser ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
