cmake_minimum_required(VERSION 2.8.3)
project(lane_detection)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  image_transport
  message_generation
  cv_bridge
  roscpp
  rospy
  std_msgs
  geometry_msgs
  dynamic_reconfigure
)

add_message_files(
  FILES
  LaneInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  include 
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Add project executable
add_executable(lane_detector src/lane-detector.cpp)
target_link_libraries(lane_detector ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(lane_detector ${catkin_EXPORTED_TARGETS})

add_executable(video_parser src/video-parser.cpp)
target_link_libraries(video_parser ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
