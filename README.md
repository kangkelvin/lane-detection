# Lane Detection
By Kelvin Kang

This is a lane detection with OpenCV canny edge and hough transform with ROS wrapper

![](data/screenshot.png)

## System Dependencies
* Ubuntu 18.04.4 LTS
* ROS Kinetic / Melodic
* cmake >= 2.8.3
* OpenCV >= 4.1

## Setup
1. Clone this repo under your workspace `~/catkin_ws/src`

2. Install ros dependencies under ~/catkin_ws 

   `rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y`

3. Install OpenCV

   https://www.learnopencv.com/install-opencv-4-on-ubuntu-18-04/

4. Go to ~/catkin_ws and run `catkin_make`

5. Edit `/launch/lane_detection.launch` file to select the video file of your input, under the param `video_filename`

6. Run the launch file `roslaunch lane_detection lane_detection.launch`

## Details

There's two nodes inside this package:

1. video-parser

   This node takes in an input video (e.g. .mp4 files) or video camera access that is connected to your computer and convert it to `sensor_msgs::Image` ROS topic

2. lane-detector

   This node takes an input `sensor_msgs::Image` topic and output another `sensor_msgs::Image` topic with the lane lines overlaid on the original image

A launch file is included with parameters to tune, located inside the /launch folder

## Rubric Points

| Rubric                                                       | File and line number                   |
| ------------------------------------------------------------ | -------------------------------------- |
| The project demonstrates an understanding of C++ functions and control structures. | video-parser.cpp lane-detector.cpp     |
| The project reads data from a file and process the data, or the program writes data to a file. | video-parser.cpp:23                    |
| The project accepts user input and processes the input.      | lane-detector.launch                   |
| The project uses Object Oriented Programming techniques.     | video-parser.cpp lane-detector.cpp     |
| Classes use appropriate access specifiers for class members. | video-parser.h lane-detector.h         |
| Class constructors utilize member initialization lists.      | video-parser.cpp:8 lane-detector.cpp:8 |
| Classes abstract implementation details from their interfaces. | video-parser.cpp lane-detector.cpp     |
| Classes encapsulate behavior.                                | video-parser.cpp lane-detector.cpp     |
| The project makes use of references in function declarations. | video-parser.cpp lane-detector.cpp     |
| The project uses scope / Resource Acquisition Is Initialization (RAII) where appropriate. | lane-detector.cpp:103                   |
| The project uses multithreading                              | lane-detector.cpp:72                   |
| A mutex or lock is used in the project                       | lane-detector.cpp:103                   |

