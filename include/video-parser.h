#pragma once

#include <ros/ros.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include <thread>
#include <memory>
#include <vector>

using namespace cv;
using std::string;
using std::vector;

namespace laneDetection {

class VideoParser {
 public:
  VideoParser();
  ~VideoParser() {};
  ros::NodeHandle nh;

 private:
  ros::NodeHandle private_nh;

  ros::Timer _video_timer;

  image_transport::ImageTransport it;
  image_transport::Publisher _output_img_pub;

  string _video_filename;
  string _output_img_topic;

  std::unique_ptr<VideoCapture> _video;
  Mat _frame;
  sensor_msgs::ImagePtr _output_img;

  int _video_fps;
  int _video_delay;
  int _frame_width;
  int _frame_height;
  int _frame_type;

  bool _is_stereo_img;

  void getRosParam();
  void getVidParam();
  void setPubSub();
  void pubFrame(const ros::TimerEvent& timer_event);
};

}  // namespace laneDetection