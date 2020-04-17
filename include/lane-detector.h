#pragma once

#include <ros.h>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <thread>
#include <vector>
#include <lane_detector/LaneInfo.h>

#include "sensor_msgs/Image.h"

using namespace cv;

namespace laneDetection {

class LaneDetector {
 public:
  LaneDetector();
  ~LaneDetector();

  ros::NodeHandle nh;

 private:
  ros::NodeHandle private_nh;

  std::vector<std::thread> _threads;

  Mat _original_img;
  Mat _processed_img;

  double _gauss_blur_size[2];
  double _gauss_blur_sigmaX;

  double _canny_threshold1;
  double _canny_threshold2;

  double _roi_polygon[1][3];

  double _hough_rho;
  double _hough_theta;
  double _hough_thershold;
  double _hough_min_line_length;
  double _hough_max_line_gap;
  
  int _video_fps;
  int _frame_width;
  int _frame_height;
  int _frame_type;
  
  Vec2d _left_lane_avg_param;
  Vec2d _right_lane_avg_param;

  std::string _input_img_topic;
  std::string _output_img_topic;
  std::string _lane_result_topic;
  ros::Subscriber _input_img_sub;
  ros::Publisher _output_img_pub;
  ros::Publisher _lane_result_pub;

  void getRosParam();
  void getVidParam();
  void setPubSub();
  void detectLane(const sensor_msgs::Image& msg);

  void cannyDetector(Mat &input, Mat &output);
  void segmentRoi(Mat &input, Mat &output);
  void calcHoughLines(Mat &input, Mat &output);
  void calcGradientIntercept(Vec4i &line, std::vector<Vec2d> &left_lanes,
                             std::vector<Vec2d> &right_lanes);
  Vec2d calcVec2dAverage(std::vector<Vec2d> &vec);
  Mat getVisualisedLines(Mat &input, Vec2d &lane_info);
  bool withinRange(double input, double lower_bound, double upper_bound);

};

}  // namespace laneDetection