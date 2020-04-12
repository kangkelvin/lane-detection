#pragma once

#include <chrono>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

using namespace cv;

namespace laneDetection {

class VideoParser {
 public:
  VideoParser(std::vector<std::string> filenames);
  ~VideoParser();

  void run();
  void process(std::string filename);
  Mat getFrame(std::string filename);
  void cannyDetector(Mat &input, Mat &output);
  void segmentRoi(Mat &input, Mat &output);
  void calcHoughLines(Mat &input);
  void calcGradientIntercept(Vec4i &line, std::vector<Vec2f> &left_lanes,
                                  std::vector<Vec2f> &right_lanes);
  Vec2f calcVec2fAverage(std::vector<Vec2f> &vec);
  Mat visualiseLines(Vec2f &left_lane, Vec2f &right_lane);

 private:
  std::vector<std::string> _filenames;
  std::vector<std::thread> _threads;
  std::unordered_map<std::string, Mat> _map_frames;
};

}  // namespace laneDetection