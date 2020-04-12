#pragma once

#include <iostream>
#include <thread>
#include <chrono>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace cv;

namespace laneDetection {

class VideoParser {
 public:
  VideoParser(std::vector<std::string> filenames);
  ~VideoParser();

  void run();
  void display(std::string filename);
  Mat getFrame(std::string filename);
  void cannyDetector(Mat &input, Mat &output);
  void segmentRoi(Mat &input, Mat & output);

 private:
  std::vector<std::string> _filenames;
  std::vector<std::thread> _threads;
  std::unordered_map<std::string, Mat> _map_frames;
};

}  // namespace laneDetection