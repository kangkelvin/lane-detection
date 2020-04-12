#include <thread>

#include "video-parser.h"

int main() {
  std::vector<std::string> video_filenames = {
      "../data/WIN_20200214_07_16_30_Pro.mp4"};

  laneDetection::VideoParser parser_obj(video_filenames);
  parser_obj.run();



  return 0;
}