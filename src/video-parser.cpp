#include "video-parser.h"

using namespace cv;

laneDetection::VideoParser::VideoParser(std::vector<std::string> filenames)
    : _filenames(filenames) {}

laneDetection::VideoParser::~VideoParser() {
  // set up thread barrier before this object is destroyed
  std::for_each(_threads.begin(), _threads.end(),
                [](std::thread &t) { t.join(); });
}

void laneDetection::VideoParser::run() {
  for (std::size_t i = 0; i < _filenames.size(); ++i) {
    _threads.emplace_back(
        std::thread(&VideoParser::display, this, _filenames[i]));
  }
}

Mat laneDetection::VideoParser::getFrame(std::string filename) {
  return _map_frames.at(filename);
}

void laneDetection::VideoParser::display(std::string filename) {
  VideoCapture video(filename);
  cv::namedWindow(filename, cv::WINDOW_NORMAL);

  int video_fps = (int)video.get(cv::CAP_PROP_FPS);
  int delay = 1000 / video_fps;

  while (1) {
    clock_t start_time = clock();

    Mat frame;
    bool isOpen = video.read(frame);
    frame = frame(Range::all(), Range(1, frame.cols / 2));

    if (!isOpen) {
      std::cout << "End of Video\n";
      break;
    }

    imshow(filename, frame);
    _map_frames.emplace(filename, frame);

    if (cv::waitKey(10) == 27) {
      std::cout << "break\n";
      break;
    }

    while (clock() - start_time < delay) {
      waitKey(1);
    }
  }
  video.release();
  destroyWindow(filename);
}