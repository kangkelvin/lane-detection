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
    // _map_frames.emplace(filename, frame);

    VideoParser::cannyDetector(frame, frame);
    VideoParser::segmentRoi(frame, frame);

    imshow(filename, frame);

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

void laneDetection::VideoParser::cannyDetector(Mat &input, Mat &output) {
  cvtColor(input, output, COLOR_RGB2GRAY); // change to grayscale
  GaussianBlur(input, output, Size(5,5), 0.0); // apply blur
  Canny(input, output, 150.0, 150.0); // detects edges
}

void laneDetection::VideoParser::segmentRoi(Mat &input, Mat &output) {
  int frame_height = input.rows;
  int frame_width = input.cols;
  int frame_type = input.type();
  Mat mask = Mat::zeros(Size(frame_width, frame_height), frame_type);
  Point2i polygon[1][3];
  polygon[0][0] = Point2i(frame_width*0.2,frame_height*0.9);
  polygon[0][1] = Point2i(frame_width*0.45,frame_height*0.6);
  polygon[0][2] = Point2i(frame_width*0.7,frame_height*0.9);
  const Point2i* ppt[1] = {polygon[0]};
  int npt[] = {3};
  fillPoly(mask, ppt, npt, 1, 255);
  bitwise_and(input, mask, output);
}

