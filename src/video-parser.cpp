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
        std::thread(&VideoParser::process, this, _filenames[i]));
  }
}

Mat laneDetection::VideoParser::getFrame(std::string filename) {
  return _map_frames.at(filename);
}

void laneDetection::VideoParser::process(std::string filename) {
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

    VideoParser::cannyDetector(frame, frame);
    VideoParser::segmentRoi(frame, frame);
    VideoParser::calcHoughLines(frame);

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
  cvtColor(input, output, COLOR_RGB2GRAY);       // change to grayscale
  GaussianBlur(input, output, Size(5, 5), 0.0);  // apply blur
  Canny(input, output, 150.0, 150.0);            // detects edges
}

void laneDetection::VideoParser::segmentRoi(Mat &input, Mat &output) {
  int frame_height = input.rows;
  int frame_width = input.cols;
  int frame_type = input.type();
  Mat mask = Mat::zeros(Size(frame_width, frame_height), frame_type);
  Point2i polygon[1][3];
  polygon[0][0] = Point2i(frame_width * 0.2, frame_height * 0.9);
  polygon[0][1] = Point2i(frame_width * 0.45, frame_height * 0.6);
  polygon[0][2] = Point2i(frame_width * 0.7, frame_height * 0.9);
  const Point2i *ppt[1] = {polygon[0]};
  int npt[] = {3};
  fillPoly(mask, ppt, npt, 1, 255);
  bitwise_and(input, mask, output);
}

void laneDetection::VideoParser::calcHoughLines(Mat &input) {
  std::vector<Vec4i> lines;
  HoughLinesP(input, lines, 1, CV_PI / 180, 50, 50, 10);

  std::vector<Vec2f> left_lanes;
  std::vector<Vec2f> right_lanes;
  std::vector<std::thread> threads;

  std::mutex mtx;

  for (size_t i = 0; i < lines.size(); ++i) {
    // threads.emplace_back(std::thread(&VideoParser::calcGradientIntercept,
    //                                  this, lines[i], left_lanes,
    //                                  right_lanes));
    calcGradientIntercept(lines[i], left_lanes, right_lanes);
  }

  // std::for_each(threads.begin(), threads.end(),
  //               [](std::thread &t) { t.join(); });

  Vec2f avg_left_lane = calcVec2fAverage(left_lanes);
  Vec2f avg_right_lane = calcVec2fAverage(right_lanes);

  std::cout << "LEFT\n#######################"
               "######\n";
  for_each(left_lanes.begin(), left_lanes.end(),
           [](Vec2f &line) { std::cout << line << std::endl; });
  std::cout << "RIGHT\n#######################"
               "######\n";
  for_each(right_lanes.begin(), right_lanes.end(),
           [](Vec2f &line) { std::cout << line << std::endl; });
}

void laneDetection::VideoParser::calcGradientIntercept(
    Vec4i &line, std::vector<Vec2f> &left_lanes,
    std::vector<Vec2f> &right_lanes) {
  float gradient = (line[3] - line[1]) * 1.0 / (line[2] - line[0]);
  float intercept = line[1] - gradient * line[0];
  Vec2f output = {gradient, intercept};
  if (gradient > 0) {
    right_lanes.push_back(output);
  } else {
    left_lanes.push_back(output);
  }
}

Vec2f laneDetection::VideoParser::calcVec2fAverage(std::vector<Vec2f> &vec) {
  Vec2f output;
  if (!vec.empty()) {
    float first_average = 0;
    float second_average = 0;
    for (int i = 0; i < vec.size(); ++i) {
      first_average += vec[i][0];
      second_average += vec[i][1];
    }
    first_average /= vec.size();
    second_average /= vec.size();
    output = Vec2f(first_average, second_average);
  }
  return output;
}
