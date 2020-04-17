#include "lane-detector.h"

using namespace cv;
using namespace laneDetection;
using std::string;
using std::vector;

LaneDetector::LaneDetector() : private_nh("~") {}

LaneDetector::~LaneDetector() {
  // set up thread barrier before this object is destroyed
  std::for_each(_threads.begin(), _threads.end(),
                [](std::thread &t) { t.join(); });
}

void LaneDetector::detectLane(const sensor_msgs::Image &msg) {
  _original_img = cv_bridge::toCvCopy(msg, "passthrough")->image;
  _original_img.copyTo(_processed_img);

  LaneDetector::cannyDetector(frame, frame);
  LaneDetector::segmentRoi(frame, frame);
  Mat lane_lines = LaneDetector::calcHoughLines(frame, original_frame);
  // addWeighted(original_frame, 0.9, lane_lines, 1, 1, original_frame);
}

void laneDetection::LaneDetector::cannyDetector(Mat &input, Mat &output) {
  cvtColor(input, output, COLOR_RGB2GRAY);       // change to grayscale
  GaussianBlur(input, output, Size(5, 5), 0.0);  // apply blur
  Canny(input, output, 150.0, 150.0);            // detects edges
}

void laneDetection::LaneDetector::segmentRoi(Mat &input, Mat &output) {
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

void laneDetection::LaneDetector::calcHoughLines(Mat &input, Mat &output) {
  std::vector<Vec4i> lines;
  HoughLinesP(input, lines, 1, CV_PI / 180, 50, 50, 10);

  std::vector<Vec2d> left_lanes;
  std::vector<Vec2d> right_lanes;
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

  Vec2d avg_left_lane = calcVec2dAverage(left_lanes);
  Vec2d avg_right_lane = calcVec2dAverage(right_lanes);

  Mat left_line = getVisualisedLines(original, avg_left_lane);
  Mat right_line = getVisualisedLines(original, avg_right_lane);

  add(left_line, right_line, left_line);
  return right_line;
}

void laneDetection::LaneDetector::calcGradientIntercept(
    Vec4i &line, std::vector<Vec2d> &left_lanes,
    std::vector<Vec2d> &right_lanes) {
  double gradient = (line[3] - line[1]) * 1.0 / (line[2] - line[0]);
  double intercept = line[1] - gradient * line[0];
  Vec2d output = {gradient, intercept};
  if (gradient > 0) {
    if (withinRange(gradient, 0.8, 1.4) && withinRange(intercept, -700, -100)) {
      right_lanes.push_back(output);
    }

  } else {
    if (withinRange(gradient, -1.2, -0.7) &&
        withinRange(intercept, 1300, 1800)) {
      left_lanes.push_back(output);
    }
  }
}

Vec2d LaneDetector::calcVec2dAverage(vector<Vec2d> &vec) {
  Vec2d output;
  if (!vec.empty()) {
    double first_average = 0;
    double second_average = 0;
    for (int i = 0; i < vec.size(); ++i) {
      first_average += vec[i][0];
      second_average += vec[i][1];
    }
    first_average /= vec.size();
    second_average /= vec.size();
    output = Vec2d(first_average, second_average);
  }
  return output;
}

Mat LaneDetector::getVisualisedLines(Mat &input, Vec2d &lane_info) {
  double x1, x2, y1, y2;
  Mat visualised_line;
  y1 = input.rows;
  y2 = input.rows * 0.7;
  x1 = (y1 - lane_info[1]) / lane_info[0];
  x2 = (y2 - lane_info[1]) / lane_info[0];
  line(input, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 0), 5);
  return visualised_line;
}

bool LaneDetector::withinRange(double input, double lower_bound,
                               double upper_bound) {
  return (input > lower_bound) && (input < upper_bound);
}

void LaneDetector::getRosParam() {
  ROS_ASSERT(private_nh.getParam("input_img_topic", _input_img_topic));
  ROS_ASSERT(private_nh.getParam("output_img_topic", _output_img_topic));
  ROS_ASSERT(private_nh.getParam("lane_result_topic", _lane_result_topic));

  private_nh.param("gauss_blur_size", _gauss_blur_size, {5, 5});
  private_nh.param("gauss_blur_sigmaX", _gauss_blur_sigmaX, 0.0);
  private_nh.param("canny_threshold1", _canny_threshold1, 0.0);
  private_nh.param("canny_threshold2", _canny_threshold2, 0.0);
  private_nh.param("roi_polygon", _roi_polygon, {0.0, 0.0, 0.0});
  private_nh.param("hough_rho", _hough_rho, 0.0);
  private_nh.param("hough_theta", _hough_theta, 0.0);
  private_nh.param("hough_thershold", _hough_thershold, 0.0);
  private_nh.param("hough_min_line_length", _hough_min_line_length, 0.0);
  private_nh.param("hough_max_line_gap", _hough_max_line_gap, 0.0);
}

void LaneDetector::getVidParam() {
  _frame_height = _original_img.rows;
  _frame_width = _original_img.cols;
  _frame_type = _original_img.type();
}

void LaneDetector::setPubSub() {
  _input_img_sub = nh.subscribe(_input_img_topic, 1, &LaneDetector::detectLane, this);
  _output_img_pub = nh.advertise<sensor_msgs::Image>(_output_img_topic, 1);
  _lane_result_pub

}

int main() { return 0; }