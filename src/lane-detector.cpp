#include "lane-detector.h"

using namespace cv;
using namespace laneDetection;
using std::string;
using std::vector;

LaneDetector::LaneDetector() : private_nh("~"), _img_transport_handle(nh) {
  getRosParam();
  getVidParam();
  setPubSub();
}

LaneDetector::~LaneDetector() {
  // set up thread barrier before this object is destroyed
  std::for_each(_threads.begin(), _threads.end(),
                [](std::thread &t) { t.join(); });
}

void LaneDetector::detectLane(const sensor_msgs::Image &msg) {
  _original_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
  _processed_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
  
  cannyDetector(_processed_img, _processed_img);
  segmentRoi(_processed_img, _processed_img);
  calcHoughLines(_processed_img, _left_lane_avg_param, _right_lane_avg_param);
  overlayLanesToImg(_original_img, _left_lane_avg_param, _right_lane_avg_param);
  _output_img = cv_bridge::CvImage(std_msgs::Header(), "mono8", _processed_img)
                    .toImageMsg();
  _output_img_pub.publish(_output_img);
}

void LaneDetector::cannyDetector(Mat &input, Mat &output) {
  cvtColor(input, output, COLOR_RGB2GRAY);  // change to grayscale
  GaussianBlur(input, output, Size(_gauss_blur_size[0], _gauss_blur_size[1]),
               _gauss_blur_sigmaX);                            // apply blur
  Canny(input, output, _canny_threshold1, _canny_threshold2);  // detects edges
}

void LaneDetector::segmentRoi(Mat &input, Mat &output) {
  Mat mask = Mat::zeros(Size(_frame_width, _frame_height), _frame_type);
  Point2i polygon[1][3];
  polygon[0][0] = Point2i(_frame_width * _roi_btm_left_ratio[0],
                          _frame_height * _roi_btm_left_ratio[1]);
  polygon[0][1] = Point2i(_frame_width * _roi_top_ratio[0],
                          _frame_height * _roi_top_ratio[1]);
  polygon[0][2] = Point2i(_frame_width * _roi_btm_right_ratio[0],
                          _frame_height * _roi_btm_right_ratio[1]);
  const Point2i *ppt[1] = {polygon[0]};
  int npt[] = {3};
  fillPoly(mask, ppt, npt, 1, 255);
  bitwise_and(input, mask, output);
}

void LaneDetector::calcHoughLines(Mat &input, Vec2d &left_lane_avg_param,
                                  Vec2d &right_lane_avg_param) {
  std::vector<Vec4i> lines;
  HoughLinesP(input, lines, _hough_rho, _hough_theta, _hough_thershold,
              _hough_min_line_length, _hough_max_line_gap);

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

  left_lane_avg_param = calcVec2dAverage(left_lanes);
  right_lane_avg_param = calcVec2dAverage(right_lanes);
}

void LaneDetector::overlayLanesToImg(Mat &input, Vec2d &left_lane_avg_param,
                                     Vec2d &right_lane_avg_param) {
  Mat left_line = getVisualisedLines(input, left_lane_avg_param);
  Mat right_line = getVisualisedLines(input, right_lane_avg_param);
  add(left_line, right_line, left_line);
  addWeighted(input, 0.9, left_line, 1, 1, input);
}

void LaneDetector::calcGradientIntercept(Vec4i &line,
                                         std::vector<Vec2d> &left_lanes,
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

  private_nh.param("gauss_blur_size", _gauss_blur_size, {5, 5});
  private_nh.param("gauss_blur_sigmaX", _gauss_blur_sigmaX, 0.0);
  private_nh.param("canny_threshold1", _canny_threshold1, 150.0);
  private_nh.param("canny_threshold2", _canny_threshold2, 150.0);
  private_nh.param("roi_btm_left_ratio", _roi_btm_left_ratio, {0.2, 0.9});
  private_nh.param("roi_top_ratio", _roi_top_ratio, {0.45, 0.6});
  private_nh.param("roi_btm_right_ratio", _roi_btm_right_ratio, {0.7, 0.9});
  private_nh.param("hough_rho", _hough_rho, 1.0);
  private_nh.param("hough_theta", _hough_theta, CV_PI / 180);
  private_nh.param("hough_thershold", _hough_thershold, 50.0);
  private_nh.param("hough_min_line_length", _hough_min_line_length, 50.0);
  private_nh.param("hough_max_line_gap", _hough_max_line_gap, 10.0);
}

void LaneDetector::getVidParam() {
  _frame_height = _original_img.rows;
  _frame_width = _original_img.cols;
  _frame_type = _original_img.type();
}

void LaneDetector::setPubSub() {
  _input_img_sub =
      nh.subscribe(_input_img_topic, 1, &LaneDetector::detectLane, this);
  _output_img_pub = _img_transport_handle.advertise(_output_img_topic, 1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_detector_node");
  laneDetection::LaneDetector lane_detector_obj;
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}