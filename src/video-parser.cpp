#include "video-parser.h"

using namespace cv;
using namespace laneDetection;
using std::string;
using std::vector;

VideoParser::VideoParser() : private_nh("~"), it(nh) {
  getRosParam();
  _video = std::make_unique<VideoCapture>(_video_filename);
  getVidParam();
  setPubSub();
  _video_timer = nh.createTimer(ros::Duration(1.0 / _video_fps),
                                &VideoParser::pubFrame, this);
}

void VideoParser::pubFrame(const ros::TimerEvent& timer_event) {
  if (_video->read(_frame)) {
    if (_is_stereo_img) {
      _frame = _frame(Range::all(), Range(1, _frame_width / 2));
    }
    // waitKey(_video_delay);
    _output_img =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", _frame).toImageMsg();
    _output_img_pub.publish(_output_img);
  } else {
    ROS_ERROR("[VideoParser] Unable to open video file");
  }
}

void VideoParser::getRosParam() {
  ROS_ASSERT(private_nh.getParam("video_filename", _video_filename));
  ROS_ASSERT(private_nh.getParam("output_img_topic", _output_img_topic));
  ROS_ASSERT(private_nh.getParam("is_stereo_img", _is_stereo_img));
}

void VideoParser::getVidParam() {
  if (_video->read(_frame)) {
    _video_fps = std::min((int)_video->get(cv::CAP_PROP_FPS),100);
    _video_delay = std::max(1000 / _video_fps, 10);
    _frame_height = _frame.rows;
    _frame_width = _frame.cols;
    _frame_type = _frame.type();
  }
}

void VideoParser::setPubSub() {
  _output_img_pub = it.advertise(_output_img_topic, 1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_parser_node");
  laneDetection::VideoParser video_parser_obj;
  while(ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}