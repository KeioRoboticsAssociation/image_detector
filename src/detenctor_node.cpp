#include "image_detector/src/detector_node.hpp"

DetectorNode::DetectorNode()
: Node("image_detector")
{
  // YAML 読み込み
  this->declare_parameter<std::string>("camera.input_topic", "/camera/image_raw");
  this->declare_parameter<std::string>("outputs.line_topic", "/detection/lines");
  this->declare_parameter<std::string>("outputs.ball_topic", "/detection/balls");
  this->declare_parameter<int>("node_parameters.queue_size", 10);
  this->declare_parameter<double>("detection.line.rho", 1.0);
  this->declare_parameter<double>("detection.line.theta", 1.0);
  this->declare_parameter<int>("detection.line.threshold", 50);
  this->declare_parameter<double>("detection.line.min_line_length", 50.0);
  this->declare_parameter<double>("detection.line.max_line_gap", 10.0);
  std::vector<std::string> color_names;
  this->declare_parameter<std::vector<std::string>>("detection.balls", {});

  // パラメータ取得
  this->get_parameter("camera.input_topic", input_topic_);
  this->get_parameter("outputs.line_topic", line_topic_);
  this->get_parameter("outputs.ball_topic", ball_topic_);
  this->get_parameter("node_parameters.queue_size", queue_size_);
  this->get_parameter("detection.line.rho", line_params_.rho);
  this->get_parameter("detection.line.theta", line_params_.theta);
  this->get_parameter("detection.line.threshold", line_params_.threshold);
  this->get_parameter("detection.line.min_line_length", line_params_.min_line_length);
  this->get_parameter("detection.line.max_line_gap", line_params_.max_line_gap);

  // 色指定（YAML 上では配列構造なので、ここは後で適宜拡張してください）
  // 例として赤と青を固定で設定
  ball_colors_.push_back({"red",   cv::Scalar(0,120,70),   cv::Scalar(10,255,255)});
  ball_colors_.push_back({"blue",  cv::Scalar(100,150,0), cv::Scalar(140,255,255)});

  // Publisher
  line_pub_ = this->create_publisher<image_detector::msg::LineSegment>(line_topic_, queue_size_);
  ball_pub_ = this->create_publisher<image_detector::msg::BallPosition>(ball_topic_, queue_size_);

  // Subscriber
  image_transport::ImageTransport it(this->shared_from_this());
  sub_ = it.subscribe(
    input_topic_, queue_size_,
    std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "DetectorNode 起動: input=%s", input_topic_.c_str());
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // OpenCV Mat に変換
  cv::Mat bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::Mat gray, edges;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Canny(gray, edges, 50, 150);

  // HoughLinesP で直線検出
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(
    edges,
    lines,
    line_params_.rho,
    line_params_.theta,
    line_params_.threshold,
    line_params_.min_line_length,
    line_params_.max_line_gap
  );

  // 検出直線を publish
  for (auto & l : lines) {
    auto msg_line = image_detector::msg::LineSegment();
    msg_line.start.x = l[0];
    msg_line.start.y = l[1];
    msg_line.end.x   = l[2];
    msg_line.end.y   = l[3];
    line_pub_->publish(msg_line);
  }

  // ボール検出（HSV で inRange → 輪郭抽出 → 重心算出）
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  for (auto & bc : ball_colors_) {
    cv::Mat mask;
    cv::inRange(hsv, bc.hsv_lower, bc.hsv_upper, mask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto & cnt : contours) {
      auto m = cv::moments(cnt);
      if (m.m00 > 1e-3) {
        image_detector::msg::BallPosition bp;
        bp.color = bc.name;
        bp.position.x = m.m10 / m.m00;
        bp.position.y = m.m01 / m.m00;
        ball_pub_->publish(bp);
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
