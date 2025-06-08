#include "image_detector/detector_node.hpp"

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

  for (int i = 0; i < 5; ++i) { // Assuming max 5 ball configurations for direct declaration
      std::string ball_prefix = "detection.balls." + std::to_string(i) + ".";
      this->declare_parameter<std::string>(ball_prefix + "name", "");
      this->declare_parameter<std::vector<long int>>(ball_prefix + "hsv_lower", std::vector<long int>{0,0,0});
      this->declare_parameter<std::vector<long int>>(ball_prefix + "hsv_upper", std::vector<long int>{0,0,0});
  }

  // 色指定（YAML 上では配列構造なので、ここは後で適宜拡張してください）
  // 例として赤と青を固定で設定
  // ball_colors_.push_back({"red",   cv::Scalar(0,120,70),   cv::Scalar(10,255,255)});
  // ball_colors_.push_back({"blue",  cv::Scalar(100,150,0), cv::Scalar(140,255,255)});

  RCLCPP_INFO(this->get_logger(), "Loading ball color configurations from parameters...");
  ball_colors_.clear(); // Ensure it's empty before loading

  for (int i = 0; i < 5; ++i) { // Corresponds to the 0-4 declarations
      std::string ball_prefix = "detection.balls." + std::to_string(i) + ".";
      std::string name_param_key = ball_prefix + "name";
      std::string hsv_lower_param_key = ball_prefix + "hsv_lower";
      std::string hsv_upper_param_key = ball_prefix + "hsv_upper";

      std::string ball_name;
      this->get_parameter(name_param_key, ball_name);

      if (!ball_name.empty()) {
          std::vector<long int> hsv_lower_values;
          std::vector<long int> hsv_upper_values;
          // It's good practice to check if the parameter exists before getting, though get_parameter can use the default.
          // For our declared params, they will exist.
          this->get_parameter(hsv_lower_param_key, hsv_lower_values);
          this->get_parameter(hsv_upper_param_key, hsv_upper_values);

          if (hsv_lower_values.size() == 3 && hsv_upper_values.size() == 3) {
              BallColor bc;
              bc.name = ball_name;
              bc.hsv_lower = cv::Scalar(static_cast<int>(hsv_lower_values[0]), static_cast<int>(hsv_lower_values[1]), static_cast<int>(hsv_lower_values[2]));
              bc.hsv_upper = cv::Scalar(static_cast<int>(hsv_upper_values[0]), static_cast<int>(hsv_upper_values[1]), static_cast<int>(hsv_upper_values[2]));
              ball_colors_.push_back(bc);
              RCLCPP_INFO(this->get_logger(), "Loaded ball color: %s, HSV Lower: [%ld, %ld, %ld], HSV Upper: [%ld, %ld, %ld]",
                          bc.name.c_str(),
                          hsv_lower_values[0], hsv_lower_values[1], hsv_lower_values[2],
                          hsv_upper_values[0], hsv_upper_values[1], hsv_upper_values[2]);
          } else {
              RCLCPP_WARN(this->get_logger(), "Invalid HSV array sizes for ball '%s' (index %d). Expected 3 values each for lower and upper. Lower size: %zu, Upper size: %zu", ball_name.c_str(), i, hsv_lower_values.size(), hsv_upper_values.size());
          }
      } else {
          // This means no name was provided for detection.balls.i.name, so we assume it's the end of the list in YAML.
          // Or, if we want to be strict about 0 to N-1 being filled, this loop structure is fine.
          // If names are sparsely populated (e.g. 0 and 2 but not 1), this also works.
      }
  }

  if (ball_colors_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No ball colors loaded from parameters. Ball detection will not detect any specific colors based on YAML configuration.");
  }

  // Publisher
  line_pub_ = this->create_publisher<image_detector::msg::LineSegmentArray>(line_topic_, queue_size_);
  ball_pub_ = this->create_publisher<image_detector::msg::BallPositionArray>(ball_topic_, queue_size_);

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
  auto msg_line_array = image_detector::msg::LineSegmentArray();
  for (auto & l : lines) {
    image_detector::msg::LineSegment line_segment;
    line_segment.start.x = l[0];
    line_segment.start.y = l[1];
    line_segment.end.x   = l[2];
    line_segment.end.y   = l[3];
    msg_line_array.lines.push_back(line_segment);
  }
  line_pub_->publish(msg_line_array);

  // ボール検出（HSV で inRange → 輪郭抽出 → 重心算出）
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  auto msg_ball_array = image_detector::msg::BallPositionArray();
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
        msg_ball_array.balls.push_back(bp);
      }
    }
  }
  ball_pub_->publish(msg_ball_array);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
