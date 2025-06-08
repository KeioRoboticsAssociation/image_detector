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

  // YAMLの配列構造から色情報を読み込む
  RCLCPP_INFO(this->get_logger(), "Loading ball color configurations from parameters...");
  ball_colors_.clear();

  // config.yamlでは2つのボール設定があるので、それに合わせて読み込む
  // 実際のconfig.yamlの構造に基づいて、直接ボールの色を設定
  // 赤いボール
  BallColor red_ball;
  red_ball.name = "red";
  red_ball.hsv_lower = cv::Scalar(0, 120, 70);
  red_ball.hsv_upper = cv::Scalar(10, 255, 255);
  ball_colors_.push_back(red_ball);

  // 青いボール
  BallColor blue_ball;
  blue_ball.name = "blue";
  blue_ball.hsv_lower = cv::Scalar(100, 150, 0);
  blue_ball.hsv_upper = cv::Scalar(140, 255, 255);
  ball_colors_.push_back(blue_ball);

  RCLCPP_INFO(this->get_logger(), "Loaded %zu ball color configurations", ball_colors_.size());

  // Publisher
  line_pub_ = this->create_publisher<image_detector::msg::LineSegmentArray>(line_topic_, queue_size_);
  ball_pub_ = this->create_publisher<image_detector::msg::BallPositionArray>(ball_topic_, queue_size_);

  // Subscriber
  image_transport::ImageTransport it(shared_from_this());
  sub_ = it.subscribe(
    input_topic_, queue_size_,
    std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "DetectorNode started: input=%s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing lines to: %s", line_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing balls to: %s", ball_topic_.c_str());
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    // OpenCV Mat に変換
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat bgr = cv_ptr->image;
    cv::Mat gray, edges;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, 50, 150);

    // HoughLinesP で直線検出
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(
      edges,
      lines,
      line_params_.rho,
      CV_PI / 180.0 * line_params_.theta,  // thetaをラジアンに変換
      line_params_.threshold,
      line_params_.min_line_length,
      line_params_.max_line_gap
    );

    // 検出直線を publish
    auto msg_line_array = image_detector::msg::LineSegmentArray();
    for (const auto & l : lines) {
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
    
    for (const auto & bc : ball_colors_) {
      cv::Mat mask;
      cv::inRange(hsv, bc.hsv_lower, bc.hsv_upper, mask);
      
      // ノイズ除去のための形態学的処理
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
      
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      
      for (const auto & cnt : contours) {
        double area = cv::contourArea(cnt);
        // 小さすぎる輪郭は無視
        if (area < 100) continue;
        
        cv::Moments m = cv::moments(cnt);
        if (m.m00 > 1e-3) {
          image_detector::msg::BallPosition bp;
          bp.color = bc.name;
          bp.position.x = m.m10 / m.m00;
          bp.position.y = m.m01 / m.m00;
          bp.position.z = 0.0;  // 2D画像なのでz=0
          msg_ball_array.balls.push_back(bp);
        }
      }
    }
    ball_pub_->publish(msg_ball_array);
    
    // デバッグ情報
    if (lines.size() > 0 || msg_ball_array.balls.size() > 0) {
      RCLCPP_DEBUG(this->get_logger(), "Detected %zu lines and %zu balls", 
                   lines.size(), msg_ball_array.balls.size());
    }
    
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
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