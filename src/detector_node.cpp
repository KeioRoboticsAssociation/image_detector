#include "image_detector/detector_node.hpp"
#include <map>

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
  this->declare_parameter<int>("detection.line.adaptive_block_size", 15);
  this->declare_parameter<double>("detection.line.adaptive_C", 5.0);
  this->declare_parameter<int>("detection.line.morph_kernel_size", 5);

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
  this->get_parameter("detection.line.adaptive_block_size", line_params_.adaptive_block_size);
  this->get_parameter("detection.line.adaptive_C", line_params_.adaptive_C);
  this->get_parameter("detection.line.morph_kernel_size", line_params_.morph_kernel_size);

  // YAMLの配列構造から色情報を読み込む
  RCLCPP_INFO(this->get_logger(), "Loading ball color configurations from parameters...");
  ball_colors_.clear();

  // まず、ボールのパラメータを明示的に宣言
  for (int i = 0; i < 5; i++) {  // 最大5個のボール設定
    std::string prefix = "detection.balls." + std::to_string(i) + ".";
    
    try {
      this->declare_parameter<std::string>(prefix + "name", "");
      this->declare_parameter<std::vector<int64_t>>(prefix + "hsv_lower", std::vector<int64_t>{});
      this->declare_parameter<std::vector<int64_t>>(prefix + "hsv_upper", std::vector<int64_t>{});
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
      // 既に宣言されている場合は無視
    }
  }

  // デバッグ: 全パラメータを表示
  auto all_params = this->list_parameters({}, 10);
  RCLCPP_INFO(this->get_logger(), "Available parameters after declaration:");
  for (const auto& param_name : all_params.names) {
    if (param_name.find("detection.balls") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "  %s", param_name.c_str());
    }
  }

  // パラメータを読み込む
  for (int i = 0; i < 5; i++) {
    std::string prefix = "detection.balls." + std::to_string(i) + ".";
    
    std::string name;
    std::vector<int64_t> hsv_lower, hsv_upper;
    
    this->get_parameter(prefix + "name", name);
    this->get_parameter(prefix + "hsv_lower", hsv_lower);
    this->get_parameter(prefix + "hsv_upper", hsv_upper);
    
    if (!name.empty() && hsv_lower.size() >= 3 && hsv_upper.size() >= 3) {
      BallColor ball_color;
      ball_color.name = name;
      ball_color.hsv_lower = cv::Scalar(
        static_cast<double>(hsv_lower[0]),
        static_cast<double>(hsv_lower[1]),
        static_cast<double>(hsv_lower[2])
      );
      ball_color.hsv_upper = cv::Scalar(
        static_cast<double>(hsv_upper[0]),
        static_cast<double>(hsv_upper[1]),
        static_cast<double>(hsv_upper[2])
      );
      
      ball_colors_.push_back(ball_color);
      RCLCPP_INFO(this->get_logger(), "Loaded color config for '%s': HSV lower[%.0f,%.0f,%.0f] upper[%.0f,%.0f,%.0f]",
                  ball_color.name.c_str(),
                  ball_color.hsv_lower[0], ball_color.hsv_lower[1], ball_color.hsv_lower[2],
                  ball_color.hsv_upper[0], ball_color.hsv_upper[1], ball_color.hsv_upper[2]);
    }
  }
  
  // ボール設定が一つも読み込めなかった場合はデフォルト値を使用
  if (ball_colors_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No valid ball configurations found in parameters, using defaults");
    
    // デフォルトの赤いボール
    BallColor red_ball;
    red_ball.name = "red";
    red_ball.hsv_lower = cv::Scalar(0, 120, 70);
    red_ball.hsv_upper = cv::Scalar(10, 255, 255);
    ball_colors_.push_back(red_ball);
    
    // デフォルトの青いボール
    BallColor blue_ball;
    blue_ball.name = "blue";
    blue_ball.hsv_lower = cv::Scalar(100, 150, 0);
    blue_ball.hsv_upper = cv::Scalar(140, 255, 255);
    ball_colors_.push_back(blue_ball);
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu ball color configurations", ball_colors_.size());

  // Publisher
  line_pub_ = this->create_publisher<image_detector::msg::LineSegmentArray>(line_topic_, queue_size_);
  ball_pub_ = this->create_publisher<image_detector::msg::BallPositionArray>(ball_topic_, queue_size_);

  RCLCPP_INFO(this->get_logger(), "DetectorNode started: input=%s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing lines to: %s", line_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing balls to: %s", ball_topic_.c_str());
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    // OpenCV Mat に変換
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat hsv;
    std::string determined_encoding;
    
    // 入力がすでにHSVと仮定
    hsv = cv_ptr->image;
    determined_encoding = "HSV";

    RCLCPP_INFO(this->get_logger(), "Received image encoding: %s, interpreted as %s", msg->encoding.c_str(), determined_encoding.c_str());

    // 黒い太いビニールテープ検出のための処理
    
    // HSV色空間で黒色を検出（SaturationとValueが低い領域）
    cv::Mat black_mask;
    cv::Mat s_channel, v_channel;
    cv::extractChannel(hsv, s_channel, 1);  // Saturation
    cv::extractChannel(hsv, v_channel, 2);  // Value
    
    // 黒色の条件: Saturation < 50 かつ Value < 80
    cv::Mat s_mask, v_mask;
    cv::threshold(s_channel, s_mask, 50, 255, cv::THRESH_BINARY_INV);
    cv::threshold(v_channel, v_mask, 80, 255, cv::THRESH_BINARY_INV);
    
    // 両方の条件を満たす領域を黒色マスクとする
    cv::bitwise_and(s_mask, v_mask, black_mask);
    
    // ノイズ除去のための形態学的処理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(black_mask, black_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(black_mask, black_mask, cv::MORPH_CLOSE, kernel);
    
    // 太い線を強調するための膨張処理
    cv::Mat thick_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(black_mask, black_mask, thick_kernel, cv::Point(-1,-1), 2);
    
    // 輪郭検出
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 太いビニールテープの条件を満たす輪郭をフィルタリング
    std::vector<cv::Vec4i> thick_lines;
    for (const auto& contour : contours) {
      // 輪郭の面積をチェック（小さすぎるものは除外）
      double area = cv::contourArea(contour);
      if (area < 1000) continue;  // 最小面積閾値
      
      // 輪郭を囲む最小外接矩形を取得
      cv::RotatedRect rect = cv::minAreaRect(contour);
      cv::Size2f size = rect.size;
      
      // アスペクト比をチェック（線状の形状かどうか）
      double aspect_ratio = std::max(size.width, size.height) / std::min(size.width, size.height);
      if (aspect_ratio < 3.0) continue;  // 線状でない場合は除外
      
      // 太さをチェック（最小幅が一定以上あるか）
      double thickness = std::min(size.width, size.height);
      if (thickness < 8.0) continue;  // 最小太さ閾値
      
      // 矩形の中心点と角度を取得
      cv::Point2f center = rect.center;
      double angle = rect.angle;
      
      // 角度を正規化（-90度から90度の範囲に）
      if (angle < -45) angle += 90;
      
      // 線分の端点を計算
      double length = std::max(size.width, size.height);
      double half_length = length / 2.0;
      
      // 角度から方向ベクトルを計算
      double rad_angle = angle * CV_PI / 180.0;
      double dx = cos(rad_angle);
      double dy = sin(rad_angle);
      
      // 線分の端点を計算
      cv::Point2f start_point(
        center.x - dx * half_length,
        center.y - dy * half_length
      );
      cv::Point2f end_point(
        center.x + dx * half_length,
        center.y + dy * half_length
      );
      
      // 画面上半分の線は検出しない
      if (start_point.y < 257 && end_point.y < 257) {
        continue;
      }
      
      // 線分をVec4i形式で保存
      cv::Vec4i line_segment(
        static_cast<int>(start_point.x),
        static_cast<int>(start_point.y),
        static_cast<int>(end_point.x),
        static_cast<int>(end_point.y)
      );
      
      thick_lines.push_back(line_segment);
    }

    // 検出直線を publish
    auto msg_line_array = image_detector::msg::LineSegmentArray();
    for (const auto & l : thick_lines) {
      image_detector::msg::LineSegment line_segment;
      line_segment.start.x = l[0];
      line_segment.start.y = l[1];
      line_segment.end.x   = l[2];
      line_segment.end.y   = l[3];
      msg_line_array.lines.push_back(line_segment);
    }
    line_pub_->publish(msg_line_array);

    // ボール検出（HSV で inRange → 輪郭抽出 → 重心算出）
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
        if (area < 2000) continue;
        
        // 円形度チェック（ボールは円形に近いはず）
        double perimeter = cv::arcLength(cnt, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        if (circularity < 0.7) continue;  // 円形度が低い場合はスキップ
        
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
    if (thick_lines.size() > 0 || msg_ball_array.balls.size() > 0) {
      RCLCPP_DEBUG(this->get_logger(), "Detected %zu thick black vinyl tape lines and %zu balls", 
                   thick_lines.size(), msg_ball_array.balls.size());
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
  
  // ノードが完全に構築された後にサブスクライバーを設定
  node->setupSubscriber();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void DetectorNode::setupSubscriber()
{
  // 通常のROS2サブスクリプションを使用（image_transportの代わりに）
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic_, queue_size_,
    std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(this->get_logger(), "Subscriber setup complete");
}