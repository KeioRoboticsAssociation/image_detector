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

    // 黒い太い線のみを検出するための処理

    // Vチャンネルを利用して局所的なしきい値処理を行う
    cv::Mat v_channel;
    cv::extractChannel(hsv, v_channel, 2);
    cv::Mat black_mask;
    cv::adaptiveThreshold(
      v_channel,
      black_mask,
      255,
      cv::ADAPTIVE_THRESH_MEAN_C,
      cv::THRESH_BINARY_INV,
      line_params_.adaptive_block_size,
      line_params_.adaptive_C
    );
    
    // 太い線を強調するための形態学的処理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(black_mask, black_mask, cv::MORPH_CLOSE, kernel);
    cv::dilate(black_mask, black_mask, kernel, cv::Point(-1,-1), 1);
    
    // エッジ検出
    cv::Mat edges;
    cv::Canny(black_mask, edges, 50, 150);

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

    // 太い線のみをフィルタリング（近接した平行線をグループ化）
    std::vector<cv::Vec4i> thick_lines;
    for (size_t i = 0; i < lines.size(); i++) {
      cv::Vec4i line1 = lines[i];
      bool is_thick = false;
      
      // 他の線との距離をチェック
      for (size_t j = 0; j < lines.size(); j++) {
        if (i == j) continue;
        cv::Vec4i line2 = lines[j];
        
        // 線分の中点間の距離を計算
        cv::Point2f mid1((line1[0] + line1[2]) / 2.0f, (line1[1] + line1[3]) / 2.0f);
        cv::Point2f mid2((line2[0] + line2[2]) / 2.0f, (line2[1] + line2[3]) / 2.0f);
        float dist = cv::norm(mid1 - mid2);
        
        // 近接した平行線があれば太い線とみなす
        if (dist < 30.0f) {  // 30ピクセル以内
          is_thick = true;
          break;
        }
      }
      
      if (is_thick) {
        thick_lines.push_back(line1);
      }
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
      RCLCPP_DEBUG(this->get_logger(), "Detected %zu thick black lines and %zu balls", 
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