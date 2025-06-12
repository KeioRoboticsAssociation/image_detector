#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "image_detector/msg/line_segment.hpp"
#include "image_detector/msg/ball_position.hpp"
#include "image_detector/msg/ball_position_array.hpp"
#include "image_detector/msg/line_segment_array.hpp"

struct LineParams {
  double rho;
  double theta;
  int threshold;
  double min_line_length;
  double max_line_gap;
  // parameters for adaptive thresholding when detecting black lines
  int adaptive_block_size;
  double adaptive_C;
};

struct BallColor {
  std::string name;
  cv::Scalar hsv_lower;
  cv::Scalar hsv_upper;
};

class DetectorNode : public rclcpp::Node {
public:
  DetectorNode();
  void setupSubscriber();  // publicに移動

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // --- Subscribers / Publishers ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;  // image_transportを使わない
  // Array型に修正
  rclcpp::Publisher<image_detector::msg::LineSegmentArray>::SharedPtr line_pub_;
  rclcpp::Publisher<image_detector::msg::BallPositionArray>::SharedPtr ball_pub_;

  // --- パラメータ ---
  std::string input_topic_;
  std::string line_topic_;
  std::string ball_topic_;
  int queue_size_;
  LineParams line_params_;
  std::vector<BallColor> ball_colors_;
};