#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
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
};

struct BallColor {
  std::string name;
  cv::Scalar hsv_lower;
  cv::Scalar hsv_upper;
};

class DetectorNode : public rclcpp::Node {
public:
  DetectorNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // --- Subscribers / Publishers ---
  image_transport::Subscriber sub_;
  rclcpp::Publisher<image_detector::msg::LineSegment>::SharedPtr line_pub_;
  rclcpp::Publisher<image_detector::msg::BallPosition>::SharedPtr ball_pub_;

  // --- パラメータ ---
  std::string input_topic_;
  std::string line_topic_;
  std::string ball_topic_;
  int queue_size_;
  LineParams line_params_;
  std::vector<BallColor> ball_colors_;
};
