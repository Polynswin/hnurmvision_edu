#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <hnurm_interfaces/msg/vision_recv_data.hpp>
#include <hnurm_interfaces/msg/vision_send_data.hpp>
#include "config.hpp"
#include "types.hpp"
#include <optional>

namespace training::io {
  const auto qos = rclcpp::SensorDataQoS().keep_last(5);
struct IoContext {
  Config config;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr sub_data;
  rclcpp::Publisher<hnurm_interfaces::msg::VisionSendData>::SharedPtr pub_cmd;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  
  SensorData latest_img;
  bool has_new_image = false;
  void open();
  std::optional<SensorData> read();
  void write(const Command& command);
  void close();
};
}  // namespace training::io
