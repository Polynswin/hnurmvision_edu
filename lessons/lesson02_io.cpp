#include "io_context.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace training::io {

namespace {
constexpr double rad2deg(double r) { return r * 180.0 / kPi; }
constexpr double deg2rad(double d) { return d * kPi / 180.0; }

double wrap180(double deg) {
  deg = std::fmod(deg, 360.0);
  if (deg > 180.0) deg -= 360.0;
  if (deg < -180.0) deg += 360.0;
  return deg;
}
}  // namespace

void IoContext::open() {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  node = std::make_shared<rclcpp::Node>("training_io");
  const auto qos = rclcpp::SensorDataQoS();

  sub_img = node->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", qos, [this](sensor_msgs::msg::Image::SharedPtr msg) {
        //这一段是人类吗，文档里没教这个，新手能不用ai写出来我吃（

        cv::Mat rgb(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
                    const_cast<uint8_t*>(msg->data.data()), static_cast<size_t>(msg->step));
        cv::cvtColor(rgb, latest_img.image, cv::COLOR_RGB2BGR);

        const auto& st = msg->header.stamp;
        latest_img.timestamp = TimePoint(std::chrono::nanoseconds(
            static_cast<int64_t>(st.sec) * 1000000000LL + static_cast<int64_t>(st.nanosec)));

        has_new_image = true;
      });

  sub_data = node->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
      "/vision_receive_data", qos, [this](hnurm_interfaces::msg::VisionRecvData::SharedPtr msg) {
        static TimePoint last_print{};
        const auto now = Clock::now();
        if (now - last_print >= std::chrono::seconds(1)) {
          last_print = now;
          printf("yaw: %.2lf deg | pitch: %.2lf deg | bullet_speed : %d m/s \n",
             msg->yaw, msg->pitch, msg->bullet_speed.data);
        }
      });

  pub_cmd = node->create_publisher<hnurm_interfaces::msg::VisionSendData>(
      "/vision_send_data", qos);

  executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
}

std::optional<SensorData> IoContext::read() {
  executor->spin_some(std::chrono::milliseconds(10));
  if (!has_new_image) return std::nullopt;

  has_new_image = false;
  return latest_img;
}

void IoContext::write(const Command& command) {
  hnurm_interfaces::msg::VisionSendData cmd;

  cmd.header.stamp = node->now();
  cmd.header.frame_id = "gimbal_link";

  cmd.pitch = rad2deg(command.pitch);
  cmd.yaw = rad2deg(wrap180(command.yaw));

  if (!command.control) cmd.target_state.data = 0;
  else if (!command.shoot || !config.allow_fire) cmd.target_state.data = 1;
  else cmd.target_state.data = 2;

  cmd.target_distance = -1;
  pub_cmd->publish(cmd);
}

void IoContext::close() {
  auto stop = hnurm_interfaces::msg::VisionSendData{};
  stop.header.stamp = node->now();
  stop.header.frame_id = "gimbal_link";
  stop.target_state.data = 0;
  stop.target_distance = 0;
  pub_cmd->publish(stop);

  executor->spin_some(std::chrono::milliseconds(50));

  executor->cancel();

  sub_img.reset();
  sub_data.reset();
  pub_cmd.reset();
  executor.reset();
  node.reset();

  if (rclcpp::ok()) rclcpp::shutdown();
}
}  // namespace training::io
