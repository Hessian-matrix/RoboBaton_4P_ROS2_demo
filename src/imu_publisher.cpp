#include "robobaton_4p_ros2_demo/imu_publisher.hpp"

#include <utility>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace robobaton_4p_ros2_demo {

namespace {

constexpr char kImuTopic[] = "/robobaton/imu/data";
constexpr char kTemperatureTopic[] = "/robobaton/imu/temperature";

}  // namespace

class ImuPublisher::Impl {
 public:
  Impl(rclcpp::Node* node, Config config)
      : node_(node), config_(std::move(config)), driver_(config_.driver_config) {
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        kImuTopic, rclcpp::SensorDataQoS().keep_last(100));
    if (config_.publish_temperature) {
      temperature_pub_ = node_->create_publisher<sensor_msgs::msg::Temperature>(
          kTemperatureTopic, rclcpp::SensorDataQoS().keep_last(10));
    }

    // 2026-07-09 修改原因：ICM-42688 驱动回调线程只发布轻量 IMU 消息，不做阻塞运算。
    driver_.RegisterCallback([this](const icm42688_x5::ImuSample& sample) {
      PublishSample(sample);
    });
  }

  ~Impl() { Stop(); }

  void Start() {
    if (running_) {
      return;
    }
    driver_.Start();
    running_ = true;
    RCLCPP_INFO(node_->get_logger(), "Started ICM-42688 IMU publisher on %s", kImuTopic);
  }

  void Stop() {
    if (!running_) {
      return;
    }
    driver_.Stop();
    running_ = false;
    RCLCPP_INFO(node_->get_logger(), "Stopped ICM-42688 IMU publisher");
  }

 private:
  void PublishSample(const icm42688_x5::ImuSample& sample) {
    auto imu_msg = sensor_msgs::msg::Imu();
    // 2026-07-09 修改原因：host_timestamp_ns 时间域未确认，第一版统一使用 ROS 发布时间。
    imu_msg.header.stamp = node_->now();
    imu_msg.header.frame_id = config_.frame_id;

    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity.x = sample.gyro_x_rps;
    imu_msg.angular_velocity.y = sample.gyro_y_rps;
    imu_msg.angular_velocity.z = sample.gyro_z_rps;
    imu_msg.linear_acceleration.x = sample.accel_x_mps2;
    imu_msg.linear_acceleration.y = sample.accel_y_mps2;
    imu_msg.linear_acceleration.z = sample.accel_z_mps2;

    imu_pub_->publish(imu_msg);

    if (temperature_pub_) {
      auto temperature_msg = sensor_msgs::msg::Temperature();
      temperature_msg.header = imu_msg.header;
      temperature_msg.temperature = sample.temperature_c;
      temperature_pub_->publish(temperature_msg);
    }
  }

  rclcpp::Node* node_ = nullptr;
  Config config_;
  icm42688_x5::Driver driver_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  bool running_ = false;
};

ImuPublisher::ImuPublisher(rclcpp::Node* node, Config config)
    : impl_(std::make_unique<Impl>(node, std::move(config))) {}

ImuPublisher::~ImuPublisher() = default;

void ImuPublisher::Start() { impl_->Start(); }

void ImuPublisher::Stop() { impl_->Stop(); }

}  // namespace robobaton_4p_ros2_demo
