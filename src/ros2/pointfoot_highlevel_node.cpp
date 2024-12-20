/**
 * @file pointfoot_highlevel_node.cpp
 *
 * @brief Provides ROS 2 interface for controlling the robot
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <limxsdk/pointfoot_highlevel.h>

using namespace std::placeholders;

// Static pointer to limxsdk::PointFootHighLevel object named robot initialized to nullptr
static limxsdk::PointFootHighLevel* robot = nullptr;

class PointFootHighLevelNode : public rclcpp::Node {
public:
  PointFootHighLevelNode() : Node("pointfoot_highlevel_node") {
    // Declare and retrieve the robot IP parameter
    ip_ = this->declare_parameter<std::string>("robot_ip", "10.192.1.2");

    // Create publishers for IMU data and diagnostic status messages
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/topic/limx/imu", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/topic/limx/diagnostic", 10);

    // Create subscribers for Twist and robot mode messages
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/topic/limx/twist", 1, std::bind(&PointFootHighLevelNode::twistCallback, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/topic/limx/mode", 1, std::bind(&PointFootHighLevelNode::modeCallback, this, std::placeholders::_1));

    // Initialize the robot
    robot = limxsdk::PointFootHighLevel::getInstance();
    if (robot->init(ip_)) {
      RCLCPP_INFO(this->get_logger(), "Robot initialized successfully.");

      // Subscribe to IMU data
      robot->subscribeImuData([this](const limxsdk::ImuDataConstPtr& imu_data) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "limx_imu";
        imu_msg.angular_velocity.x = imu_data->gyro[0];
        imu_msg.angular_velocity.y = imu_data->gyro[1];
        imu_msg.angular_velocity.z = imu_data->gyro[2];
        imu_msg.linear_acceleration.x = imu_data->acc[0];
        imu_msg.linear_acceleration.y = imu_data->acc[1];
        imu_msg.linear_acceleration.z = imu_data->acc[2];
        imu_msg.orientation.w = imu_data->quat[0];
        imu_msg.orientation.x = imu_data->quat[1];
        imu_msg.orientation.y = imu_data->quat[2];
        imu_msg.orientation.z = imu_data->quat[3];

        imu_pub_->publish(imu_msg);
      });

      // Subscribe to diagnostic values
      robot->subscribeDiagnosticValue([this](const std::string& name, int level, int code, const std::string& message) {
        diagnostic_msgs::msg::DiagnosticStatus diag_msg;
        diag_msg.name = name;
        diag_msg.level = level;
        diag_msg.message = message;
        diag_msg.values.resize(1);
        diag_msg.values[0].key = name;
        diag_msg.values[0].value = std::to_string(code);

        diag_pub_->publish(diag_msg);
      });
    } else {
      RCLCPP_ERROR(this->get_logger(), "Robot initialization failed.");
    }
  }

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (robot != nullptr) {
      robot->publishTwist(static_cast<float>(msg->linear.x),
                          static_cast<float>(msg->linear.y),
                          static_cast<float>(msg->angular.z));
    }
  }

  void modeCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (robot != nullptr) {
      switch (msg->data) {
      case 0:
        robot->setRobotMode(limxsdk::RobotMode::STAND);
        break;
      case 1:
        robot->setRobotMode(limxsdk::RobotMode::WALK);
        break;
      case 2:
        robot->setRobotMode(limxsdk::RobotMode::SITDOWN);
        break;
      case 3:
        robot->setRobotMode(limxsdk::RobotMode::EMERGENCY);
        break;
      case 4:
        robot->setRobotMode(limxsdk::RobotMode::STAIR_MODE_ENTER);
        break;
      case 5:
        robot->setRobotMode(limxsdk::RobotMode::STAIR_MODE_EXIT);
        break;
      default:
        break;
      }
    }
  }

  std::string ip_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
};

int main(int argc, char** argv) {
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);

  // Create the node and spin it
  auto node = std::make_shared<PointFootHighLevelNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
