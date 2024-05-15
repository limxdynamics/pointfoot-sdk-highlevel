/**
 * @file pointfoot_highlevel_node.cpp
 *
 * @brief Provides ROS interface for controlling the robot
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <limxsdk/pointfoot_highlevel.h>

// Declaration of a static pointer to limxsdk::PointFootHighLevel object named robot initialized to nullptr
static limxsdk::PointFootHighLevel* robot = nullptr;

// Declaration of sensor_msgs::Imu message object named imu_msg
static sensor_msgs::Imu imu_msg;

// Declaration of diagnostic_msgs::DiagnosticStatus message object named diag_msg
static diagnostic_msgs::DiagnosticStatus diag_msg;

// Callback function that handles geometry_msgs::Twist messages
static void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Check if the robot pointer is not null
  if (robot != nullptr) {
    // Call the publishTwist method of the robot object with linear x, y, and z values extracted from the received message
    robot->publishTwist((float)msg->linear.x, (float)msg->linear.y, (float)msg->linear.z);
  }
}

int main(int argc, char** argv)
{
  // Initialize the ROS node with the name "pointfoot_highlevel_node"
  ros::init(argc, argv, "pointfoot_highlevel_node");
  
  // Create publishers and subscriber for IMU data, joystick data, and diagnostic status messages
  ros::NodeHandle nh;
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/topic/limx/imu", 10);
  ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("/topic/limx/diagnostic", 10);
  ros::Subscriber twist_sub = nh.subscribe("/topic/limx/twist", 1, &twistCallback);
  
  // Retrieve the robot IP parameter from the NodeHandle, defaulting to "10.192.1.2" if not found
  std::string ip = nh.param<std::string>("robot_ip", "10.192.1.2");
  
  // Get an instance of limxsdk::PointFootHighLevel and initialize it with the retrieved IP
  robot = limxsdk::PointFootHighLevel::getInstance();
  if (robot->init(ip)) {
    // Subscribe to IMU data and update the IMU message fields when data is received
    robot->subscribeImuData([&](const limxsdk::ImuDataConstPtr& imu_data) {
      imu_msg.header.stamp = ros::Time::now();
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
      
      // Publish the updated IMU message
      imu_pub.publish(imu_msg);
    });
    
    // Subscribe to diagnostic values and update the diagnostic message when new data is received
    robot->subscribeDiagnosticValue([&](const std::string& name, int level, int code, const std::string& message) {
      diag_msg.name = name;
      diag_msg.level = level;
      diag_msg.message = message;
      diag_msg.values.resize(1);
      diag_msg.values[0].key = name;
      diag_msg.values[0].value = std::to_string(code);
      
      // Publish the updated diagnostic message
      diag_pub.publish(diag_msg);
    });

    // Enter a loop to process ROS callbacks and events, effectively spinning the node
    ros::spin();
  } else {
    // Print an error message if robot initialization fails
    ROS_ERROR("robot init failed!");
  }
  
  // Return 0 to indicate successful program execution
  return 0;
}
