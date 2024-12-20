/**
 * @file pointfoot_highlevel_node.cpp
 *
 * @brief Provides a ROS interface for controlling the robot.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * Build:
 * $ mkdir build
 * $ cd build
 * $ cmake ..
 * $ make
 * 
 * Run:
 * $ cd build
 * $ ./pointfoot_highlevel_example
 */

#include <iostream>
#include <string>
#include <limxsdk/pointfoot_highlevel.h>

int main(int argc, char *argv[]) {
  // Get an instance of limxsdk::PointFootHighLevel and initialize it with the provided robot IP address.
  limxsdk::PointFootHighLevel* robot = limxsdk::PointFootHighLevel::getInstance();

  std::string robot_ip = "10.192.1.2"; // Default robot IP address.
  if (argc > 1) {
    robot_ip = argv[1]; // Use the provided command-line argument as the robot IP address, if any.
  }

  // Initialize the robot.
  if (!robot->init(robot_ip)) {
    exit(1); // Exit if robot initialization fails.
  }

  // Subscribe to IMU data and update the IMU message fields when new data is received.
  robot->subscribeImuData([&](const limxsdk::ImuDataConstPtr& imu_data) {
    // TODO: Process IMU data here.
  });
    
  // Subscribe to diagnostic values and update the diagnostic message when new data is received.
  robot->subscribeDiagnosticValue([&](const std::string& name, int level, int code, const std::string& message) {
    // TODO: Process diagnostic data here.
  });

  bool should_exit = false;

  while (!should_exit) {
    std::string command;
    std::cout << "Enter a command ('stand', 'walk', 'twist', 'sit', 'stair', 'stop') or 'exit' to quit:" << std::endl;
    std::getline(std::cin, command);  // Read user input.

    if (command == "exit") {
      should_exit = true;  // Exit the loop if 'exit' is entered.
      break;
    } else if (command == "stand") {
      robot->setRobotMode(limxsdk::RobotMode::STAND);  // Set robot to stand mode.
    } else if (command == "walk") {
      robot->setRobotMode(limxsdk::RobotMode::WALK);  // Set robot to walk mode.
    } else if (command == "twist") {
      float x, y, z;
      std::cout << "Enter x, y, z values:" << std::endl;
      std::cin >> x >> y >> z;  // Get twist values from the user.
      robot->publishTwist(x, y, z);  // Publish the twist message.
    } else if (command == "sit") {
      robot->setRobotMode(limxsdk::RobotMode::SITDOWN);  // Set robot to sit down mode.
    } else if (command == "stair") {
      bool enable;
      std::cout << "Enable stair mode (true/false):" << std::endl;
      std::cin >> enable;  // Get the stair mode enable flag from the user.
      if (enable) {
        robot->setRobotMode(limxsdk::RobotMode::STAIR_MODE_ENTER);  // Enter stair mode.
      } else {
        robot->setRobotMode(limxsdk::RobotMode::STAIR_MODE_EXIT);  // Exit stair mode.
      }
    } else if (command == "stop") {
      robot->setRobotMode(limxsdk::RobotMode::EMERGENCY);  // Send emergency stop request.
    }
  }

  return 0;
}
