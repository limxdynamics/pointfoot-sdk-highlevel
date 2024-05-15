/**
 * @file pointfoot_highlevel.h
 *
 * @brief This file contains the declarations of classes related to the high-level control of point-foot robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */
#ifndef _LIMX_SDK_POINTFOOT_HIGH_LEVEL_H_
#define _LIMX_SDK_POINTFOOT_HIGH_LEVEL_H_
#include <stdint.h>
#include <vector>
#include <memory>
#include <string>
#include <functional>

#if defined(_MSC_VER)
  #define LIMX_SDK_API __declspec(dllexport)
#elif __GNUC__ >= 4
  #define LIMX_SDK_API __attribute__ ((visibility("default")))
#else
  #define LIMX_SDK_API
#endif

#ifdef _MSC_VER
  #pragma warning(disable: 4251)
  #pragma warning(disable: 4275)
#endif

namespace limxsdk {

  struct ImuData {
    ImuData() { }
    uint64_t stamp;         // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    float acc[3];           // Array to store IMU (Inertial Measurement Unit) accelerometer data for tracking linear acceleration along three axes
    float gyro[3];          // Array to store IMU gyroscope data for tracking angular velocity or rotational speed
    float quat[4];          // Array to store IMU quaternion data, represents orientation in 3D space
  };
  typedef std::shared_ptr<ImuData> ImuDataPtr;
  typedef std::shared_ptr<ImuData const> ImuDataConstPtr;

  /**
   * @brief Class for controlling a point-foot robot using the LIMX SDK API.
   */
  class LIMX_SDK_API PointFootHighLevel {
    public:
      /**
       * @brief Get an instance of the PointFootHighLevel class.
       * @return A pointer to a PointFootHighLevel instance (Singleton pattern).
       */
      static PointFootHighLevel* getInstance();

       /**
       * @brief This method should specify the operations to be performed before using the object in the main function.
       * @param robot_ip_address The IP address of the robot, default set to "10.192.1.2".
       * @return True if initialized successfully, otherwise false.
       */
      bool init(const std::string& robot_ip_address = "10.192.1.2");

      /**
       * @brief Publishes a twist command to control the robot's actions.
       * 
       * @param x Linear velocity along the x-axis.
       * @param y Linear velocity along the y-axis.
       * @param z Angular velocity around the z-axis.
       * @return True if the twist command was successfully published, otherwise false.
       */
      bool publishTwist(float x, float y, float z);

      /**
       * @brief Method to subscribe to diagnostic values from the robot.
       * 
       * Examples:
       * | name        | level  | code | msg
       * |-------------|--------|------|--------------------
       * | imu         | OK     | 0    | - IMU is functioning properly.
       * | imu         | ERROR  | -1   | - Error in IMU.
       * |-------------|--------|------|--------------------
       * | ethercat    | OK     | 0    | - EtherCAT is working fine.
       * | ethercat    | ERROR  | -1   | - EtherCAT error.
       * |-------------|--------|------|--------------------
       * | calibration | OK     | 0    | - Robot calibration successful.
       * | calibration | WARN   | 1    | - Robot calibration in progress.
       * | calibration | ERROR  | -1   | - Robot calibration failed.
       * |-------------|--------|------|--------------------
       * 
       * @param cb The callback function to be invoked when diagnostic values are received from the robot.
       */
      void subscribeDiagnosticValue(std::function<void(const std::string& name, int level, int code, const std::string& message)> cb);

      /**
       * @brief Subscribe to receive updates about the robot's IMU data.
       * 
       * @param cb The callback function to be invoked when new IMU data is received from the robot.
       */
      void subscribeImuData(std::function<void(const ImuDataConstPtr&)> cb);

      /**
       * @brief Destructor for the PointFootHighLevel class.
       *        Cleans up any resources used by the object.
       */
      virtual ~PointFootHighLevel();

    private:
      /**
       * @brief Private constructor to prevent external instantiation of the PointFootHighLevel class.
       */
      PointFootHighLevel();
  };
};

#endif