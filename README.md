

# LimX SDK 使用说明	

## 1. 创建工作空间

我们假设您的开发电脑中安装ROS（推荐在Ubuntu 20.04操作系统上安装ROS Noetic）。ROS Noetic 安装请参考文档：https://wiki.ros.org/noetic/Installation/Ubuntu ，选择“ros-noetic-desktop-full”进行安装。

可以按照以下步骤，创建一个算法开发工作空间：

- 打开一个Bash终端。
- 创建一个新目录来存放工作空间。例如，可以在用户的主目录下创建一个名为“limx_ws”的目录：
  ```Bash
  mkdir -p ~/limx_ws/src
  ```
- 下载运动控制开发接口：
  ```Bash
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/pointfoot-sdk-highlevel.git
  ```
- 编译工程：
  ```Bash
  cd ~/limx_ws
  catkin_make install
  ```



## 2. ROS 应用接口说明

- ROS的应用接口，通过节点程序pointfoot_highlevel_node提供。pointfoot_highlevel_node基于C++应用接口实现的ROS节点应用程序。它提供下面主题接口：

   | **话题**               | **数据类型**                     | **功能**                                                     |
    | ---------------------- | -------------------------------- | ------------------------------------------------------------ |
    | /topic/limx/twist      | geometry_msgs/Twist              | linear.x：沿 x 轴的线性速度，用于控制机器人的前进后退，单位米每秒。linear.y：沿 y 轴的线性速度，用于控制机器人的横向左右移动，单位米每秒。linear.z：围绕 z 轴的角速度，用于控制机器人的左右转弯，单位弧度每秒。 |
    | /topic/limx/imu        | sensor_msgs/Imu                  | 上报机器人IMU数据                                            |
    | /topic/limx/diagnostic | diagnostic_msgs/DiagnosticStatus | 上报机器人诊断信息                                           |

- 在您的开发工作空间中，`catkin_make install`编译安装成功后，运行下面Shell命令启动pointfoot_highlevel_node节点：

  ```Plaintext
  roslaunch pointfoot_highlevel pointfoot_highlevel.launch
  ```
