#ifndef HIKROBOT_CAMERA_CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA_CAMERA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
// 引入海康 SDK 头文件
#include "MvCameraControl.h"

namespace hikrobot_camera
{
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode(); // 析构函数，释放资源

private:
  // ROS 2 发布者和定时器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  rcl_interfaces::msg::SetParametersResult on_parameter_event(
      const std::vector<rclcpp::Parameter> & parameters);
  // 海康相机句柄
  void* camera_handle_ = nullptr;

  // 内部功能函数
  bool connect_camera();                 // 连接相机
  void disconnect_camera();              // 断开相机并释放资源
  void set_camera_parameters();          // 设置曝光、增益等参数
  void grab_image();                     // 定时器回调，抓图并发布
  void on_parameter_event(...);          // 参数动态修改回调（具体签名参考 ROS2 文档）
}; // 注意这里的结尾分号！
} // namespace hikrobot_camera

#endif // HIKROBOT_CAMERA_CAMERA_NODE_HPP_