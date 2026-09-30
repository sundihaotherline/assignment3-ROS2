# 项目简介

 本功能包基于 Ubuntu 22.04 / ROS 2 Humble 与海康 MVS SDK 开发，旨在将工业相机接入 ROS 2 生态，实现了设备发现与 IP/序列号匹配、标准 sensor_msgs/msg/Image 图像发布（话题 /image_raw）、曝光/增益/帧率的动态参数配置与范围校验、以及断线重连机制。

## 环境与依赖
 操作系统：Ubuntu 22.04

 ROS 2 版本：Humble

硬件驱动：海康 MVS SDK（需前往官网下载并安装）

ROS 2 依赖：rclcpp、sensor_msgs、cv_bridge、OpenCV（用于彩色图像转换）

## 编译与运行
 1. 清理并编译
rm -rf build install log
colcon build --packages-select hikrobot_camera
source install/setup.zsh

 2. 运行节点（推荐使用 sudo -E 提权以防 Linux USB 权限不足）
sudo -E env "PATH=$PATH" ros2 run hikrobot_camera hikrobot_camera_node

## 图像查看
source /opt/ros/humble/setup.zsh
ros2 run rqt_image_view rqt_image_view

## 功能验证

### 动态参数测试
ros2 param set /hikrobot_camera exposure_time 30000。0
ros2 param set /hikrobot_camera gain 15.0
ros2 param set /hikrobot_camera frame_rate 60.0
### 实际帧率验证
ros2 topic hz /image_raw
### 错误处理与范围校验
ros2 param set /hikrobot_camera exposure_time -100.0
### 断线重连测试
在节点运行过程中，直接拔掉 USB 线，终端将进入重连循环；等待约 3-5 秒重新插回 USB 线，节点应自动打印 相机重连成功！ 并恢复图像发布。