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
ros2 run hikrobot_camera hikrobot_camera_node

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
### 可配置参数
参数名	               类型	                 默认值	                    说明
ip_address	          string	            ""	                    相机 IP（GigE 相机填 IP，USB 相机留空）
serial_number	      string	            "00D36741056"	                    相机序列号（用于精准匹配设备）
exposure_time	      double	           10000.0	                曝光时间，单位微秒 (us)
gain	              double	           10.0	                    增益值，单位 dB
frame_rate	          double	           30.0	                    目标采集帧率 (FPS)
auto_exposure	      bool	              false	                    是否开启自动曝光
pixel_format	      string	          Bayer RG8                    像素格式（需与代码中 encoding 对应）
## SDK及依赖的安装方式

### 1. 海康 MVS SDK 安装
- **下载**：前往 [海康机器人官网](https://www.hikrobotics.com/cn/machinevision/service/download) 下载对应平台的 Linux 版本 SDK（例如：`MVS-5.1.0_Linux_x86_64_xxxxxxxx.deb`）。
- **安装**：在终端中进入下载目录，执行以下命令：
  ```bash
  sudo dpkg -i MVS-*.deb
 ### 依赖修复
 sudo apt --fix-broken install
 ### 2.所有依赖
 <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>rcl_interfaces</depend>
  <depend>camera_info_manager</depend>
    <depend>cv_bridge</depend>
  <depend>libopencv-dev</depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>ament_index_python</exec_depend>

# 1. 查找 ROS 2 依赖
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 2. 包含海康 SDK 的头文件路径
include_directories(/opt/MVS/include)

# 3. 生成可执行文件
add_executable(hikrobot_camera_node src/main.cpp src/camera_node.cpp)

# 4. 链接 ROS 2 依赖
ament_target_dependencies(hikrobot_camera_node rclcpp sensor_msgs cv_bridge)

# 5. 链接海康 SDK 动态库（这是最关键的底层依赖！）
target_link_libraries(hikrobot_camera_node /opt/MVS/lib/64/libMvCameraControl.so)