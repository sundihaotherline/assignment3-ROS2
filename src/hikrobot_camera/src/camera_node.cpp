#include "hikrobot_camera/camera_node.hpp"

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  // 1. SDK 初始化（全局只做一次！）
  int nRet = MV_CC_Initialize();
  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "初始化SDK失败！错误码: 0x%x", nRet);
  }

  // 2. 声明参数
  this->declare_parameter("serial_number", "");
  this->declare_parameter("exposure_time", 10000.0);
  this->declare_parameter("gain", 10.0);
  this->declare_parameter("frame_rate", 30.0);
  this->declare_parameter("ip_address", "192.168.1.10");
  this->declare_parameter("pixel_format", "BayerRG8");
  this->declare_parameter("auto_exposure", false);

  // 3. 创建发布者
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

  // 4. 绑定参数回调
  parameters_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->on_parameter_event(parameters);
    });

  // 5. 创建定时器（无论初次是否连上，定时器必须转起来，用于重连）
  double fps = this->get_parameter("frame_rate").as_double();
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));
  timer_ = this->create_wall_timer(period, std::bind(&CameraNode::grab_image, this));

  // 6. 尝试初次连接
  if (connect_camera()) {
    set_camera_parameters();
    is_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "相机节点启动成功！");
  } else {
    RCLCPP_ERROR(this->get_logger(), "相机连接失败，将自动尝试重连...");
  }
}

CameraNode::~CameraNode()
{
  disconnect_camera();
  MV_CC_Finalize(); // 退出时反初始化 SDK（只做一次！）
}

bool CameraNode::connect_camera()
{
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  // 1. 枚举设备
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
    return false; // 找不到设备，直接返回失败，让定时器去重试
  }

  // 2. 匹配 IP
  std::string target_ip = this->get_parameter("ip_address").as_string();
  MV_CC_DEVICE_INFO* pDeviceInfo = nullptr;

  if (!target_ip.empty()) {
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
      MV_CC_DEVICE_INFO* pInfo = stDeviceList.pDeviceInfo[i];
      if (pInfo->nTLayerType == MV_GIGE_DEVICE) {
        unsigned int ip = pInfo->SpecialInfo.stGigEInfo.nCurrentIp;
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF);
        if (target_ip == ip_str) {
          pDeviceInfo = pInfo;
          break;
        }
      }
    }
  }

  if (pDeviceInfo == nullptr) {
    pDeviceInfo = stDeviceList.pDeviceInfo[0]; // 没匹配到就选第 0 个
  }

  // 3. 创建句柄并打开
  nRet = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
  if (nRet != MV_OK) return false;

  nRet = MV_CC_OpenDevice(camera_handle_);
  if (nRet != MV_OK) {
    MV_CC_DestroyHandle(camera_handle_); // 记得销毁句柄，防止泄漏！
    camera_handle_ = nullptr;
    return false;
  }

  // 4. 连续采集模式
  MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0); 
  MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2);

  nRet = MV_CC_StartGrabbing(camera_handle_);
  if (nRet != MV_OK) {
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    return false;
  }

  return true;
}

void CameraNode::disconnect_camera()
{
  if (camera_handle_ != nullptr) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    RCLCPP_INFO(this->get_logger(), "相机资源已释放");
  }
  is_connected_ = false; // 标记为断开
}

void CameraNode::set_camera_parameters()
{
  if (camera_handle_ == nullptr) return;

  double exposure = this->get_parameter("exposure_time").as_double();
  double gain = this->get_parameter("gain").as_double();
  bool auto_exposure = this->get_parameter("auto_exposure").as_bool();
  double fps = this->get_parameter("frame_rate").as_double();

  if (auto_exposure) {
    MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 2);
  } else {
    MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure);
  }

  MV_CC_SetEnumValue(camera_handle_, "GainAuto", 0);
  MV_CC_SetFloatValue(camera_handle_, "Gain", gain);

  MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
  MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", fps);
}

void CameraNode::grab_image()
{
  // 1. 如果没连上，按频率尝试重连（约每秒重试一次）
  if (!is_connected_) {
    retry_count_++;
    if (retry_count_ % 30 == 0) { // 30fps 约每秒重试一次
      RCLCPP_INFO(this->get_logger(), "尝试重新连接相机...");
      if (connect_camera()) {
        set_camera_parameters();
        is_connected_ = true;
        retry_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "相机重连成功！");
      }
    }
    return; // 没连上就不要往下抓图了
  }

  // 2. 正常抓图
  MV_FRAME_OUT stImageInfo = {0};
  int nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 1000);

  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "抓图失败，错误码: 0x%x，准备断线重连...", nRet);
    disconnect_camera(); // 标记为断开，等待定时器去重连
    return;
  }

  // 3. 转换为 ROS 消息
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = this->now();
  msg->header.frame_id = "camera_link";
  msg->height = stImageInfo.stFrameInfo.nHeight;
  msg->width = stImageInfo.stFrameInfo.nWidth;
  msg->is_bigendian = false;
  msg->encoding = "mono8"; 
  msg->step = stImageInfo.stFrameInfo.nWidth;

  // ⚠️ 安全计算内存大小，防止越界崩溃！
  size_t data_size = msg->step * msg->height;
  if (stImageInfo.stFrameInfo.nFrameLen < data_size) {
      data_size = stImageInfo.stFrameInfo.nFrameLen;
  }
  msg->data.resize(data_size);
  memcpy(msg->data.data(), stImageInfo.pBufAddr, data_size);

  image_pub_->publish(std::move(msg));
  MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
}

rcl_interfaces::msg::SetParametersResult CameraNode::on_parameter_event(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    std::string name = param.get_name();
    if (name == "exposure_time") {
      double val = param.as_double();
      if (val < 10.0 || val > 1000000.0) {
        result.successful = false;
        result.reason = "曝光时间超出范围 [10, 1000000] 微秒";
        continue;
      }
      MV_CC_SetFloatValue(camera_handle_, "ExposureTime", val);
    }
    else if (name == "gain") {
      double val = param.as_double();
      MV_CC_SetFloatValue(camera_handle_, "Gain", val);
    }
    else if (name == "frame_rate") {
      double val = param.as_double();
      auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / val));
      timer_->cancel();
      timer_ = this->create_wall_timer(period, std::bind(&CameraNode::grab_image, this));
      MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", val);
    }
  }
  return result;
}

} // namespace hikrobot_camera