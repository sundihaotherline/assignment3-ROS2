#include "hikrobot_camera/camera_node.hpp"

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  // 1. 声明参数（注意：这里不要写 class CameraNode... 直接写 this-> 即可）
  this->declare_parameter("serial_number", "");
  this->declare_parameter("exposure_time", 10000.0);
  this->declare_parameter("gain", 10.0);
  this->declare_parameter("frame_rate", 30.0);
  this->declare_parameter("ip_address", "192.168.1.10");
  this->declare_parameter("pixel_format", "BayerRG8");
  this->declare_parameter("auto_exposure", false);

  // 2. 创建发布者
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

  // 3. 连接相机
  if (connect_camera()) {
    set_camera_parameters();
    
    // 4. 创建定时器
    double fps = this->get_parameter("frame_rate").as_double();
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));
    timer_ = this->create_wall_timer(period, std::bind(&CameraNode::grab_image, this));
    
    RCLCPP_INFO(this->get_logger(), "相机节点启动成功！");
  } else {
    RCLCPP_ERROR(this->get_logger(), "相机连接失败！");
  }
    // 使用 Lambda 表达式绑定参数回调，避免 std::bind 的模板推导问题
  parameters_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->on_parameter_event(parameters);
    });
}

CameraNode::~CameraNode()
{
  disconnect_camera();
}


  bool CameraNode::connect_camera()
{
    int nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 1. 初始化 SDK (参考图1/图2)
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "初始化SDK失败！错误码: 0x%x", nRet);
        return false;
    }

    // 2. 枚举设备 (参考图2中的 MV_CC_EnumDevices)
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "未找到相机设备！");
        return false;
    }

    // 3. 创建句柄并打开设备 (参考图2中的 MV_CC_CreateHandle 和 MV_CC_OpenDevice)
    // 暂时选第0个相机，如果要根据YAML里的序列号/IP选，需要在这里加遍历逻辑
        // 1. 匹配相机
    // 如果 YAML 里配置了 IP，就按 IP 找，否则默认找第 0 个
    std::string target_ip = this->get_parameter("ip_address").as_string();
    MV_CC_DEVICE_INFO* pDeviceInfo = nullptr;

    if (!target_ip.empty()) {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO* pInfo = stDeviceList.pDeviceInfo[i];
            if (pInfo->nTLayerType == MV_GIGE_DEVICE) {
                // 将 SDK 返回的 int IP 转为字符串比较
                unsigned int ip = pInfo->SpecialInfo.stGigEInfo.nCurrentIp;
                char ip_str[16];
                snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF);
                
                if (target_ip == ip_str) {
                    pDeviceInfo = pInfo;
                    RCLCPP_INFO(this->get_logger(), "已匹配到 IP 为 %s 的相机", target_ip.c_str());
                    break;
                }
            }
        }
    }

    if (pDeviceInfo == nullptr) {
        // 没匹配到或者未配置 IP，退而求其次选择第 0 个设备
        pDeviceInfo = stDeviceList.pDeviceInfo[0];
        RCLCPP_WARN(this->get_logger(), "未匹配到指定的 IP，将连接第一个发现的相机");
    }

    // 2. 创建句柄并打开设备（注意：这里用的是上面匹配好的 pDeviceInfo！）
    nRet = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "创建相机句柄失败！");
        return false;
    }

    nRet = MV_CC_OpenDevice(camera_handle_);
    if (nRet != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "打开相机失败！");
        return false;
    }

    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0); 
    MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2); // 2 代表连续采集
      // 3. 开始取流
  nRet = MV_CC_StartGrabbing(camera_handle_);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "开始取流失败！");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "相机连接成功！");
  return true;
} // <--- 注意这里！这个右大括号必须存在，用来闭合 connect_camera 函数！

void CameraNode::disconnect_camera()

{
    if (camera_handle_ != nullptr) {
        MV_CC_StopGrabbing(camera_handle_);   // 停止取流
        MV_CC_CloseDevice(camera_handle_);    // 关闭设备
        MV_CC_DestroyHandle(camera_handle_);  // 销毁句柄 (参考图4)
        MV_CC_Finalize();                     // 反初始化SDK (参考图4)
        camera_handle_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "相机资源已释放");
    }
}


void CameraNode::set_camera_parameters()
{
  if (camera_handle_ == nullptr) return;

  double exposure = this->get_parameter("exposure_time").as_double();
    // ... 在设置增益的那行代码上方加上这行 ...
  MV_CC_SetEnumValue(camera_handle_, "GainAuto", 0); // 0 代表关闭自动增益
  int ret_gain = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
  double gain = this->get_parameter("gain").as_double();
  bool auto_exposure = this->get_parameter("auto_exposure").as_bool();
  double fps = this->get_parameter("frame_rate").as_double();

  // 1. 设置曝光模式（0: 关闭自动，1: 一次，2: 连续）
  if (auto_exposure) {
    MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 2);
  } else {
    MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0);
    // 真正设置曝光时间
    int ret = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置曝光失败，错误码: %x", ret);
    }
  }

  // 2. 设置增益
  int ret_gain = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
  if (ret_gain != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "设置增益失败，错误码: %x", ret_gain);
  }

  // 3. 设置帧率（硬件层面）
  MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
  int ret_fps = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", fps);
  if (ret_fps != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "设置相机硬件帧率失败（可能超出上限）");
  }

  RCLCPP_INFO(this->get_logger(), "参数设置完成: 曝光=%.2f, 增益=%.2f", exposure, gain);
}

void CameraNode::grab_image()
{
  if (camera_handle_ == nullptr) return;

  MV_FRAME_OUT stImageInfo = {0};
  int nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 1000);

  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "抓图失败，尝试重连...");
    disconnect_camera();
    if (connect_camera()) { set_camera_parameters(); }
    return;
  }

  // --- 下面的代码是您之前缺失的最核心部分 ---
  // 1. 转换为 ROS 2 的 Image 消息
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = this->now();
  msg->header.frame_id = "camera_link"; // 和 RViz2 里的 Fixed Frame 保持一致

  // 2. 填写图像参数（这里需要根据你的相机实际输出格式调整！）
  msg->height = stImageInfo.stFrameInfo.nHeight;
  msg->width = stImageInfo.stFrameInfo.nWidth;
  msg->is_bigendian = false;
  
  // ⚠️ 重点：编码格式必须和你的 YAML 设置匹配
  // 如果你 YAML 设了 BayerRG8，这里就写 "bayer_rgg8"（RViz2 可能需要 cv_bridge 转换）
  // 为了测试，先写 "mono8" 试试，或者 "rgb8"
  msg->encoding = "mono8"; 
  msg->step = stImageInfo.stFrameInfo.nWidth; // 单通道 8位：宽度 = 步长。如果是彩色，步长=宽度*3

  // 3. 拷贝图像数据
  size_t data_size = stImageInfo.stFrameInfo.nFrameLen;
  msg->data.resize(data_size);
  memcpy(msg->data.data(), stImageInfo.pBufAddr, data_size);

  // 4. 发布！就这一行，之前没有它所以 RViz2 没画面！
  image_pub_->publish(std::move(msg));

  // 5. 别忘了释放缓存
  MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
}


rcl_interfaces::msg::SetParametersResult CameraNode::on_parameter_event(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    std::string name = param.get_name();

    // 1. 动态调曝光
    if (name == "exposure_time") {
      double val = param.as_double();
      // 校验范围（作业要求3：参数更新应校验范围）
      if (val < 10.0 || val > 1000000.0) {
        result.successful = false;
        result.reason = "曝光时间超出范围 [10, 1000000] 微秒";
        continue;
      }
      int ret = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", val);
      if (ret != MV_OK) {
        result.successful = false;
        result.reason = "SDK设置曝光失败，错误码: " + std::to_string(ret);
      }
    }
    // 2. 动态调增益
    else if (name == "gain") {
      double val = param.as_double();
      int ret = MV_CC_SetFloatValue(camera_handle_, "Gain", val);
      if (ret != MV_OK) { result.successful = false; result.reason = "SDK设置增益失败"; }
    }
    // 3. 动态调帧率
    else if (name == "frame_rate") {
      double val = param.as_double();
      if (val <= 0) { result.successful = false; result.reason = "帧率必须大于0"; continue; }
      
      // 修改 ROS 2 定时器周期
      auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / val));
      timer_->cancel();
      timer_ = this->create_wall_timer(period, std::bind(&CameraNode::grab_image, this));
      
      // 修改硬件帧率
      MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", val);
    }
  }
  return result;
}


}

