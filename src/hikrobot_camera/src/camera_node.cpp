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
  double exposure = this->get_parameter("exposure_time").as_double();
  double gain = this->get_parameter("gain").as_double();
  RCLCPP_INFO(this->get_logger(), "参数设置: 曝光=%f, 增益=%f", exposure, gain);
}

void CameraNode::grab_image()
{
  if (camera_handle_ == nullptr) return;

  // 1. 定义海康的图像信息结构体
  MV_FRAME_OUT stImageInfo = {0};
  
  // 2. 获取一帧图像数据（超时时间设为1000ms）
  int nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 1000);
  
  if (nRet != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "抓图失败，错误码: 0x%x。尝试断线重连...", nRet);
    // 作业要求：断线重连机制
    disconnect_camera();
    connect_camera();
    return;
  }

  // 3. 转换为 ROS 2 的 Image 消息
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = this->now();
  msg->header.frame_id = "camera_link";
  
  // 🌟 注意：这里需要根据您的实际像素格式（YAML里的pixel_format）来设置 encoding
  // 如果格式不匹配，RViz2 里会显示花屏或无法显示
  msg->height = stImageInfo.stFrameInfo.nExtendHeight;
  msg->width = stImageInfo.stFrameInfo.nExtendWidth;
  msg->encoding = "bayer_rgg8"; // 假设您的YAML里配置的是 BayerRG8，如果是 Mono8 则改为 "mono8"
  msg->step = stImageInfo.stFrameInfo.nExtendWidth; // 通常等于宽度（单通道8位时）
  
  // 拷贝海康图像数据到 ROS 2 消息中
  msg->data.assign(stImageInfo.pBufAddr, stImageInfo.pBufAddr + stImageInfo.stFrameInfo.nFrameLen);

  // 4. 发布图像话题
  image_pub_->publish(std::move(msg));

  // 5. 🚨 必须释放 SDK 的图像缓存，否则相机会卡死！
  MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
}}

