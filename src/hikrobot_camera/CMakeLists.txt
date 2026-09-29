cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 1. 查找依赖
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. 包含头文件路径（解决找不到 hikrobot_camera/camera_node.hpp 的问题）
include_directories(
  include
  /opt/MVS/include  # 海康SDK头文件
)

# 3. 生成可执行文件（必须准确写全 main.cpp 和 camera_node.cpp）
add_executable(hikrobot_camera_node src/main.cpp src/camera_node.cpp)

# 4. 链接 ROS 2 依赖（这行必须写在 add_executable 之后！它也会解决找不到 sensor_msgs 的问题）
ament_target_dependencies(hikrobot_camera_node rclcpp sensor_msgs)

# 5. 链接海康 SDK 库
target_link_libraries(hikrobot_camera_node /opt/MVS/lib/64/libMvCameraControl.so)

# 6. 安装规则
install(TARGETS
  hikrobot_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
