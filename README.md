# ROS 2 Real-Time Face Detector (C++ & OpenCV)

[![ROS 2 Humble CI Build](https://github.com/Getmorepower/ros2_cpp_realtime_face_detector/actions/workflows/ros2_build.yml/badge.svg)](https://github.com/Getmorepower/ros2_cpp_realtime_face_detector/actions/workflows/ros2_build.yml)
![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green)

基于 **ROS 2 (Humble / Iron)** 与 **C++ OpenCV** 构建的实时人脸检测功能包。支持通过笔记本内置摄像头或外接 USB 摄像头进行实时图像采集、灰度直方图均衡化增强、多尺度人脸检测，并将标注画面与结构化目标位置发布至 ROS 2 话题网络，可直接与下游云台追踪或机械臂视觉伺服节点对接。

---

## 1. 功能特点

* **标准分布式节点架构 (`rclcpp`)**：采用定时回调机制驱动图像采集与视觉推理流水线，完全解耦传感器采集与下游控制逻辑。
* **低延迟图像传输 (`SensorDataQoS`)**：标注图像流采用 Best-Effort 传输策略，避免无线或高负载网络下的图像队列积压延迟。
* **双路话题输出**：同时提供可视化调试图像与结构化目标中心坐标，兼顾人工监控与机器控制需求。
* **强环境适应性**：推理前引入直方图均衡化（`cv::equalizeHist`），提升复杂光照条件下的检测鲁棒性；支持无桌面端无头模式（`show_window:=false`）部署于嵌入式机器人边缘计算平台。

---

## 2. ROS 2 通信接口

### 发布话题 (Published Topics)

| 话题名称 | 消息类型 | QoS 策略 | 说明 |
| :--- | :--- | :--- | :--- |
| `/face_detection/image_annotated` | `sensor_msgs/msg/Image` | `SensorDataQoS` | 叠加绿色检测框、中心准星及实时 FPS 的可视化图像流 |
| `/face_detection/face_centers` | `geometry_msgs/msg/PoseArray` | `Reliable (Depth=10)` | 检测到的所有人脸中心像素坐标 `(x, y)` 及检测框宽度尺度 `(z)` |

### 可配置参数 (Parameters)

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `camera_id` | `int` | `0` | 摄像头设备索引号（对应 `/dev/video0`） |
| `frame_width` | `int` | `640` | 采集图像宽度（像素） |
| `frame_height` | `int` | `480` | 采集图像高度（像素） |
| `fps` | `double` | `30.0` | 目标采集与推理帧率（Hz） |
| `show_window` | `bool` | `true` | 是否开启 OpenCV HighGUI 本地可视化预览窗口 |
| `cascade_path` | `string` | `""` | 自定义 Haar 级联分类器 XML 路径（留空则自动检索系统默认模型） |

---

## 3. 目录结构

```text
ros2_cpp_realtime_face_detector/
├── .github/
│   └── workflows/
│       └── ros2_build.yml        # GitHub Actions 自动化编译测试工作流
├── CMakeLists.txt                # CMake 构建配置
├── package.xml                   # ROS 2 功能包清单与依赖声明
├── README.md                     # 项目说明文档
├── launch/
│   └── face_detector.launch.py   # ROS 2 启动文件与默认参数配置
└── src/
    └── face_detector_node.cpp    # 核心 C++ 实时采集与人脸识别节点源码
