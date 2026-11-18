# ROS 2 Real-Time Face Detector (C++ & OpenCV)



[![ROS 2 Humble CI Build](https://github.com/Getmorepower/ros2_cpp_realtime_face_detector/actions/workflows/ros2_build.yml/badge.svg)](https://github.com/Getmorepower/ros2_cpp_realtime_face_detector/actions/workflows/ros2_build.yml)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Iron-blue)

![C++](https://img.shields.io/badge/C%2B%2B-17-orange)

![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green)



基于 **ROS 2 (Humble / Iron)** 与 **C++ OpenCV** 构建的实时人脸检测功能包。节点通过笔记本内置摄像头或外接 USB 摄像头实时采集视频流，结合灰度直方图均衡化增强与多尺度级联分类器实现稳定人脸检测，并将可视化标注图像与结构化目标坐标实时发布至 ROS 2 话题网络，可直接对接下游云台追踪或机械臂视觉伺服控制节点。



---



## 1. 核心特性与工程设计



* **标准分布式 ROS 2 架构 (`rclcpp`)**：采用定时器驱动非阻塞采集与推理流水线，完全兼容 ROS 2 Launch 参数动态配置与无桌面端（Headless）边缘设备部署。

* **低延迟双路话题通信**：

  * 图像流采用 `rclcpp::SensorDataQoS()`（Best-Effort 传输策略），有效避免网络拥塞导致的画面延迟积压。

  * 同步输出目标中心点坐标消息，实现感知模块与运动控制模块的彻底解耦。

* **复杂光照鲁棒性增强**：内置灰度空间转换与直方图均衡化（`cv::equalizeHist`）预处理，提升室内外逆光、暗光环境下的特征提取稳定性。

* **自适应模型检索**：支持自定义传入分类器权重路径，同时内置多路径自动检索机制，开箱即用。



---



## 2. 节点接口说明 (ROS 2 API)



### 2.1 发布话题 (Published Topics)



| 话题名称 | 消息类型 | QoS 策略 | 说明 |

| :--- | :--- | :--- | :--- |

| `/face_detection/image_annotated` | `sensor_msgs/msg/Image` | `SensorDataQoS` | 叠加绿色人脸边界框、中心准星、目标编号及实时 FPS 的可视化图像流 (`bgr8`) |

| `/face_detection/face_centers` | `geometry_msgs/msg/PoseArray` | `Reliable (depth=10)` | 检测到的人脸目标中心像素坐标 `(x, y)` 及检测框宽度尺度 `(z)` |



### 2.2 可配置参数 (Parameters)



| 参数名 | 类型 | 默认值 | 说明 |

| :--- | :--- | :--- | :--- |

| `camera_id` | `int` | `0` | 摄像头设备索引号（如 `/dev/video0` 对应 `0`） |

| `frame_width` | `int` | `640` | 摄像头采集图像宽度（像素） |

| `frame_height` | `int` | `480` | 摄像头采集图像高度（像素） |

| `fps` | `double` | `30.0` | 目标采集与推理频率（Hz） |

| `show_window` | `bool` | `true` | 是否开启 OpenCV HighGUI 本地可视化窗口（按 `ESC` 或 `q` 退出） |

| `cascade_path` | `string` | `""` | 自定义 Haar 级联模型 XML 路径（留空则自动加载系统 OpenCV 默认模型） |



---



## 3. 目录结构



```text

ros2_cpp_realtime_face_detector/

├── .github/

│   └── workflows/

│       └── ros2_build.yml            # GitHub Actions CI 自动云端编译验证脚本

├── CMakeLists.txt                    # ament_cmake 编译与安装配置

├── package.xml                       # ROS 2 功能包清单与依赖声明

├── README.md                         # 项目说明文档

├── launch/

│   └── face_detector.launch.py       # 节点一键启动与参数配置 Launch 文件

└── src/

    └── face_detector_node.cpp        # 实时人脸识别核心 C++ 节点源码

```



---



## 4. 环境依赖与编译运行



### 4.1 系统与依赖要求

* **操作系统**：Ubuntu 22.04 LTS

* **ROS 2 版本**：ROS 2 Humble Hawksbill（兼容 Foxy / Iron / Jazzy）

* **依赖库**：`rclcpp`、`sensor_msgs`、`geometry_msgs`、`cv_bridge`、`image_transport`、`OpenCV 4.x`



安装依赖命令：

```bash

sudo apt update

sudo apt install -y ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport libopencv-dev

```



### 4.2 编译功能包

```bash

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

git clone https://github.com/<你的GitHub用户名>/ros2_cpp_realtime_face_detector.git



cd ~/ros2_ws

colcon build --packages-select ros2_face_detector --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

```



### 4.3 启动运行

* **默认模式（笔记本内置摄像头 + 开启预览窗口）：**

  ```bash

  ros2 launch ros2_face_detector face_detector.launch.py

  ```

* **指定外接 USB 摄像头或无桌面端后台模式：**

  ```bash

  ros2 launch ros2_face_detector face_detector.launch.py camera_id:=1 show_window:=false

  ```
