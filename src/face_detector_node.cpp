#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class RealtimeFaceDetectorNode : public rclcpp::Node {
public:
  RealtimeFaceDetectorNode() : Node("realtime_face_detector_node") {
    // 1. 声明可配置 ROS 2 参数
    this->declare_parameter<int>("camera_id", 0);
    this->declare_parameter<int>("frame_width", 640);
    this->declare_parameter<int>("frame_height", 480);
    this->declare_parameter<double>("fps", 30.0);
    this->declare_parameter<bool>("show_window", true);
    this->declare_parameter<std::string>("cascade_path", "");

    camera_id_ = this->get_parameter("camera_id").as_int();
    frame_width_ = this->get_parameter("frame_width").as_int();
    frame_height_ = this->get_parameter("frame_height").as_int();
    double fps = this->get_parameter("fps").as_double();
    show_window_ = this->get_parameter("show_window").as_bool();
    std::string cascade_path = this->get_parameter("cascade_path").as_string();

    // 2. 加载 OpenCV 人脸检测模型（支持自定义路径或自动搜索系统默认路径）
    if (!loadFaceCascade(cascade_path)) {
      RCLCPP_ERROR(this->get_logger(), "无法加载人脸分类器 XML 文件，请检查 OpenCV 安装或传入 cascade_path 参数！");
      throw std::runtime_error("Failed to load Haar Cascade model.");
    }

    // 3. 初始化摄像头采集设备
    cap_.open(camera_id_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      // 回退到默认后端（兼容 macOS / Windows WSL / USB 摄像头）
      cap_.open(camera_id_);
    }
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开摄像头设备 (camera_id: %d)", camera_id_);
      throw std::runtime_error("Failed to open camera device.");
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    cap_.set(cv::CAP_PROP_FPS, fps);

    // 4. 创建 ROS 2 发布者（使用 SensorDataQoS 保证低延迟图像传输）
    auto sensor_qos = rclcpp::SensorDataQoS();
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/face_detection/image_annotated", sensor_qos);
    face_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/face_detection/face_centers", 10);

    // 5. 创建高频定时器驱动采集与推理流水线
    auto period = std::chrono::duration<double>(1.0 / fps);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&RealtimeFaceDetectorNode::processFrameCallback, this));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "实时人脸识别节点已启动 [Camera ID: %d, 分辨率: %dx%d, 目标帧率: %.1f Hz]",
                camera_id_, frame_width_, frame_height_, fps);
  }

  ~RealtimeFaceDetectorNode() override {
    if (cap_.isOpened()) {
      cap_.release();
    }
    if (show_window_) {
      cv::destroyAllWindows();
    }
  }

private:
  bool loadFaceCascade(const std::string &user_path) {
    std::vector<std::string> candidate_paths;
    if (!user_path.empty()) {
      candidate_paths.push_back(user_path);
    }
    // 常见 Ubuntu / ROS 2 环境下的 OpenCV 自带模型路径
    candidate_paths.emplace_back("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");
    candidate_paths.emplace_back("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");
    candidate_paths.emplace_back("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml");

    for (const auto &path : candidate_paths) {
      if (std::filesystem::exists(path) && face_cascade_.load(path)) {
        RCLCPP_INFO(this->get_logger(), "成功加载人脸检测模型: %s", path.c_str());
        return true;
      }
    }
    return false;
  }

  void processFrameCallback() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "读取摄像头画面为空，等待下一帧...");
      return;
    }

    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    double instant_fps = (dt > 0.0) ? (1.0 / dt) : 0.0;
    smoothed_fps_ = 0.9 * smoothed_fps_ + 0.1 * instant_fps;

    // 图像预处理：灰度化 + 直方图均衡化（提升大棚/室内复杂光照下的鲁棒性）
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray_frame, gray_frame);

    // 多尺度人脸检测
    std::vector<cv::Rect> faces;
    face_cascade_.detectMultiScale(
        gray_frame,
        faces,
        1.1,                              // 图像金字塔缩放比例
        5,                                // 最小邻近矩形数（抑制误检）
        0 | cv::CASCADE_SCALE_IMAGE,
        cv::Size(60, 60)                  // 最小检测人脸尺寸
    );

    // 构造下游控制可用的目标中心点数组消息 (x, y 像素坐标, z 为检测框宽度估计尺度)
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = current_time;
    pose_array_msg.header.frame_id = "camera_optical_frame";

    for (size_t i = 0; i < faces.size(); ++i) {
      const auto &face = faces[i];

      // 绘制人脸边界框与中心准星
      cv::rectangle(frame, face, cv::Scalar(0, 255, 0), 2);
      cv::Point center(face.x + face.width / 2, face.y + face.height / 2);
      cv::circle(frame, center, 4, cv::Scalar(0, 0, 255), -1);

      std::string label = "Face #" + std::to_string(i + 1) +
                          " (" + std::to_string(center.x) + "," + std::to_string(center.y) + ")";
      cv::putText(frame, label, cv::Point(face.x, std::max(face.y - 8, 15)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);

      // 填充 ROS 2 位姿消息
      geometry_msgs::msg::Pose pose;
      pose.position.x = static_cast<double>(center.x);
      pose.position.y = static_cast<double>(center.y);
      pose.position.z = static_cast<double>(face.width);
      pose.orientation.w = 1.0;
      pose_array_msg.poses.push_back(pose);
    }

    // 左上角绘制状态仪表盘（FPS 与检测人数）
    std::string status_text = "FPS: " + std::to_string(static_cast<int>(smoothed_fps_)) +
                              " | Detected Faces: " + std::to_string(faces.size());
    cv::putText(frame, status_text, cv::Point(15, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 200, 0), 2);

    // 发布检测结果话题与标注图像话题
    face_poses_pub_->publish(pose_array_msg);

    std_msgs::msg::Header img_header;
    img_header.stamp = current_time;
    img_header.frame_id = "camera_optical_frame";
    auto img_msg = cv_bridge::CvImage(img_header, "bgr8", frame).toImageMsg();
    image_pub_->publish(*img_msg);

    // 本地可视化预览
    if (show_window_) {
      cv::imshow("ROS2 Real-time Face Detector", frame);
      int key = cv::waitKey(1);
      if (key == 27 || key == 'q') {  // 按 ESC 或 q 退出
        RCLCPP_INFO(this->get_logger(), "用户触发退出按键，正在关闭节点...");
        rclcpp::shutdown();
      }
    }
  }

  int camera_id_;
  int frame_width_;
  int frame_height_;
  bool show_window_;
  double smoothed_fps_{30.0};

  cv::VideoCapture cap_;
  cv::CascadeClassifier face_cascade_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr face_poses_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RealtimeFaceDetectorNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "节点启动异常: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}