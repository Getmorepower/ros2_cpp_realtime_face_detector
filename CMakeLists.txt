cmake_minimum_required(VERSION 3.8)
project(ros2_face_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找 ROS 2 与 OpenCV 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# 编译可执行节点
add_executable(face_detector_node src/face_detector_node.cpp)

target_include_directories(face_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(face_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(face_detector_node ${OpenCV_LIBS})

# 安装可执行文件、Launch 与模型文件
install(TARGETS
  face_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
