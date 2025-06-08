cmake_minimum_required(VERSION 3.8)
project(image_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# カスタムメッセージの生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPosition.msg"
  "msg/LineSegment.msg"
  "msg/BallPositionArray.msg"
  "msg/LineSegmentArray.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# 実行ファイルの追加
add_executable(detector_node src/detector_node.cpp)

# インクルードディレクトリの設定
target_include_directories(detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# C++17の使用を指定（より互換性のある方法）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 依存関係のリンク
ament_target_dependencies(detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  OpenCV
)

# カスタムメッセージへの依存関係を追加（新しい方法）
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(detector_node ${cpp_typesupport_target})

# 実行ファイルのインストール
install(TARGETS detector_node
  DESTINATION lib/${PROJECT_NAME})

# インクルードディレクトリのインストール
install(DIRECTORY include/
  DESTINATION include/)

# 設定ファイルのインストール
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/)

# launchファイルのインストール（もしあれば）
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()