cmake_minimum_required(VERSION 3.8)
project(binary_image_compressor)

# C++17を使用
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2の必要パッケージを検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# msgディレクトリ内のメッセージファイルを処理
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedBinaryImage.msg"
  DEPENDENCIES std_msgs
)

# インターフェースのエクスポート
ament_export_dependencies(rosidl_default_runtime)

# メッセージを使用するノードのための依存関係
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# インクルードディレクトリを指定
include_directories(include)

# ライブラリのソースファイル
set(LIB_SOURCES
  src/core/BinaryImageCompressor.cpp
  src/core/BlockProcessor.cpp
  src/core/DictionaryBuilder.cpp
  src/core/PatternEncoder.cpp
  src/io/ImageIO.cpp
  src/model/BinaryPattern.cpp
  src/model/BlockSizeResult.cpp
)

# ライブラリを作成
add_library(${PROJECT_NAME}_lib ${LIB_SOURCES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)


# パブリッシャーノードとビューアーノード - 既存のファイルを使用
add_executable(compressed_image_publisher
  src/ros/compressed_image_publisher.cpp)
target_link_libraries(compressed_image_publisher ${PROJECT_NAME}_lib "${cpp_typesupport_target}")
ament_target_dependencies(compressed_image_publisher
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(compressed_image_viewer
  src/ros/compressed_image_viewer.cpp)
target_link_libraries(compressed_image_viewer ${PROJECT_NAME}_lib "${cpp_typesupport_target}")
ament_target_dependencies(compressed_image_viewer
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
)

# OpenCVライブラリのリンク
target_link_libraries(compressed_image_viewer ${OpenCV_LIBRARIES})

# インストール設定
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS compressed_image_publisher compressed_image_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# 設定ファイルとLaunchファイルのインストール
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# パッケージ化の設定
ament_package()