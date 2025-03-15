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

# インクルードディレクトリを指定
include_directories(include)

# ライブラリのソースファイル
set(LIB_SOURCES
  src/BinaryImageCompressor.cpp
  src/BlockProcessor.cpp
  src/DictionaryBuilder.cpp
  src/PatternEncoder.cpp
  src/ImageIO.cpp
  src/BinaryPattern.cpp
  src/BlockSizeResult.cpp
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

# ROS2ノード作成
add_executable(compressor_node src/binary_image_compressor_node.cpp)
target_link_libraries(compressor_node ${PROJECT_NAME}_lib)
ament_target_dependencies(compressor_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  std_srvs
)

target_include_directories(compressor_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ... その他のインクルードディレクトリ
)

target_link_libraries(compressor_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # ... その他のライブラリ
)

# インストール設定
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS compressor_node
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