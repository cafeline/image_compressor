cmake_minimum_required(VERSION 3.8)
project(binary_image_compressor)

# C++17を使用
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake_autoを使用して依存関係の検出を自動化
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# メッセージの生成
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedBinaryImage.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# ライブラリのソースファイル
set(LIB_SOURCES
  src/core/BinaryImageCompressor.cpp
  src/core/BlockProcessor.cpp
  src/core/DictionaryBuilder.cpp
  src/core/PatternEncoder.cpp
  src/io/ImageIO.cpp
  src/io/FileSystem.cpp
  src/io/ImageReader.cpp
  src/io/PgmImageReader.cpp
  src/io/YamlImageReader.cpp
  src/io/ImageProcessor.cpp
  src/model/BinaryPattern.cpp
  src/model/BlockSizeResult.cpp
)

# ライブラリを作成
ament_auto_add_library(${PROJECT_NAME}_lib ${LIB_SOURCES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib
  yaml-cpp
  ${OpenCV_LIBRARIES}
)

# 実行ファイルを作成
ament_auto_add_executable(compressed_image_publisher
  src/ros/compressed_image_publisher.cpp
)
target_link_libraries(compressed_image_publisher
  ${PROJECT_NAME}_lib
  yaml-cpp
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
  ament_index_cpp::ament_index_cpp
)

ament_auto_add_executable(compressed_image_viewer
  src/ros/compressed_image_viewer.cpp
)
target_link_libraries(compressed_image_viewer
  ${PROJECT_NAME}_lib
  yaml-cpp
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

# compressed_map_benchmarkノードを追加
ament_auto_add_executable(compressed_map_benchmark
  src/ros/compressed_map_benchmark.cpp
)
target_link_libraries(compressed_map_benchmark
  "${cpp_typesupport_target}"
)
ament_target_dependencies(compressed_map_benchmark
  rclcpp
  nav_msgs
  std_msgs
)

# ordinary_map_benchmarkノードを追加
ament_auto_add_executable(ordinary_map_benchmark
  src/ros/ordinary_map_benchmark.cpp
)
target_link_libraries(ordinary_map_benchmark
  "${cpp_typesupport_target}"
)
ament_target_dependencies(ordinary_map_benchmark
  rclcpp
  nav_msgs
  std_msgs
)

# 設定ファイルとLaunchファイルのインストール
install(DIRECTORY
  config
  launch
  map
  temp
  DESTINATION share/${PROJECT_NAME}
)

# 自動インストール設定
ament_auto_package()