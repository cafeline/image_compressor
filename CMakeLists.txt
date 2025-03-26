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
  src/ros/compressed_image_publisher.cpp)
target_link_libraries(compressed_image_publisher
  ${PROJECT_NAME}_lib
  yaml-cpp
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

ament_auto_add_executable(compressed_image_viewer
  src/ros/compressed_image_viewer.cpp)
target_link_libraries(compressed_image_viewer
  ${PROJECT_NAME}_lib
  yaml-cpp
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

# 設定ファイルとLaunchファイルのインストール
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 自動インストール設定
ament_auto_package()