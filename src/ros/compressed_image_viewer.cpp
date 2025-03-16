#include <memory>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "binary_image_compressor/msg/compressed_binary_image.hpp"
#include "binary_image_compressor/core/BlockProcessor.h"
#include "binary_image_compressor/model/ImageHeader.h"

class CompressedImageViewer : public rclcpp::Node
{
public:
  CompressedImageViewer() : Node("compressed_image_viewer")
  {
    // サブスクライバーの作成
    subscription_ = this->create_subscription<binary_image_compressor::msg::CompressedBinaryImage>(
        "compressed_binary_image", 10,
        std::bind(&CompressedImageViewer::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "圧縮画像ビューアーが初期化されました");
  }

private:
  void image_callback(const binary_image_compressor::msg::CompressedBinaryImage::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "圧縮画像データを受信しました");
    RCLCPP_INFO(this->get_logger(), "  - 画像サイズ: %dx%d", msg->original_width, msg->original_height);
    RCLCPP_INFO(this->get_logger(), "  - ブロックサイズ: %d", msg->block_size);
    RCLCPP_INFO(this->get_logger(), "  - パターン数: %d", msg->pattern_count);
    RCLCPP_INFO(this->get_logger(), "  - 圧縮率: %.2f%%", msg->compression_ratio);

    try {
      // 画像ヘッダー情報の設定
      compressor::ImageHeader header;
      header.width = msg->original_width;
      header.height = msg->original_height;

      // パターンデータの抽出
      std::vector<std::vector<uint8_t>> patterns;
      extract_patterns(msg, patterns);

      // ブロックプロセッサーの初期化
      compressor::BlockProcessor bp(msg->block_size, msg->block_size);

      // 画像データの再構築
      std::vector<uint8_t> image_data;
      if (!bp.reconstructImage(msg->block_indices, patterns, header, image_data)) {
        RCLCPP_ERROR(this->get_logger(), "画像の再構築に失敗しました");
        return;
      }

      // OpenCVマット形式に変換
      cv::Mat image(header.height, header.width, CV_8UC1, image_data.data());

      // 表示用にサイズを調整
      cv::Mat display_image;
      const int max_display_size = 800;
      if (header.width > max_display_size || header.height > max_display_size) {
        double scale = std::min(
            static_cast<double>(max_display_size) / header.width,
            static_cast<double>(max_display_size) / header.height);
        cv::resize(image, display_image, cv::Size(), scale, scale, cv::INTER_NEAREST);
      } else {
        display_image = image;
      }

      // 画像の表示
      cv::imshow("圧縮画像ビューアー", display_image);
      cv::waitKey(30); // 30ミリ秒待機（ウィンドウの更新用）

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "例外発生: %s", e.what());
    }
  }

  void extract_patterns(const binary_image_compressor::msg::CompressedBinaryImage::SharedPtr msg,
                       std::vector<std::vector<uint8_t>>& patterns)
  {
    patterns.clear();
    patterns.resize(msg->pattern_count);

    // パターンデータのサイズを計算
    size_t pattern_bytes = msg->pattern_bytes;

    // 各パターンを抽出
    for (uint16_t i = 0; i < msg->pattern_count; i++) {
      patterns[i].resize(pattern_bytes);

      // パターンデータをコピー
      for (size_t j = 0; j < pattern_bytes; j++) {
        size_t index = i * pattern_bytes + j;
        if (index < msg->pattern_data.size()) {
          patterns[i][j] = msg->pattern_data[index];
        } else {
          RCLCPP_ERROR(this->get_logger(), "パターンデータのインデックスが範囲外です");
          patterns[i][j] = 0;
        }
      }
    }
  }

  rclcpp::Subscription<binary_image_compressor::msg::CompressedBinaryImage>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CompressedImageViewer>();

  // OpenCVウィンドウの初期化
  cv::namedWindow("圧縮画像ビューアー", cv::WINDOW_AUTOSIZE);

  rclcpp::spin(node);

  // ウィンドウの破棄
  cv::destroyAllWindows();

  rclcpp::shutdown();
  return 0;
}