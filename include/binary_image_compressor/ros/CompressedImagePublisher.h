#pragma once

#include <rclcpp/rclcpp.hpp>
#include "binary_image_compressor/msg/compressed_binary_image.hpp"
#include "binary_image_compressor/core/BinaryImageCompressor.h"

namespace compressor
{
  class CompressedImagePublisher : public rclcpp::Node
  {
  public:
    // コンストラクタ
    CompressedImagePublisher();

    // 画像を一度だけ圧縮・パブリッシュするメソッド
    void publishCompressedImageOnce();

  private:
    // 圧縮画像メッセージを作成するヘルパーメソッド
    binary_image_compressor::msg::CompressedBinaryImage createCompressedImageMessage(const BinaryImageCompressor &compressor);
    std::string getPackagePath(const std::string &relative_path);

    // パブリッシャー
    rclcpp::Publisher<binary_image_compressor::msg::CompressedBinaryImage>::SharedPtr compressed_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;
  };
} // namespace compressor