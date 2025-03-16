#ifndef COMPRESSED_IMAGE_PUBLISHER_H
#define COMPRESSED_IMAGE_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "binary_image_compressor/core/BinaryImageCompressor.h"
#include "binary_image_compressor/msg/compressed_binary_image.hpp"
#include "sensor_msgs/msg/image.hpp"

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
    binary_image_compressor::msg::CompressedBinaryImage
    createCompressedImageMessage(const BinaryImageCompressor &compressor);

    // パブリッシャー
    rclcpp::Publisher<binary_image_compressor::msg::CompressedBinaryImage>::SharedPtr compressed_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;
  };
} // namespace compressor

#endif // COMPRESSED_IMAGE_PUBLISHER_H