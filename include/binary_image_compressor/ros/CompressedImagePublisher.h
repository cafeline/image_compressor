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
    // クラスの内容は省略
  };
} // namespace compressor

#endif // COMPRESSED_IMAGE_PUBLISHER_H