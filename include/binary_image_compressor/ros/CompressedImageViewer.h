#ifndef COMPRESSED_IMAGE_VIEWER_H
#define COMPRESSED_IMAGE_VIEWER_H

#include "rclcpp/rclcpp.hpp"
#include "binary_image_compressor/core/BlockProcessor.h"
#include "binary_image_compressor/model/ImageHeader.h"
#include "binary_image_compressor/msg/compressed_binary_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

namespace compressor
{
  class CompressedImageViewer : public rclcpp::Node
  {
    // クラスの内容は省略
  };
} // namespace compressor

#endif // COMPRESSED_IMAGE_VIEWER_H