#include <chrono>
#include <memory>
#include <vector>
#include <filesystem>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "binary_image_compressor/msg/compressed_binary_image.hpp"
#include "binary_image_compressor/compressor/BinaryImageCompressor.h"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class CompressedImagePublisher : public rclcpp::Node
{
public:
  CompressedImagePublisher() : Node("compressed_image_publisher")
  {
    // パラメータの宣言
    this->declare_parameter("input_file", "");
    this->declare_parameter("output_file", "");
    this->declare_parameter("block_size", 8);
    this->declare_parameter("threshold", 128);
    this->declare_parameter("publish_rate", 1.0); // Hz

    // パラメータの取得
    input_file_ = this->get_parameter("input_file").as_string();
    output_file_ = this->get_parameter("output_file").as_string();
    block_size_ = this->get_parameter("block_size").as_int();
    threshold_ = this->get_parameter("threshold").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // パブリッシャーの作成
    publisher_ = this->create_publisher<binary_image_compressor::msg::CompressedBinaryImage>(
        "compressed_binary_image", 10);

    // タイマーの設定
    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
        publish_period, std::bind(&CompressedImagePublisher::publish_compressed_image, this));

    RCLCPP_INFO(this->get_logger(), "圧縮画像パブリッシャーが初期化されました");
    RCLCPP_INFO(this->get_logger(), "入力ファイル: %s", input_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "ブロックサイズ: %d", block_size_);
  }

private:
  void publish_compressed_image()
  {
    if (input_file_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "入力ファイルが指定されていません");
      return;
    }

    try {
      // 一時ファイルパスの設定
      fs::path input_path(input_file_);
      std::string base_name = input_path.stem().string();
      std::string temp_dir = "temp";
      fs::create_directories(temp_dir);

      std::string pattern_data_path = temp_dir + "/" + base_name + "_pattern.bin";
      std::string index_data_path = temp_dir + "/" + base_name + "_index.bin";
      std::string dictionary_path = temp_dir + "/" + base_name + "_dict.bin";

      // 画像圧縮の実行
      compressor::BinaryImageCompressor compressor(input_file_, output_file_, block_size_, threshold_);

      // ここで圧縮処理を実行
      if (!compressor.compress()) {
        RCLCPP_ERROR(this->get_logger(), "画像の圧縮に失敗しました");
        return;
      }

      // メッセージの作成
      auto message = binary_image_compressor::msg::CompressedBinaryImage();

      // ヘッダーの設定
      message.header.stamp = this->now();
      message.header.frame_id = "map_frame";

      // 元画像の情報を取得
      compressor::ImageHeader header;
      std::vector<char> header_data;
      if (!compressor::ImageIO::parseHeader(input_file_, header, header_data)) {
        RCLCPP_ERROR(this->get_logger(), "画像ヘッダーの解析に失敗しました");
        return;
      }

      // 元画像情報の設定
      message.original_width = header.width;
      message.original_height = header.height;
      message.block_size = block_size_;
      message.compression_ratio = compressor.calculateCompressionRatio();

      // 辞書ファイルの読み込み
      std::ifstream dict_file(dictionary_path, std::ios::binary);
      if (!dict_file) {
        RCLCPP_ERROR(this->get_logger(), "辞書ファイルを開けません: %s", dictionary_path.c_str());
        return;
      }

      // パターン数の読み込み
      uint16_t pattern_count;
      dict_file.read(reinterpret_cast<char*>(&pattern_count), sizeof(uint16_t));
      message.pattern_count = pattern_count;

      // パターンサイズの計算
      int pattern_bits = block_size_ * block_size_;
      int pattern_bytes = (pattern_bits + 7) / 8;
      message.pattern_bytes = pattern_bytes;

      // パターンデータの読み込み
      std::vector<compressor::BinaryPattern> patterns;
      uint16_t actual_pattern_count;
      if (!compressor::DictionaryBuilder().loadDictionary(dictionary_path, patterns, actual_pattern_count, pattern_bytes)) {
        RCLCPP_ERROR(this->get_logger(), "辞書の読み込みに失敗しました");
        return;
      }

      // パターンデータの設定
      for (const auto& pattern : patterns) {
        message.pattern_data.insert(message.pattern_data.end(), pattern.pattern.begin(), pattern.pattern.end());
      }

      // インデックスファイルの読み込み
      std::ifstream index_file(index_data_path, std::ios::binary);
      if (!index_file) {
        RCLCPP_ERROR(this->get_logger(), "インデックスファイルを開けません: %s", index_data_path.c_str());
        return;
      }

      // ブロック数の計算
      int row_blocks, col_blocks, total_blocks;
      compressor::BlockProcessor bp(block_size_, block_size_);
      bp.calculateBlockCount(header, row_blocks, col_blocks, total_blocks);

      // インデックスデータの読み込み
      message.block_indices.resize(total_blocks);
      for (int i = 0; i < total_blocks; ++i) {
        uint16_t index;
        if (!index_file.read(reinterpret_cast<char*>(&index), sizeof(uint16_t))) {
          RCLCPP_ERROR(this->get_logger(), "インデックスデータの読み込みエラー");
          return;
        }
        message.block_indices[i] = index;
      }

      // メッセージのパブリッシュ
      RCLCPP_INFO(this->get_logger(), "圧縮画像データをパブリッシュします");
      RCLCPP_INFO(this->get_logger(), "  - 画像サイズ: %dx%d", message.original_width, message.original_height);
      RCLCPP_INFO(this->get_logger(), "  - ブロックサイズ: %d", message.block_size);
      RCLCPP_INFO(this->get_logger(), "  - パターン数: %d", message.pattern_count);
      RCLCPP_INFO(this->get_logger(), "  - 圧縮率: %.2f%%", message.compression_ratio);

      publisher_->publish(message);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "例外発生: %s", e.what());
    }
  }

  // メンバ変数
  std::string input_file_;
  std::string output_file_;
  int block_size_;
  int threshold_;
  rclcpp::Publisher<binary_image_compressor::msg::CompressedBinaryImage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CompressedImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}