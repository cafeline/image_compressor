#include <chrono>
#include <memory>
#include <vector>
#include <filesystem>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "binary_image_compressor/msg/compressed_binary_image.hpp"
#include "binary_image_compressor/core/BinaryImageCompressor.h"
#include "binary_image_compressor/ros/CompressedImagePublisher.h"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace compressor
{
  CompressedImagePublisher::CompressedImagePublisher() : Node("compressed_image_publisher")
  {
    // パラメータの宣言
    this->declare_parameter("input_file", "");
    this->declare_parameter("block_size", 8);
    this->declare_parameter("threshold", 128);
    this->declare_parameter("publish_rate", 1.0);

    // パブリッシャーの作成
    compressed_pub_ = this->create_publisher<binary_image_compressor::msg::CompressedBinaryImage>(
        "compressed_binary_image", 10);

    // 一度だけ圧縮して送信するためのワンショットタイマー
    timer_ = this->create_wall_timer(1s, std::bind(&CompressedImagePublisher::publishCompressedImageOnce, this));
  }

  void CompressedImagePublisher::publishCompressedImageOnce()
  {
    // ワンショットなのでタイマーをキャンセル（一度だけ実行）
    if (timer_)
    {
      timer_->cancel();
    }

    std::string input_file = this->get_parameter("input_file").as_string();
    int block_size = this->get_parameter("block_size").as_int();
    int threshold = this->get_parameter("threshold").as_int();

    RCLCPP_INFO(this->get_logger(), "画像の圧縮を開始します: %s", input_file.c_str());

    if (input_file.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "入力ファイルが指定されていません");
      return;
    }

    // 一時出力ファイルパスの作成
    fs::path input_path(input_file);
    std::string base_name = input_path.stem().string();
    std::string temp_dir = "temp";

    // 一時ディレクトリの作成（存在しない場合）
    if (!fs::exists(temp_dir))
    {
      fs::create_directory(temp_dir);
    }

    std::string temp_output_file = temp_dir + "/" + base_name + "_compressed.pgm";

    // 圧縮処理 - 必ず出力パスを指定
    BinaryImageCompressor compressor(input_file, temp_output_file, block_size, threshold);
    if (!compressor.compress())
    {
      RCLCPP_ERROR(this->get_logger(), "画像の圧縮に失敗しました");
      return;
    }

    // 圧縮結果からメッセージを作成
    auto message = createCompressedImageMessage(compressor);

    // メッセージをパブリッシュ
    RCLCPP_INFO(this->get_logger(), "圧縮画像をパブリッシュします (サイズ: %zu バイト)",
                message.pattern_data.size() + message.block_indices.size() * sizeof(uint16_t));
    compressed_pub_->publish(message);

    RCLCPP_INFO(this->get_logger(), "圧縮画像のパブリッシュが完了しました - プログラムを終了するには Ctrl+C を押してください");
  }

  binary_image_compressor::msg::CompressedBinaryImage CompressedImagePublisher::createCompressedImageMessage(const BinaryImageCompressor &compressor)
  {
    // メッセージの作成
    auto message = binary_image_compressor::msg::CompressedBinaryImage();

    // ヘッダーの設定
    message.header.stamp = this->now();
    message.header.frame_id = "map_frame";

    // 元画像の情報を取得
    ImageHeader header;
    std::vector<char> header_data;
    if (!ImageIO::parseHeader(this->get_parameter("input_file").as_string(), header, header_data))
    {
      RCLCPP_ERROR(this->get_logger(), "画像ヘッダーの解析に失敗しました");
      return message;
    }

    // 元画像情報の設定
    message.original_width = header.width;
    message.original_height = header.height;
    message.block_size = this->get_parameter("block_size").as_int();
    message.compression_ratio = compressor.calculateCompressionRatio();

    // 一時ファイルパスの設定
    fs::path input_path(this->get_parameter("input_file").as_string());
    std::string base_name = input_path.stem().string();
    std::string temp_dir = "temp";

    std::string dictionary_path = temp_dir + "/" + base_name + "_dict.bin";
    std::string index_data_path = temp_dir + "/" + base_name + "_index.bin";

    // パターンサイズの計算
    int block_size = this->get_parameter("block_size").as_int();
    int pattern_bits = block_size * block_size;
    int pattern_bytes = (pattern_bits + 7) / 8;
    message.pattern_bytes = pattern_bytes;

    // 辞書とパターンの読み込み
    std::vector<BinaryPattern> patterns;
    uint16_t pattern_count;
    if (!DictionaryBuilder().loadDictionary(dictionary_path, patterns, pattern_count, pattern_bytes))
    {
      RCLCPP_ERROR(this->get_logger(), "辞書の読み込みに失敗しました");
      return message;
    }

    message.pattern_count = pattern_count;

    // パターンデータの設定
    for (const auto &pattern : patterns)
    {
      message.pattern_data.insert(message.pattern_data.end(), pattern.pattern.begin(), pattern.pattern.end());
    }

    // インデックスファイルの読み込み
    std::ifstream index_file(index_data_path, std::ios::binary);
    if (!index_file)
    {
      RCLCPP_ERROR(this->get_logger(), "インデックスファイルを開けません: %s", index_data_path.c_str());
      return message;
    }

    // ブロック数の計算
    int row_blocks, col_blocks, total_blocks;
    BlockProcessor bp(block_size, block_size);
    bp.calculateBlockCount(header, row_blocks, col_blocks, total_blocks);

    // インデックスデータの読み込み
    message.block_indices.resize(total_blocks);
    index_file.read(reinterpret_cast<char *>(message.block_indices.data()), total_blocks * sizeof(uint16_t));

    return message;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<compressor::CompressedImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}