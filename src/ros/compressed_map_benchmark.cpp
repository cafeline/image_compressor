#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <std_msgs/msg/float64.hpp>
#include "binary_image_compressor/msg/compressed_binary_image.hpp"
#include <chrono>
#include <random>
#include <vector>
#include <numeric>
#include <cmath>

namespace compressor
{

// 簡単な圧縮地図クラス（emcl2_ros2に依存しない）
class SimplifiedCompressedMap
{
public:
  SimplifiedCompressedMap(
    uint32_t width, uint32_t height, double resolution,
    double origin_x, double origin_y, uint8_t block_size,
    const std::vector<std::vector<int8_t>>& patterns,
    const std::vector<uint16_t>& block_indices)
  : width_(width), height_(height), resolution_(resolution),
    origin_x_(origin_x), origin_y_(origin_y), block_size_(block_size),
    patterns_(patterns), block_indices_(block_indices)
  {
    blocks_per_row_ = (width_ + block_size_ - 1) / block_size_;
  }

  uint8_t getValue(double x, double y) const
  {
    int ix = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    int iy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

    if (ix < 0 || iy < 0 || ix >= static_cast<int>(width_) || iy >= static_cast<int>(height_)) {
      return 0;
    }

    int8_t value = getValueFromPattern(getBlockIndex(ix, iy), ix % block_size_, iy % block_size_);
    return (value > 0) ? 255 : 0;
  }

  uint32_t getWidth() const { return width_; }
  uint32_t getHeight() const { return height_; }
  double getResolution() const { return resolution_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }

private:
  uint16_t getBlockIndex(int grid_x, int grid_y) const
  {
    int block_x = grid_x / block_size_;
    int block_y = grid_y / block_size_;
    return block_indices_[block_y * blocks_per_row_ + block_x];
  }

  int8_t getValueFromPattern(uint16_t pattern_index, int block_x, int block_y) const
  {
    if (pattern_index >= patterns_.size()) {
      return 0;
    }
    return patterns_[pattern_index][block_y * block_size_ + block_x];
  }

  uint32_t width_;
  uint32_t height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  uint8_t block_size_;
  std::vector<std::vector<int8_t>> patterns_;
  std::vector<uint16_t> block_indices_;
  int blocks_per_row_;
};

class CompressedMapBenchmark : public rclcpp::Node
{
public:
  CompressedMapBenchmark() : Node("compressed_map_benchmark")
  {
    // パラメータの宣言
    this->declare_parameter("map_resolution", 0.05);
    this->declare_parameter("map_origin_x", 0.0);
    this->declare_parameter("map_origin_y", 0.0);

    this->declare_parameter("benchmark_iterations", 1000000); // 100万回アクセス
    this->declare_parameter("random_seed", 42);

    // サブスクライバーの作成
    compressed_image_sub_ = create_subscription<binary_image_compressor::msg::CompressedBinaryImage>(
      "/compressed_binary_image",
      10,
      std::bind(&CompressedMapBenchmark::cbCompressedImage, this, std::placeholders::_1));

    // パブリッシャーの作成（結果通知用）
    benchmark_result_pub_ = create_publisher<std_msgs::msg::Float64>(
      "compressed_map_benchmark_result", 10);

    RCLCPP_INFO(get_logger(), "圧縮地図ベンチマークノードを開始しました");
  }

private:
  void cbCompressedImage(const binary_image_compressor::msg::CompressedBinaryImage::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "圧縮地図データを受信しました");
    
    // パターンデータの復元
    std::vector<std::vector<int8_t>> patterns;
    const size_t block_pixel_count = static_cast<size_t>(msg->block_size) * msg->block_size;
    patterns.resize(msg->pattern_count);
    
    for (uint16_t i = 0; i < msg->pattern_count; ++i) {
      patterns[i].resize(block_pixel_count);
      const uint8_t* pattern_start = &msg->pattern_data[i * msg->pattern_bytes];
      
      for (size_t p_idx = 0; p_idx < block_pixel_count; ++p_idx) {
        const size_t byte_idx = p_idx / 8;
        const size_t bit_idx = p_idx % 8;
        bool is_set = (pattern_start[byte_idx] >> (7 - bit_idx)) & 1;
        patterns[i][p_idx] = is_set ? 0 : 100;
      }
    }

    // インデックスデータを取得
    std::vector<uint16_t> block_indices;
    if (msg->use_8bit_indices) {
      // 8bitから16bitに変換
      block_indices.resize(msg->block_indices_8bit.size());
      for (size_t i = 0; i < msg->block_indices_8bit.size(); ++i) {
        block_indices[i] = static_cast<uint16_t>(msg->block_indices_8bit[i]);
      }
    } else {
      block_indices = msg->block_indices_16bit;
    }

    // SimplifiedCompressedMapの作成
    auto compressed_map = std::make_unique<SimplifiedCompressedMap>(
      msg->original_width, msg->original_height,
      this->get_parameter("map_resolution").as_double(),
      this->get_parameter("map_origin_x").as_double(),
      this->get_parameter("map_origin_y").as_double(),
      msg->block_size, patterns, block_indices);

    RCLCPP_INFO(get_logger(), "SimplifiedCompressedMapを作成しました");
    RCLCPP_INFO(get_logger(), "地図サイズ: %dx%d", 
                compressed_map->getWidth(), compressed_map->getHeight());
    RCLCPP_INFO(get_logger(), "解像度: %f", compressed_map->getResolution());
    
    // ベンチマークの実行
    runBenchmarks(compressed_map.get());
  }

  void runBenchmarks(SimplifiedCompressedMap* map)
  {
    int iterations = this->get_parameter("benchmark_iterations").as_int();
    int seed = this->get_parameter("random_seed").as_int();
    
    RCLCPP_INFO(get_logger(), "=== 圧縮地図読み出し性能ベンチマーク開始 ===");
    RCLCPP_INFO(get_logger(), "テスト回数: %d", iterations);
    
    // 1. ランダムアクセステスト
    runRandomAccessBenchmark(map, iterations, seed);
    
    // 2. 連続アクセステスト（行方向）
    runSequentialRowBenchmark(map, iterations);
    
    // 3. 連続アクセステスト（列方向）
    runSequentialColumnBenchmark(map, iterations);
    
    // 4. LiDAR模擬アクセステスト
    runLidarSimulationBenchmark(map, iterations, seed);
    
    // 5. 全地図走査テスト
    runFullMapScanBenchmark(map);
    
    RCLCPP_INFO(get_logger(), "=== ベンチマーク完了 ===");
  }

  void runRandomAccessBenchmark(SimplifiedCompressedMap* map, int iterations, int seed)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> x_dist(map->getOriginX(), 
                                            map->getOriginX() + map->getWidth() * map->getResolution());
    std::uniform_real_distribution<> y_dist(map->getOriginY(), 
                                            map->getOriginY() + map->getHeight() * map->getResolution());
    
    // ウォームアップ
    for (int i = 0; i < 1000; ++i) {
      map->getValue(x_dist(gen), y_dist(gen));
    }
    
    // 測定開始
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < iterations; ++i) {
      map->getValue(x_dist(gen), y_dist(gen));
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    double avg_time_ns = static_cast<double>(duration.count()) / iterations;
    double throughput = iterations / (duration.count() / 1e9); // queries per second
    
    RCLCPP_INFO(get_logger(), "ランダムアクセス: 平均 %.2f ns/query, %.2f M queries/sec", 
                avg_time_ns, throughput / 1e6);
  }

  void runSequentialRowBenchmark(SimplifiedCompressedMap* map, int iterations)
  {
    uint32_t width = map->getWidth();
    uint32_t height = map->getHeight();
    double resolution = map->getResolution();
    double origin_x = map->getOriginX();
    double origin_y = map->getOriginY();
    
    // 測定開始
    auto start = std::chrono::high_resolution_clock::now();
    
    int count = 0;
    uint32_t total_pixels = width * height;
    for (uint32_t iter = 0; iter < static_cast<uint32_t>(iterations) / total_pixels + 1; ++iter) {
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          map->getValue(origin_x + x * resolution, origin_y + y * resolution);
          if (++count >= iterations) goto done;
        }
      }
    }
    done:
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    double avg_time_ns = static_cast<double>(duration.count()) / iterations;
    
    RCLCPP_INFO(get_logger(), "行方向連続アクセス: 平均 %.2f ns/query", avg_time_ns);
  }

  void runSequentialColumnBenchmark(SimplifiedCompressedMap* map, int iterations)
  {
    uint32_t width = map->getWidth();
    uint32_t height = map->getHeight();
    double resolution = map->getResolution();
    double origin_x = map->getOriginX();
    double origin_y = map->getOriginY();
    
    // 測定開始
    auto start = std::chrono::high_resolution_clock::now();
    
    int count = 0;
    uint32_t total_pixels = width * height;
    for (uint32_t iter = 0; iter < static_cast<uint32_t>(iterations) / total_pixels + 1; ++iter) {
      for (uint32_t x = 0; x < width; ++x) {
        for (uint32_t y = 0; y < height; ++y) {
          map->getValue(origin_x + x * resolution, origin_y + y * resolution);
          if (++count >= iterations) goto done;
        }
      }
    }
    done:
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    double avg_time_ns = static_cast<double>(duration.count()) / iterations;
    
    RCLCPP_INFO(get_logger(), "列方向連続アクセス: 平均 %.2f ns/query", avg_time_ns);
  }

  void runLidarSimulationBenchmark(SimplifiedCompressedMap* map, int iterations, int seed)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> x_dist(map->getOriginX(), 
                                            map->getOriginX() + map->getWidth() * map->getResolution());
    std::uniform_real_distribution<> y_dist(map->getOriginY(), 
                                            map->getOriginY() + map->getHeight() * map->getResolution());
    std::uniform_real_distribution<> angle_dist(0, 2 * M_PI);
    
    const int beams_per_scan = 720; // 720本のレーザービーム
    const double max_range = 10.0; // 10メートル
    
    // 測定開始
    auto start = std::chrono::high_resolution_clock::now();
    
    int scans = iterations / beams_per_scan;
    for (int scan = 0; scan < scans; ++scan) {
      double robot_x = x_dist(gen);
      double robot_y = y_dist(gen);
      double robot_angle = angle_dist(gen);
      
      for (int beam = 0; beam < beams_per_scan; ++beam) {
        double angle = robot_angle + (beam * 2 * M_PI / beams_per_scan);
        double end_x = robot_x + max_range * std::cos(angle);
        double end_y = robot_y + max_range * std::sin(angle);
        map->getValue(end_x, end_y);
      }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    double avg_scan_time_us = static_cast<double>(duration.count()) / scans;
    
    RCLCPP_INFO(get_logger(), "LiDAR模擬 (%dビーム/スキャン): 平均 %.2f μs/スキャン", 
                beams_per_scan, avg_scan_time_us);
  }

  void runFullMapScanBenchmark(SimplifiedCompressedMap* map)
  {
    uint32_t width = map->getWidth();
    uint32_t height = map->getHeight();
    double resolution = map->getResolution();
    double origin_x = map->getOriginX();
    double origin_y = map->getOriginY();
    
    RCLCPP_INFO(get_logger(), "全地図走査開始 (%d x %d = %d ピクセル)", 
                width, height, width * height);
    
    // 測定開始
    auto start = std::chrono::high_resolution_clock::now();
    
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        map->getValue(origin_x + x * resolution, origin_y + y * resolution);
      }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    RCLCPP_INFO(get_logger(), "全地図走査時間: %ld ms", duration.count());
    RCLCPP_INFO(get_logger(), "1ピクセルあたり: %.2f ns", 
                static_cast<double>(duration.count() * 1e6) / (width * height));
    
    // 結果をパブリッシュ
    std_msgs::msg::Float64 result;
    result.data = static_cast<double>(duration.count());
    benchmark_result_pub_->publish(result);
  }

  // メンバ変数
  rclcpp::Subscription<binary_image_compressor::msg::CompressedBinaryImage>::SharedPtr compressed_image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr benchmark_result_pub_;
};

} // namespace compressor

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<compressor::CompressedMapBenchmark>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 