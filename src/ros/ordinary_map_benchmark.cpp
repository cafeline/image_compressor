#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <random>
#include <vector>
#include <cmath>

namespace compressor
{

// 通常の占有格子地図クラス
class OrdinaryOccupancyMap
{
public:
  OrdinaryOccupancyMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  : width_(msg->info.width), height_(msg->info.height), 
    resolution_(msg->info.resolution),
    origin_x_(msg->info.origin.position.x),
    origin_y_(msg->info.origin.position.y),
    data_(msg->data)
  {
  }

  uint8_t getValue(double x, double y) const
  {
    int ix = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    int iy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

    if (ix < 0 || iy < 0 || ix >= static_cast<int>(width_) || iy >= static_cast<int>(height_)) {
      return 0;
    }

    int8_t value = data_[iy * width_ + ix];
    return value;  // 占有/自由領域
  }

  uint32_t getWidth() const { return width_; }
  uint32_t getHeight() const { return height_; }
  double getResolution() const { return resolution_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }

private:
  uint32_t width_;
  uint32_t height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<int8_t> data_;
};

class OrdinaryMapBenchmark : public rclcpp::Node
{
public:
  OrdinaryMapBenchmark() : Node("ordinary_map_benchmark")
  {
    // パラメータの宣言
    this->declare_parameter("benchmark_iterations", 1000000);
    this->declare_parameter("random_seed", 42);

    // サブスクライバーの作成
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      10,
      std::bind(&OrdinaryMapBenchmark::cbMap, this, std::placeholders::_1));

    // パブリッシャーの作成
    benchmark_result_pub_ = create_publisher<std_msgs::msg::Float64>(
      "ordinary_map_benchmark_result", 10);

    RCLCPP_INFO(get_logger(), "通常地図ベンチマークノードを開始しました");
  }

private:
  void cbMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "占有格子地図を受信しました");
    
    // OrdinaryOccupancyMapの作成
    auto ordinary_map = std::make_unique<OrdinaryOccupancyMap>(msg);

    RCLCPP_INFO(get_logger(), "OrdinaryOccupancyMapを作成しました");
    RCLCPP_INFO(get_logger(), "地図サイズ: %dx%d", 
                ordinary_map->getWidth(), ordinary_map->getHeight());
    RCLCPP_INFO(get_logger(), "解像度: %f", ordinary_map->getResolution());
    
    // ベンチマークの実行
    runBenchmarks(ordinary_map.get());
  }

  void runBenchmarks(OrdinaryOccupancyMap* map)
  {
    int iterations = this->get_parameter("benchmark_iterations").as_int();
    int seed = this->get_parameter("random_seed").as_int();
    
    RCLCPP_INFO(get_logger(), "=== 通常地図読み出し性能ベンチマーク開始 ===");
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

  void runRandomAccessBenchmark(OrdinaryOccupancyMap* map, int iterations, int seed)
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
    double throughput = iterations / (duration.count() / 1e9);
    
    RCLCPP_INFO(get_logger(), "ランダムアクセス: 平均 %.2f ns/query, %.2f M queries/sec", 
                avg_time_ns, throughput / 1e6);
  }

  void runSequentialRowBenchmark(OrdinaryOccupancyMap* map, int iterations)
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

  void runSequentialColumnBenchmark(OrdinaryOccupancyMap* map, int iterations)
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

  void runLidarSimulationBenchmark(OrdinaryOccupancyMap* map, int iterations, int seed)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> x_dist(map->getOriginX(), 
                                            map->getOriginX() + map->getWidth() * map->getResolution());
    std::uniform_real_distribution<> y_dist(map->getOriginY(), 
                                            map->getOriginY() + map->getHeight() * map->getResolution());
    std::uniform_real_distribution<> angle_dist(0, 2 * M_PI);
    
    const int beams_per_scan = 720;
    const double max_range = 10.0;
    const int num_tests = 1000; // 統計的信頼性のために1000回テスト
    
    std::vector<double> scan_times;
    scan_times.reserve(num_tests);
    
    // 1000回のスキャンテスト
    for (int test = 0; test < num_tests; ++test) {
      double robot_x = x_dist(gen);
      double robot_y = y_dist(gen);
      double robot_angle = angle_dist(gen);
      
      // 1スキャンの測定開始
      auto start = std::chrono::high_resolution_clock::now();

      // 1スキャン分の720ビームを処理
      for (int beam = 0; beam < beams_per_scan; ++beam) {
        double angle = robot_angle + (beam * 2 * M_PI / beams_per_scan);
        double end_x = robot_x + max_range * std::cos(angle);
        double end_y = robot_y + max_range * std::sin(angle);
        map->getValue(end_x, end_y);
      }
      
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
      scan_times.push_back(duration.count() / 1000.0); // マイクロ秒に変換
    }
    
    // 統計計算
    double total_time = 0.0;
    for (double time : scan_times) {
      total_time += time;
    }
    double avg_scan_time_us = total_time / num_tests;
    double avg_beam_time_ns = (avg_scan_time_us * 1000.0) / beams_per_scan;
    
    RCLCPP_INFO(get_logger(), "LiDAR模擬 (%dビーム/スキャン、%d回テスト): 平均 %.2f μs/スキャン", 
                beams_per_scan, num_tests, avg_scan_time_us);
    RCLCPP_INFO(get_logger(), "1ビームあたり: 平均 %.2f ns", avg_beam_time_ns);
  }

  void runFullMapScanBenchmark(OrdinaryOccupancyMap* map)
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
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    RCLCPP_INFO(get_logger(), "全地図走査時間: %ld μs", duration.count());
    RCLCPP_INFO(get_logger(), "1ピクセルあたり: %.2f ns", 
                static_cast<double>(duration.count() * 1e6) / (width * height));
    
    // 結果をパブリッシュ
    std_msgs::msg::Float64 result;
    result.data = static_cast<double>(duration.count());
    benchmark_result_pub_->publish(result);
  }

  // メンバ変数
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr benchmark_result_pub_;
};

} // namespace compressor

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<compressor::OrdinaryMapBenchmark>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 