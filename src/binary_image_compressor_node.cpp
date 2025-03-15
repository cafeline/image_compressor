#include <rclcpp/rclcpp.hpp>
#include "binary_image_compressor/compressor/BinaryImageCompressor.h"
#include <memory>
#include <string>
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <filesystem>
#include <vector>

class BinaryImageCompressorNode : public rclcpp::Node
{
public:
  BinaryImageCompressorNode() : Node("binary_image_compressor_node")
  {
    // パラメータの宣言
    this->declare_parameter("input_file", "");
    this->declare_parameter("output_file", "");
    this->declare_parameter("block_size", 8);
    this->declare_parameter("threshold", 128);
    this->declare_parameter("mode", "compress"); // compress または decompress

    // サービスの提供
    process_service_ = this->create_service<std_srvs::srv::Trigger>(
        "process_image",
        std::bind(&BinaryImageCompressorNode::processImage, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "バイナリー画像圧縮ノードが初期化されました");
  }

  void processCommandLineArgs(int argc, char **argv)
  {
    // コマンドライン引数からファイルパスを取得
    if (argc >= 3)
    {
      // ROS 2システム引数かどうかチェック
      if (argv[1][0] == '-' || argv[2][0] == '-')
      {
        RCLCPP_INFO(this->get_logger(), "ROS 2システム引数が検出されました。パラメータ処理をスキップします。");
        return;
      }

      std::string input_file = argv[1];
      std::string output_file = argv[2];

      // パラメータの設定
      this->set_parameter(rclcpp::Parameter("input_file", input_file));
      this->set_parameter(rclcpp::Parameter("output_file", output_file));

      // 任意のブロックサイズとしきい値
      if (argc >= 4)
      {
        try
        {
          int block_size = std::stoi(argv[3]);
          this->set_parameter(rclcpp::Parameter("block_size", block_size));
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(this->get_logger(), "ブロックサイズの変換に失敗しました: %s. デフォルト値を使用します。", e.what());
        }
      }

      if (argc >= 5)
      {
        int threshold = std::stoi(argv[4]);
        this->set_parameter(rclcpp::Parameter("threshold", threshold));
      }

      if (argc >= 6)
      {
        std::string mode = argv[5];
        this->set_parameter(rclcpp::Parameter("mode", mode));
      }

      // 画像処理を実行
      processImageFromParams();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "使用方法: ros2 run binary_image_compressor compressor_node input.pgm output.pgm [block_size] [threshold] [mode]");
    }
  }

  void processImageFromParams()
  {
    try
    {
      // パラメータの取得
      std::string input_file = this->get_parameter("input_file").as_string();
      std::string output_file = this->get_parameter("output_file").as_string();
      int block_size = this->get_parameter("block_size").as_int();
      int threshold = this->get_parameter("threshold").as_int();
      std::string mode = this->get_parameter("mode").as_string();

      // 環境変数の展開
      if (input_file.find("$HOME") != std::string::npos)
      {
        const char *home = std::getenv("HOME");
        if (home)
        {
          input_file.replace(input_file.find("$HOME"), 5, home);
        }
      }

      if (output_file.find("$HOME") != std::string::npos)
      {
        const char *home = std::getenv("HOME");
        if (home)
        {
          output_file.replace(output_file.find("$HOME"), 5, home);
        }
      }

      RCLCPP_INFO(this->get_logger(), "展開後のパス: 入力=%s, 出力=%s",
                  input_file.c_str(), output_file.c_str());

      // ファイルの存在確認
      std::ifstream file_check(input_file);
      if (!file_check)
      {
        RCLCPP_ERROR(this->get_logger(), "入力ファイルが見つかりません: %s", input_file.c_str());
        return;
      }

      // ファイルの最初の数バイトを読み込んで確認
      char header_buffer[100] = {0};
      file_check.read(header_buffer, 20);
      file_check.close();

      RCLCPP_INFO(this->get_logger(), "ファイルヘッダー: %.20s", header_buffer);

      // ファイルが正常なPGM形式かチェック
      if (strncmp(header_buffer, "P5", 2) != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "ファイルはPGM形式ではありません");
        // サンプルPGMファイルの作成
        createSamplePGM(input_file);
        RCLCPP_INFO(this->get_logger(), "サンプルPGMファイルを作成しました: %s", input_file.c_str());
      }

      // パラメータ値の検証
      RCLCPP_INFO(this->get_logger(), "デバッグ: パラメータ値の検証");
      if (block_size <= 0 || block_size > 64)
      {
        RCLCPP_WARN(this->get_logger(), "無効なブロックサイズ: %ld (1-64の範囲で指定してください)", block_size);
        block_size = 8; // デフォルト値を使用
      }

      if (threshold < 0 || threshold > 255)
      {
        RCLCPP_WARN(this->get_logger(), "無効なしきい値: %ld (0-255の範囲で指定してください)", threshold);
        threshold = 128; // デフォルト値を使用
      }

      RCLCPP_INFO(this->get_logger(), "処理開始: 入力=%s, 出力=%s, ブロックサイズ=%ld, しきい値=%ld, モード=%s",
                  input_file.c_str(), output_file.c_str(), block_size, threshold, mode.c_str());

      // ディレクトリの作成（出力先ディレクトリが存在しない場合）
      std::filesystem::path output_path(output_file);
      std::filesystem::path output_dir = output_path.parent_path();
      if (!output_dir.empty())
      {
        try
        {
          std::filesystem::create_directories(output_dir);
          RCLCPP_INFO(this->get_logger(), "出力ディレクトリを確認/作成しました: %s", output_dir.string().c_str());
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(this->get_logger(), "出力ディレクトリの作成に失敗: %s", e.what());
        }
      }

      RCLCPP_INFO(this->get_logger(), "デバッグ: 圧縮器のインスタンス化を開始");
      // 圧縮器のインスタンス化
      compressor::BinaryImageCompressor compressor(input_file, output_file, block_size, threshold);
      RCLCPP_INFO(this->get_logger(), "デバッグ: 圧縮器のインスタンス化完了");

      bool result = false;
      RCLCPP_INFO(this->get_logger(), "デバッグ: 処理モード=%s", mode.c_str());
      if (mode == "compress")
      {
        RCLCPP_INFO(this->get_logger(), "デバッグ: 圧縮処理を開始");
        result = compressor.compress();
        if (result)
        {
          compressor.printCompressionInfo();
        }
      }
      else if (mode == "decompress")
      {
        result = compressor.decompress();
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "無効なモード: %s (compress または decompress を指定)", mode.c_str());
        return;
      }

      if (result)
      {
        RCLCPP_INFO(this->get_logger(), "処理が成功しました");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "処理に失敗しました");
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "パラメータ処理中のエラー: %s", e.what());
      return;
    }
  }

  void processImage(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try
    {
      processImageFromParams();
      response->success = true;
      response->message = "画像処理が完了しました";
    }
    catch (const std::exception &e)
    {
      response->success = false;
      response->message = "エラー: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "処理エラー: %s", e.what());
    }
  }

  // サンプルPGMファイルを作成する関数
  void createSamplePGM(const std::string &filename)
  {
    std::ofstream file(filename, std::ios::binary);
    if (!file)
    {
      RCLCPP_ERROR(this->get_logger(), "サンプルファイルを作成できません");
      return;
    }

    // ヘッダー情報
    file << "P5\n10 10\n255\n";

    // 10x10の画像データ (すべて0で黒い画像)
    std::vector<char> data(100, 0);
    file.write(data.data(), data.size());

    file.close();
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr process_service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BinaryImageCompressorNode>();

  // デバッグ情報出力
  try
  {
    RCLCPP_INFO(node->get_logger(), "パラメータ確認: input_file=%s, output_file=%s, block_size=%ld, threshold=%ld, mode=%s",
                node->get_parameter("input_file").as_string().c_str(),
                node->get_parameter("output_file").as_string().c_str(),
                node->get_parameter("block_size").as_int(),
                node->get_parameter("threshold").as_int(),
                node->get_parameter("mode").as_string().c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "パラメータ確認中のエラー: %s", e.what());
  }

  // 特定のパターンのコマンドライン引数があるときのみ処理（ROS 2システム引数を無視）
  // 最初の引数がファイルパスかどうかで判断
  if (argc > 1 && argv[1][0] != '-')
  {
    node->processCommandLineArgs(argc, argv);
  }
  else
  {
    // パラメータが設定されていれば自動的に処理実行
    if (!node->get_parameter("input_file").as_string().empty() &&
        !node->get_parameter("output_file").as_string().empty())
    {
      node->processImageFromParams();
    }
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}