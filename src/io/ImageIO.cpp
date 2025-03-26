#include "binary_image_compressor/io/ImageIO.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;
namespace compressor
{

  bool ImageIO::parseHeader(const std::string &filePath,
                            ImageHeader &header,
                            std::vector<char> &headerData)
  {
    // ファイル拡張子の確認
    std::string extension = filePath.substr(filePath.find_last_of(".") + 1);
    if (extension == "yaml" || extension == "yml")
    {
      return parseYamlHeader(filePath, header, headerData);
    }
    else
    {
      return parsePgmHeader(filePath, header, headerData);
    }
  }

  bool ImageIO::parseYamlHeader(const std::string &filePath,
                                ImageHeader &header,
                                std::vector<char> &headerData)
  {
    try
    {
      // YAMLファイルから参照先のPGMファイルパスを取得
      std::string pgmPath = getPgmPathFromYaml(filePath);
      std::cout << "PGMファイルの絶対パス: " << pgmPath << std::endl;

      // PGMファイルのヘッダーを解析
      return parsePgmHeader(pgmPath, header, headerData);
    }
    catch (const std::exception &e)
    {
      std::cerr << "YAMLファイルの解析エラー: " << e.what() << std::endl;
      return false;
    }
  }

  bool ImageIO::parsePgmHeader(const std::string &filePath,
                               ImageHeader &header,
                               std::vector<char> &headerData)
  {
    std::ifstream inputFile(filePath, std::ios::binary);
    if (!inputFile)
    {
      std::cerr << "入力ファイルを開けません: " << filePath << std::endl;
      return false;
    }

    // ヘッダー読み込み処理
    std::string line;
    headerData.clear();
    char c;

    try
    {
      // マジックナンバーの確認（P5またはP2）
      inputFile >> line;
      std::cout << "PGMマジックナンバー: " << line << std::endl;
      if (line != "P5" && line != "P2")
      {
        std::cerr << "無効なPGMファイル形式: " << line << std::endl;
        return false;
      }
      header.isPGMBinary = (line == "P5");

      // 改行文字を取得
      inputFile.get(c);

      // コメント行のスキップ
      while (inputFile.peek() == '#')
      {
        std::getline(inputFile, line);
        std::cout << "PGMコメント行: " << line << std::endl;
      }

      // 幅と高さの読み込み
      int width, height;
      inputFile >> width >> height;
      std::cout << "PGM画像サイズ: " << width << "x" << height << std::endl;

      if (width <= 0 || height <= 0)
      {
        std::cerr << "無効な画像サイズ: " << width << "x" << height << std::endl;
        return false;
      }

      // 最大輝度値の読み込み
      int maxval;
      inputFile >> maxval;
      std::cout << "PGM最大輝度値: " << maxval << std::endl;

      if (maxval <= 0 || maxval > 255)
      {
        std::cerr << "無効な最大輝度値: " << maxval << std::endl;
        return false;
      }

      // ヘッダー情報の設定
      header.width = width;
      header.height = height;

      // ヘッダーの終わりの位置を記録
      std::streampos headerEnd = inputFile.tellg();
      inputFile.get(c); // データ部分の直前の1バイトを読み込む
      header.start = static_cast<size_t>(headerEnd) + 1;
      std::cout << "PGMヘッダーサイズ: " << header.start << " バイト" << std::endl;

      // ここまでの内容をヘッダーデータとして保存
      inputFile.seekg(0, std::ios::beg);
      headerData.resize(static_cast<size_t>(headerEnd) + 1);
      inputFile.read(headerData.data(), headerData.size());

      // P2形式の場合、P5形式のヘッダーを生成
      if (!header.isPGMBinary)
      {
        std::string p5Header = "P5\n";
        p5Header += std::to_string(width) + " " + std::to_string(height) + "\n";
        p5Header += std::to_string(maxval) + "\n";
        headerData.assign(p5Header.begin(), p5Header.end());
        header.start = headerData.size();
        std::cout << "ASCII PGMをバイナリPGMに変換するためのヘッダーを生成: " << header.start << " バイト" << std::endl;
      }

      return true;
    }
    catch (const std::exception &e)
    {
      std::cerr << "PGMヘッダー解析エラー: " << e.what() << std::endl;
      return false;
    }
  }

  bool ImageIO::binarizeImage(const std::string &inputPath,
                              const std::string &outputPath,
                              const ImageHeader &header,
                              const std::vector<char> &headerData,
                              int threshold)
  {
    std::cout << "画像の2値化を実行中..." << std::endl;

    // 入力パスを分析
    std::cout << "入力ファイル: " << inputPath << std::endl;
    std::cout << "出力ファイル: " << outputPath << std::endl;
    std::cout << "画像サイズ: " << header.width << "x" << header.height << std::endl;
    std::cout << "2値化閾値: " << threshold << std::endl;

    std::ifstream input(inputPath, std::ios::binary);
    if (!input)
    {
      std::cerr << "入力ファイルを開けません: " << inputPath << std::endl;
      return false;
    }

    // ファイル拡張子の確認
    std::string extension = inputPath.substr(inputPath.find_last_of(".") + 1);
    std::vector<uint8_t> imageData;

    if (extension == "yaml" || extension == "yml")
    {
      // YAMLファイルの場合は、参照されているPGMファイルを読み込む
      try
      {
        // YAMLファイルから参照先のPGMファイルパスを取得
        std::string pgmPath = getPgmPathFromYaml(inputPath);
        std::cout << "PGMファイルの絶対パス: " << pgmPath << std::endl;

        // PGMファイルを読み込む
        input.close();
        input.open(pgmPath, std::ios::binary);
        if (!input)
        {
          std::cerr << "PGMファイルを開けません: " << pgmPath << std::endl;
          return false;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "YAMLファイルの解析エラー: " << e.what() << std::endl;
        return false;
      }
    }

    // 画像データの読み込み
    imageData.resize(header.width * header.height);

    if (header.isPGMBinary)
    {
      // バイナリPGM形式 (P5)
      input.seekg(header.start);
      std::cout << "ヘッダーサイズ（スキップ）: " << header.start << " バイト" << std::endl;
      if (!input.read(reinterpret_cast<char *>(imageData.data()), imageData.size()))
      {
        std::cerr << "画像データの読み込みエラー" << std::endl;
        return false;
      }
    }
    else
    {
      // ASCII PGM形式 (P2)
      std::cout << "ASCII PGM形式を読み込んでいます" << std::endl;

      // ヘッダー部分をスキップ
      input.seekg(header.start);
      std::cout << "ヘッダーサイズ（スキップ）: " << header.start << " バイト" << std::endl;

      // ASCII形式ではピクセル値が空白で区切られたテキストとして格納されている
      for (size_t i = 0; i < imageData.size(); ++i)
      {
        int pixelValue;
        input >> pixelValue;

        if (input.fail())
        {
          std::cerr << "ピクセルデータの読み込みエラー（位置: " << i << "）" << std::endl;
          return false;
        }

        imageData[i] = static_cast<uint8_t>(pixelValue);
      }
    }

    input.close();

    // 画像データの統計情報を表示
    int minVal = 255, maxVal = 0;
    long long sum = 0;
    for (const auto &pixel : imageData)
    {
      minVal = std::min(minVal, static_cast<int>(pixel));
      maxVal = std::max(maxVal, static_cast<int>(pixel));
      sum += pixel;
    }
    double avgVal = static_cast<double>(sum) / imageData.size();
    std::cout << "画像統計 - 最小値: " << minVal << ", 最大値: " << maxVal << ", 平均値: " << avgVal << std::endl;

    // 2値化処理
    for (auto &pixel : imageData)
    {
      pixel = (pixel >= threshold) ? 255 : 0;
    }

    // 2値化後の統計情報を表示
    int zeroCount = 0, fullCount = 0;
    for (const auto &pixel : imageData)
    {
      if (pixel == 0)
        zeroCount++;
      if (pixel == 255)
        fullCount++;
    }
    std::cout << "2値化後 - 0の数: " << zeroCount << ", 255の数: " << fullCount << std::endl;

    // 閾値が適切かどうかを調査
    if (zeroCount == 0 || fullCount == 0)
    {
      std::cout << "警告: 2値化後の画像が単色になっています。閾値の調整が必要かもしれません。" << std::endl;
      std::cout << "現在の閾値: " << threshold << ", 推奨閾値: " << static_cast<int>(avgVal) << std::endl;

      // 閾値を調整して再試行
      int newThreshold = static_cast<int>(avgVal);
      if (newThreshold == threshold)
      {
        // 閾値が同じ場合は、最小値と最大値の中間を使用
        newThreshold = (minVal + maxVal) / 2;
      }

      std::cout << "閾値を自動調整: " << threshold << " -> " << newThreshold << std::endl;

      // 閾値を調整して2値化をやり直し
      for (auto &pixel : imageData)
      {
        pixel = (pixel >= newThreshold) ? 255 : 0;
      }

      // 再度統計情報を表示
      zeroCount = 0;
      fullCount = 0;
      for (const auto &pixel : imageData)
      {
        if (pixel == 0)
          zeroCount++;
        if (pixel == 255)
          fullCount++;
      }
      std::cout << "調整後の2値化 - 0の数: " << zeroCount << ", 255の数: " << fullCount << std::endl;
    }

    // 出力ファイルの作成
    std::ofstream output(outputPath, std::ios::binary);
    if (!output)
    {
      std::cerr << "出力ファイルを作成できません: " << outputPath << std::endl;
      return false;
    }

    // ヘッダーの書き込み
    output.write(headerData.data(), headerData.size());

    // 2値化画像データの書き込み
    output.write(reinterpret_cast<char *>(imageData.data()), imageData.size());

    output.close();
    std::cout << "2値化完了: " << outputPath << std::endl;

    return true;
  }

  bool ImageIO::readYamlImageData(const std::string &filePath,
                                  std::vector<uint8_t> &imageData)
  {
    try
    {
      // YAMLファイルから参照先のPGMファイルパスを取得
      std::string pgmPath = getPgmPathFromYaml(filePath);
      std::cout << "PGMファイルの絶対パス: " << pgmPath << std::endl;

      // PGMファイルを読み込む
      std::ifstream input(pgmPath, std::ios::binary);
      if (!input)
      {
        std::cerr << "PGMファイルを開けません: " << pgmPath << std::endl;
        return false;
      }

      // ヘッダーを読み込む
      std::string magic;
      int width, height, maxval;
      input >> magic;

      // コメント行のスキップ
      while (input.peek() == '#')
      {
        std::string comment;
        std::getline(input, comment);
      }

      input >> width >> height >> maxval;

      bool isBinary = (magic == "P5");

      // ヘッダーの終わりの位置を記録
      char c;
      input.get(c); // データ部分の直前の1バイトを読み込む

      // 画像データの読み込み
      imageData.resize(width * height);

      if (isBinary)
      {
        // バイナリPGM形式 (P5)
        if (!input.read(reinterpret_cast<char *>(imageData.data()), imageData.size()))
        {
          std::cerr << "画像データの読み込みエラー" << std::endl;
          return false;
        }
      }
      else
      {
        // ASCII PGM形式 (P2)
        for (size_t i = 0; i < imageData.size(); ++i)
        {
          int pixelValue;
          input >> pixelValue;

          if (input.fail())
          {
            std::cerr << "ピクセルデータの読み込みエラー（位置: " << i << "）" << std::endl;
            return false;
          }

          imageData[i] = static_cast<uint8_t>(pixelValue);
        }
      }

      input.close();
      return true;
    }
    catch (const std::exception &e)
    {
      std::cerr << "YAMLデータの読み込みエラー: " << e.what() << std::endl;
      return false;
    }
  }

  bool ImageIO::saveDecompressedImage(const std::string &outputPath,
                                      const std::vector<char> &headerData,
                                      const std::vector<uint8_t> &imageData)
  {
    std::ofstream outputFile(outputPath, std::ios::binary);

    if (!outputFile)
    {
      std::cerr << "出力ファイルを作成できません: " << outputPath << std::endl;
      return false;
    }

    // ヘッダー情報を出力ファイルに書き込む
    outputFile.write(headerData.data(), headerData.size());

    // 画像データを書き込む
    outputFile.write(reinterpret_cast<const char *>(imageData.data()), imageData.size());

    if (!outputFile)
    {
      std::cerr << "画像データの書き込みエラー" << std::endl;
      return false;
    }

    outputFile.close();
    std::cout << "復元画像の保存完了: " << outputPath << std::endl;

    return true;
  }

  // 入力ファイルのサイズを取得（YAMLの場合は参照先PGMファイルのサイズ）
  uintmax_t ImageIO::getInputFileSize(const std::string &filePath)
  {
    std::error_code ec;
    uintmax_t fileSize = 0;

    // 入力ファイルがYAMLの場合、参照先のPGMファイルのサイズを取得
    std::string extension = filePath.substr(filePath.find_last_of(".") + 1);
    if (extension == "yaml" || extension == "yml")
    {
      try
      {
        // YAMLファイルから参照先のPGMファイルパスを取得
        std::string pgmPath = getPgmPathFromYaml(filePath);

        // PGMファイルのサイズを取得
        fileSize = fs::file_size(pgmPath, ec);
        if (ec)
        {
          std::cerr << "PGMファイルのサイズを取得できません: " << pgmPath << std::endl;
          return 0;
        }
        std::cout << "参照先PGMファイル: " << pgmPath << ", サイズ: " << fileSize << " バイト" << std::endl;
      }
      catch (const std::exception &e)
      {
        std::cerr << "YAMLファイルの解析エラー: " << e.what() << std::endl;
        return 0;
      }
    }
    else
    {
      // 通常のファイルの場合は直接そのサイズを取得
      fileSize = fs::file_size(filePath, ec);
      if (ec)
      {
        std::cerr << "入力ファイルのサイズを取得できません: " << filePath << std::endl;
        return 0;
      }
    }

    return fileSize;
  }

  // YAMLファイルから参照先のPGMファイルパスを取得
  std::string ImageIO::getPgmPathFromYaml(const std::string &yamlPath)
  {
    try
    {
      YAML::Node config = YAML::LoadFile(yamlPath);
      std::string pgmFile = config["image"].as<std::string>();
      std::cout << "YAMLファイルから読み込んだPGMファイル: " << pgmFile << std::endl;

      // PGMファイルの絶対パスを構築
      fs::path yaml(yamlPath);
      fs::path pgm = yaml.parent_path() / pgmFile;

      return pgm.string();
    }
    catch (const std::exception &e)
    {
      std::cerr << "YAMLファイルの解析エラー: " << e.what() << std::endl;
      throw; // 例外を再スロー
    }
  }

} // namespace compressor