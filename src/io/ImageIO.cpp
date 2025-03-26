#include "binary_image_compressor/io/ImageIO.h"
#include "binary_image_compressor/io/FileSystem.h"
#include "binary_image_compressor/io/ImageReader.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <sstream>

namespace compressor
{

  bool ImageIO::parseHeader(const std::string &filePath,
                            ImageHeader &header,
                            std::vector<char> &headerData)
  {
    // ファイル拡張子の確認
    std::string extension = FileSystem::getFileExtension(filePath);
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
      std::string pgmPath = FileSystem::getPgmPathFromYaml(filePath);
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
    auto reader = createImageReader(filePath);
    if (!reader)
    {
      std::cerr << "対応するリーダーを作成できません: " << filePath << std::endl;
      return false;
    }

    return reader->readHeader(header, headerData);
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

    // 画像データの読み込み
    std::vector<uint8_t> imageData;
    auto reader = createImageReader(inputPath);
    if (!reader || !reader->readImageData(imageData))
    {
      std::cerr << "画像データの読み込みに失敗しました" << std::endl;
      return false;
    }

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
    auto reader = createImageReader(filePath);
    if (!reader)
    {
      std::cerr << "対応するリーダーを作成できません: " << filePath << std::endl;
      return false;
    }

    return reader->readImageData(imageData);
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

} // namespace compressor