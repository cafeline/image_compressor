#include "binary_image_compressor/io/ImageProcessor.h"
#include "binary_image_compressor/io/ImageReader.h"
#include <iostream>
#include <fstream>
#include <algorithm>

namespace compressor
{
  bool ImageProcessor::binarizeImage(const std::string &inputPath,
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

    // 画像データの統計情報を計算
    int minVal, maxVal;
    double avgVal;
    calculateImageStats(imageData, minVal, maxVal, avgVal);
    std::cout << "画像統計 - 最小値: " << minVal << ", 最大値: " << maxVal << ", 平均値: " << avgVal << std::endl;

    // 2値化処理
    for (auto &pixel : imageData)
    {
      pixel = (pixel >= threshold) ? 255 : 0;
    }

    // 2値化後の統計情報を計算
    int zeroCount, fullCount;
    countBinarizedPixels(imageData, zeroCount, fullCount);
    std::cout << "2値化後 - 0の数: " << zeroCount << ", 255の数: " << fullCount << std::endl;

    // 閾値が適切かどうかを調査
    if (zeroCount == 0 || fullCount == 0)
    {
      std::cout << "警告: 2値化後の画像が単色になっています。閾値の調整が必要かもしれません。" << std::endl;
      std::cout << "現在の閾値: " << threshold << ", 推奨閾値: " << static_cast<int>(avgVal) << std::endl;

      // 閾値を自動調整
      int newThreshold = adjustThreshold(imageData, threshold, minVal, maxVal, avgVal);

      // 再度統計情報を計算
      countBinarizedPixels(imageData, zeroCount, fullCount);
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

  void ImageProcessor::calculateImageStats(const std::vector<uint8_t> &imageData,
                                           int &minVal,
                                           int &maxVal,
                                           double &avgVal)
  {
    minVal = 255;
    maxVal = 0;
    long long sum = 0;

    for (const auto &pixel : imageData)
    {
      minVal = std::min(minVal, static_cast<int>(pixel));
      maxVal = std::max(maxVal, static_cast<int>(pixel));
      sum += pixel;
    }

    avgVal = static_cast<double>(sum) / imageData.size();
  }

  void ImageProcessor::countBinarizedPixels(const std::vector<uint8_t> &imageData,
                                            int &zeroCount,
                                            int &fullCount)
  {
    zeroCount = 0;
    fullCount = 0;

    for (const auto &pixel : imageData)
    {
      if (pixel == 0)
        zeroCount++;
      if (pixel == 255)
        fullCount++;
    }
  }

  int ImageProcessor::adjustThreshold(std::vector<uint8_t> &imageData,
                                      int currentThreshold,
                                      int minVal,
                                      int maxVal,
                                      double avgVal)
  {
    // 平均値を新しい閾値として使用
    int newThreshold = static_cast<int>(avgVal);

    // 平均値が現在の閾値と同じ場合は、最小値と最大値の中間を使用
    if (newThreshold == currentThreshold)
    {
      newThreshold = (minVal + maxVal) / 2;
    }

    std::cout << "閾値を自動調整: " << currentThreshold << " -> " << newThreshold << std::endl;

    // 新しい閾値で2値化をやり直し
    for (auto &pixel : imageData)
    {
      pixel = (pixel >= newThreshold) ? 255 : 0;
    }

    return newThreshold;
  }

} // namespace compressor