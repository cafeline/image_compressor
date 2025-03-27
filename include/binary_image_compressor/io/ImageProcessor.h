#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include "binary_image_compressor/model/ImageHeader.h"
#include <vector>
#include <string>
#include <cstdint>

namespace compressor
{
  /**
   * @brief 画像処理を担当するクラス
   *
   * 画像の2値化や統計処理など、画像データに対する処理を行います。
   * ImageIOクラスから処理ロジックを分離しました。
   */
  class ImageProcessor
  {
  public:
    /**
     * @brief 画像を2値化処理する
     * @param inputPath 入力画像パス
     * @param outputPath 出力画像パス
     * @param header 画像ヘッダー情報
     * @param headerData ヘッダーデータ
     * @param threshold 2値化閾値
     * @return 処理成功の場合はtrue
     */
    static bool binarizeImage(const std::string &inputPath,
                              const std::string &outputPath,
                              const ImageHeader &header,
                              const std::vector<char> &headerData,
                              int threshold = 128);

    /**
     * @brief 画像データの統計情報を計算する
     * @param imageData 画像データ
     * @param minVal 出力・最小輝度値
     * @param maxVal 出力・最大輝度値
     * @param avgVal 出力・平均輝度値
     */
    static void calculateImageStats(const std::vector<uint8_t> &imageData,
                                    int &minVal,
                                    int &maxVal,
                                    double &avgVal);

    /**
     * @brief 2値化された画像の黒と白のピクセル数を計算する
     * @param imageData 画像データ
     * @param zeroCount 出力・黒（0）のピクセル数
     * @param fullCount 出力・白（255）のピクセル数
     */
    static void countBinarizedPixels(const std::vector<uint8_t> &imageData,
                                     int &zeroCount,
                                     int &fullCount);

    /**
     * @brief 閾値を自動調整する
     * @param imageData 画像データ
     * @param currentThreshold 現在の閾値
     * @param minVal 最小輝度値
     * @param maxVal 最大輝度値
     * @param avgVal 平均輝度値
     * @return 調整された閾値
     */
    static int adjustThreshold(std::vector<uint8_t> &imageData,
                               int currentThreshold,
                               int minVal,
                               int maxVal,
                               double avgVal);
  };

} // namespace compressor

#endif // IMAGE_PROCESSOR_H