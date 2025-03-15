#ifndef BINARY_IMAGE_COMPRESSOR_H
#define BINARY_IMAGE_COMPRESSOR_H

#include <string>
#include <vector>
#include <memory>
#include "binary_image_compressor/model/ImageHeader.h"
#include "binary_image_compressor/model/BlockSizeResult.h"
#include "ImageIO.h"
#include "BlockProcessor.h"
#include "DictionaryBuilder.h"
#include "PatternEncoder.h"

namespace compressor
{

  // メインの圧縮クラス（コーディネーター）
  class BinaryImageCompressor
  {
  private:
    // 設定情報
    std::string inputPath;
    std::string patternDataPath;
    std::string indexDataPath;
    std::string dictionaryPath;
    std::string outputPath;

    // パラメータ
    int blockSize;
    int threshold;

    // 画像情報
    ImageHeader header;
    std::vector<char> headerData;

    // コンポーネント
    std::unique_ptr<BlockProcessor> blockProcessor;
    std::unique_ptr<DictionaryBuilder> dictionaryBuilder;
    std::unique_ptr<PatternEncoder> patternEncoder;

    // ユーティリティメソッド
    void initializeTempPaths();
    void ensureTempDirectoryExists();

  public:
    // コンストラクタ
    BinaryImageCompressor(
        const std::string &inputFile = "",
        const std::string &outputFile = "",
        int blockSize = 8,
        int thresholdValue = 128);

    // デストラクタ
    ~BinaryImageCompressor();

    // セッター
    void setInputPath(const std::string &path);
    void setOutputPath(const std::string &path);
    void setBlockSize(int size);
    void setThreshold(int value);

    // メイン処理メソッド
    bool compress();
    bool decompress();

    // 最適なブロックサイズを見つける
    BlockSizeResult findOptimalBlockSize();

    // ユーティリティ
    float calculateCompressionRatio() const;
    void printCompressionInfo() const;
  };

} // namespace compressor

#endif // BINARY_IMAGE_COMPRESSOR_H