#ifndef BINARY_IMAGE_COMPRESSOR_H
#define BINARY_IMAGE_COMPRESSOR_H

#include <string>
#include <vector>
#include <memory>
#include "binary_image_compressor/model/ImageHeader.h"
#include "binary_image_compressor/model/BlockSizeResult.h"
#include "binary_image_compressor/io/ImageIO.h"
#include "binary_image_compressor/core/BlockProcessor.h"
#include "binary_image_compressor/core/DictionaryBuilder.h"
#include "binary_image_compressor/core/PatternEncoder.h"

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
    // ムーブコンストラクタとムーブ代入演算子の追加
    BinaryImageCompressor(BinaryImageCompressor &&) noexcept = default;
    BinaryImageCompressor &operator=(BinaryImageCompressor &&) noexcept = default;

    // コピー操作を禁止（リソース管理の観点から）
    BinaryImageCompressor(const BinaryImageCompressor &) = delete;
    BinaryImageCompressor &operator=(const BinaryImageCompressor &) = delete;

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
    BlockSizeResult findOptimalBlockSize(int minSize = 4, int maxSize = 16, int step = 2);

    // ユーティリティ
    float calculateCompressionRatio() const;
    void printCompressionInfo() const;
  };

} // namespace compressor

#endif // BINARY_IMAGE_COMPRESSOR_H