#include "binary_image_compressor/core/BinaryImageCompressor.h"
#include "binary_image_compressor/io/ImageIO.h"
#include "binary_image_compressor/io/FileSystem.h"
#include <iostream>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
namespace compressor
{

  BinaryImageCompressor::BinaryImageCompressor(
      const std::string &inputFile,
      const std::string &outputFile,
      int blockSize,
      int thresholdValue) : inputPath(inputFile),
                            outputPath(outputFile),
                            blockSize(blockSize),
                            threshold(thresholdValue)
  {
    // コンポーネントの初期化
    blockProcessor = std::make_unique<BlockProcessor>(blockSize, blockSize);
    dictionaryBuilder = std::make_unique<DictionaryBuilder>();
    patternEncoder = std::make_unique<PatternEncoder>();

    // 一時ファイルパスの初期化
    initializeTempPaths();
    ensureTempDirectoryExists();
  }

  BinaryImageCompressor::~BinaryImageCompressor()
  {
    // 必要に応じて一時ファイルを削除
  }

  void BinaryImageCompressor::initializeTempPaths()
  {
    if (!inputPath.empty())
    {
      fs::path input(inputPath);
      std::string baseName = input.stem().string();

      patternDataPath = FileSystem::generateTempFilePath(baseName, "_pattern.bin");
      indexDataPath = FileSystem::generateTempFilePath(baseName, "_index.bin");
      dictionaryPath = FileSystem::generateTempFilePath(baseName, "_dict.bin");
    }
  }

  void BinaryImageCompressor::ensureTempDirectoryExists()
  {
    FileSystem::createDirectory("temp");
  }

  void BinaryImageCompressor::setInputPath(const std::string &path)
  {
    inputPath = path;
    initializeTempPaths();
  }

  void BinaryImageCompressor::setOutputPath(const std::string &path)
  {
    outputPath = path;
  }

  void BinaryImageCompressor::setBlockSize(int size)
  {
    blockSize = size;
    blockProcessor = std::make_unique<BlockProcessor>(size, size);
  }

  void BinaryImageCompressor::setThreshold(int value)
  {
    threshold = value;
  }

  bool BinaryImageCompressor::compress()
  {
    if (inputPath.empty() || outputPath.empty())
    {
      std::cerr << "入力または出力パスが設定されていません" << std::endl;
      return false;
    }

    // ヘッダー解析
    if (!ImageIO::parseHeader(inputPath, header, headerData))
    {
      std::cerr << "ヘッダー解析に失敗しました" << std::endl;
      return false;
    }

    // 一時ファイルパスの確保
    initializeTempPaths();
    ensureTempDirectoryExists();

    // 画像の2値化
    std::string binarizedPath = FileSystem::generateTempFilePath("binarized", ".pgm");
    if (!ImageIO::binarizeImage(inputPath, binarizedPath, header, headerData, threshold))
    {
      std::cerr << "画像の2値化に失敗しました" << std::endl;
      return false;
    }

    // ブロック分割
    if (!blockProcessor->divideIntoBlocks(binarizedPath, patternDataPath, header))
    {
      std::cerr << "ブロック分割に失敗しました" << std::endl;
      return false;
    }

    // ブロック数の計算
    int rowBlocks, colBlocks, totalBlocks;
    blockProcessor->calculateBlockCount(header, rowBlocks, colBlocks, totalBlocks);

    // パターンサイズの計算
    int patternBits, patternBytes;
    blockProcessor->calculatePatternSize(patternBits, patternBytes);

    // パターン辞書の構築
    if (!dictionaryBuilder->buildDictionary(patternDataPath, dictionaryPath, totalBlocks, patternBytes, patternBits))
    {
      std::cerr << "辞書構築に失敗しました" << std::endl;
      return false;
    }

    // パターンのエンコード
    if (!patternEncoder->encodePatterns(patternDataPath, dictionaryPath, indexDataPath, totalBlocks, patternBytes))
    {
      std::cerr << "パターンエンコードに失敗しました" << std::endl;
      return false;
    }

    // 圧縮情報の表示
    printCompressionInfo();

    return true;
  }

  bool BinaryImageCompressor::decompress()
  {
    std::cout << "画像の復元を開始します..." << std::endl;

    if (inputPath.empty() || outputPath.empty() ||
        dictionaryPath.empty() || indexDataPath.empty())
    {
      std::cerr << "必要なファイルパスが設定されていません" << std::endl;
      return false;
    }

    // ブロック数の計算
    int rowBlocks, colBlocks, totalBlocks;
    blockProcessor->calculateBlockCount(header, rowBlocks, colBlocks, totalBlocks);

    // パターンサイズの計算
    int patternBits, patternBytes;
    blockProcessor->calculatePatternSize(patternBits, patternBytes);

    // インデックスとパターンの復元
    std::vector<uint16_t> indices;
    std::vector<std::vector<uint8_t>> patterns;

    if (!patternEncoder->decodePatterns(indexDataPath, dictionaryPath, indices, patterns, totalBlocks, patternBytes))
    {
      std::cerr << "パターンのデコードに失敗しました" << std::endl;
      return false;
    }

    // 画像の再構築
    std::vector<uint8_t> imageData(header.width * header.height);

    if (!blockProcessor->reconstructImage(indices, patterns, header, imageData))
    {
      std::cerr << "画像の再構築に失敗しました" << std::endl;
      return false;
    }

    // 画像の保存
    if (!ImageIO::saveDecompressedImage(outputPath, headerData, imageData))
    {
      std::cerr << "画像の保存に失敗しました" << std::endl;
      return false;
    }

    std::cout << "画像の復元が完了しました: " << outputPath << std::endl;
    return true;
  }

  float BinaryImageCompressor::calculateCompressionRatio() const
  {
    if (inputPath.empty() || dictionaryPath.empty() || indexDataPath.empty())
    {
      return -1.0f;
    }

    std::error_code ec;

    // 入力ファイルのサイズを取得（YAMLの場合は参照先PGMファイルのサイズ）
    uintmax_t rawSize = FileSystem::getInputFileSize(inputPath);
    if (rawSize == 0)
    {
      return -1.0f;
    }

    uintmax_t dictSize = FileSystem::getFileSize(dictionaryPath);
    if (dictSize == 0)
      return -1.0f;

    uintmax_t indexSize = FileSystem::getFileSize(indexDataPath);
    if (indexSize == 0)
      return -1.0f;

    // 元のサイズに対する圧縮後のサイズの比率を計算
    float compressedSize = static_cast<float>(dictSize + indexSize);
    float originalSize = static_cast<float>(rawSize);
    float compressionRatio = 100.0f * (1.0f - compressedSize / originalSize);

    std::cout << "dictSize: " << dictSize << std::endl;
    std::cout << "indexSize: " << indexSize << std::endl;
    std::cout << "compressedSize: " << compressedSize << std::endl;
    std::cout << "originalSize: " << originalSize << std::endl;
    std::cout << "compressionRatio: " << compressionRatio << std::endl;

    return compressionRatio;
  }

  void BinaryImageCompressor::printCompressionInfo() const
  {
    if (inputPath.empty() || dictionaryPath.empty() || indexDataPath.empty())
    {
      return;
    }

    std::cout << "圧縮情報:" << std::endl;

    std::error_code ec;

    // 入力ファイルのサイズを取得（YAMLの場合は参照先PGMファイルのサイズ）
    uintmax_t rawSize = FileSystem::getInputFileSize(inputPath);
    if (rawSize == 0)
    {
      std::cerr << "入力ファイルのサイズを取得できません" << std::endl;
      return;
    }

    uintmax_t dictSize = FileSystem::getFileSize(dictionaryPath);
    if (dictSize == 0)
    {
      std::cerr << "辞書ファイルのサイズを取得できません" << std::endl;
      return;
    }

    uintmax_t indexSize = FileSystem::getFileSize(indexDataPath);
    if (indexSize == 0)
    {
      std::cerr << "インデックスファイルのサイズを取得できません" << std::endl;
      return;
    }

    float compressionRatio = calculateCompressionRatio();

    std::cout << "- 元画像サイズ: " << rawSize << " バイト" << std::endl;
    std::cout << "- 辞書サイズ: " << dictSize << " バイト" << std::endl;
    std::cout << "- インデックスサイズ: " << indexSize << " バイト" << std::endl;
    std::cout << "- 合計圧縮サイズ: " << (dictSize + indexSize) << " バイト" << std::endl;
    std::cout << "- 圧縮率: " << compressionRatio << "%" << std::endl;
    std::cout << "- ブロックサイズ: " << blockSize << "x" << blockSize << std::endl;
  }

  BlockSizeResult BinaryImageCompressor::findOptimalBlockSize(int minSize, int maxSize, int step)
  {
    if (inputPath.empty())
    {
      std::cerr << "入力パスが設定されていません" << std::endl;
      return BlockSizeResult();
    }

    std::cout << "最適なブロックサイズを探索中..." << std::endl;

    // 元のブロックサイズを保存
    int originalBlockSize = blockSize;
    BlockSizeResult bestResult(0, -100.0f);

    // 各ブロックサイズをテスト
    float bestRatio = 0.0f;
    for (int size = minSize; size <= maxSize; size += step)
    {
      std::cout << "ブロックサイズ " << size << "x" << size << " をテスト中..." << std::endl;

      // ブロックサイズを設定
      setBlockSize(size);

      // 圧縮を実行
      if (!compress())
      {
        std::cerr << "ブロックサイズ " << size << " での圧縮に失敗しました" << std::endl;
        continue;
      }

      // 圧縮率を計算
      float ratio = calculateCompressionRatio();
      std::cout << "  圧縮率: " << ratio << "%" << std::endl;

      // 最良の圧縮率を保存
      if (ratio > bestRatio)
      {
        std::cout << "  新しい最良の圧縮率を発見: " << ratio << "%" << std::endl;
        bestRatio = ratio;
        bestResult.blockSize = size;
        bestResult.compressionRatio = ratio;
      }
    }

    // 元のブロックサイズに戻す
    setBlockSize(originalBlockSize);

    std::cout << "最適なブロックサイズが見つかりました: " << bestResult.blockSize << "x"
              << bestResult.blockSize << " (圧縮率: " << bestResult.compressionRatio << "%)" << std::endl;

    return bestResult;
  }

} // namespace compressor