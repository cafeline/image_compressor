#include "binary_image_compressor/compressor//BinaryImageCompressor.h"
#include <iostream>
#include <filesystem>

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

      patternDataPath = "temp/" + baseName + "_pattern.bin";
      indexDataPath = "temp/" + baseName + "_index.bin";
      dictionaryPath = "temp/" + baseName + "_dict.bin";
    }
  }

  void BinaryImageCompressor::ensureTempDirectoryExists()
  {
    if (!fs::exists("temp"))
    {
      fs::create_directory("temp");
    }
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
    std::cout << "デバッグ: 圧縮処理開始" << std::endl;
    if (inputPath.empty() || outputPath.empty())
    {
      std::cerr << "入力または出力パスが設定されていません" << std::endl;
      return false;
    }

    std::cout << "デバッグ: ヘッダー解析開始" << std::endl;
    // ヘッダー解析
    if (!ImageIO::parseHeader(inputPath, header, headerData))
    {
      std::cerr << "ヘッダー解析に失敗しました" << std::endl;
      return false;
    }
    std::cout << "デバッグ: ヘッダー解析完了" << std::endl;

    // 一時ファイルパスの確保
    initializeTempPaths();
    ensureTempDirectoryExists();

    std::cout << "デバッグ: 画像の2値化開始" << std::endl;
    // 画像の2値化
    std::string binarizedPath = "temp/binarized.pgm";
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

    if (!patternEncoder->decodePatterns(indexDataPath, dictionaryPath,
                                        indices, patterns, totalBlocks, patternBytes))
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

  BlockSizeResult BinaryImageCompressor::findOptimalBlockSize()
  {
    std::cout << "最適なブロックサイズを検索中..." << std::endl;

    // 元のブロックサイズを保存
    int originalBlockSize = blockSize;

    // テスト用の一時ファイルパスを保存
    std::string origPatternPath = patternDataPath;
    std::string origIndexPath = indexDataPath;
    std::string origDictPath = dictionaryPath;

    // 最適なブロックサイズを見つけるための変数
    BlockSizeResult bestResult(8, 0.0);
    float bestRatio = 0.0f;

    // 様々なブロックサイズでテスト
    for (int size = 4; size <= 16; size += 2)
    {
      std::cout << "ブロックサイズ " << size << "x" << size << " でテスト中..." << std::endl;

      // ブロックサイズの更新
      setBlockSize(size);

      // テスト用一時ファイルパス
      std::string tempPatternPath = "temp/test_pattern_" + std::to_string(size) + ".bin";
      std::string tempIndexPath = "temp/test_index_" + std::to_string(size) + ".bin";
      std::string tempDictPath = "temp/test_dict_" + std::to_string(size) + ".bin";

      patternDataPath = tempPatternPath;
      indexDataPath = tempIndexPath;
      dictionaryPath = tempDictPath;

      // このブロックサイズで圧縮テスト
      bool success = compress();

      if (success)
      {
        // 圧縮率の計算
        float ratio = calculateCompressionRatio();
        std::cout << "  圧縮率: " << ratio << "%" << std::endl;

        // より良い圧縮率が見つかった場合、結果を更新
        if (ratio > bestRatio)
        {
          bestRatio = ratio;
          bestResult.blockSize = size;
          bestResult.compressionRatio = ratio;
        }
      }

      // 元のパスを復元
      patternDataPath = origPatternPath;
      indexDataPath = origIndexPath;
      dictionaryPath = origDictPath;

      // 一時ファイルの削除
      std::remove(tempPatternPath.c_str());
      std::remove(tempIndexPath.c_str());
      std::remove(tempDictPath.c_str());
    }

    // 元のブロックサイズを復元
    setBlockSize(originalBlockSize);

    std::cout << "最適なブロックサイズが見つかりました: " << bestResult.blockSize << "x"
              << bestResult.blockSize << " (圧縮率: " << bestResult.compressionRatio << "%)" << std::endl;

    return bestResult;
  }

  float BinaryImageCompressor::calculateCompressionRatio() const
  {
    if (inputPath.empty() || dictionaryPath.empty() || indexDataPath.empty())
    {
      return -1.0f;
    }

    // ファイルサイズの取得
    std::error_code ec;
    uintmax_t rawSize = fs::file_size(inputPath, ec);
    if (ec)
      return -1.0f;

    uintmax_t dictSize = fs::file_size(dictionaryPath, ec);
    if (ec)
      return -1.0f;

    uintmax_t indexSize = fs::file_size(indexDataPath, ec);
    if (ec)
      return -1.0f;

    // 元のサイズに対する圧縮後のサイズの比率を計算
    float compressedSize = static_cast<float>(dictSize + indexSize);
    float originalSize = static_cast<float>(rawSize);
    float compressionRatio = 100.0f * (1.0f - compressedSize / originalSize);

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
    uintmax_t rawSize = fs::file_size(inputPath, ec);
    if (ec)
    {
      std::cerr << "入力ファイルのサイズを取得できません" << std::endl;
      return;
    }

    uintmax_t dictSize = fs::file_size(dictionaryPath, ec);
    if (ec)
    {
      std::cerr << "辞書ファイルのサイズを取得できません" << std::endl;
      return;
    }

    uintmax_t indexSize = fs::file_size(indexDataPath, ec);
    if (ec)
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

} // namespace compressor