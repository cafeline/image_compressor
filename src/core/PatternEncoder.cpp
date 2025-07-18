#include "binary_image_compressor/core/PatternEncoder.h"
#include <iostream>
#include <fstream>

namespace compressor
{

  bool PatternEncoder::encodePatterns(const std::string &patternDataPath,
                                      const std::string &dictionaryPath,
                                      const std::string &indexDataPath,
                                      int totalBlocks,
                                      int patternBytes)
  {
    return encodePatterns(patternDataPath, dictionaryPath, indexDataPath, totalBlocks, patternBytes, false);
  }

  bool PatternEncoder::encodePatterns8bit(const std::string &patternDataPath,
                                         const std::string &dictionaryPath,
                                         const std::string &indexDataPath,
                                         int totalBlocks,
                                         int patternBytes)
  {
    return encodePatterns(patternDataPath, dictionaryPath, indexDataPath, totalBlocks, patternBytes, true);
  }

  bool PatternEncoder::encodePatterns(const std::string &patternDataPath,
                                      const std::string &dictionaryPath,
                                      const std::string &indexDataPath,
                                      int totalBlocks,
                                      int patternBytes,
                                      bool use8bit)
  {
    std::cout << "ブロックパターンのエンコード中 (" << (use8bit ? "8bit" : "16bit") << " インデックス)..." << std::endl;

    std::ifstream blockFile(patternDataPath, std::ios::binary);
    std::ifstream dictFile(dictionaryPath, std::ios::binary);
    std::ofstream indexFile(indexDataPath, std::ios::binary | std::ios::trunc);

    if (!blockFile || !dictFile || !indexFile)
    {
      std::cerr << "ファイルのオープンに失敗しました" << std::endl;
      return false;
    }

    // パターン数を読み込む
    uint16_t patternCount;
    dictFile.read(reinterpret_cast<char *>(&patternCount), sizeof(uint16_t));

    // 8bitインデックスの場合、パターン数をチェック
    if (use8bit && patternCount > 255)
    {
      std::cerr << "8bitインデックスを使用するには、パターン数が255以下である必要があります。現在のパターン数: " << patternCount << std::endl;
      return false;
    }

    // すべてのパターンを読み込む
    std::vector<BinaryPattern> patterns(patternCount);
    for (int i = 0; i < patternCount; ++i)
    {
      patterns[i].pattern.resize(patternBytes);
      dictFile.read(reinterpret_cast<char *>(patterns[i].pattern.data()), patternBytes);
    }

    // 各ブロックを処理
    std::vector<uint8_t> blockData(patternBytes);
    for (int i = 0; i < totalBlocks; ++i)
    {
      if (!blockFile.read(reinterpret_cast<char *>(blockData.data()), patternBytes))
      {
        if (blockFile.eof())
          break;
        std::cerr << "ブロックデータの読み込みエラー" << std::endl;
        return false;
      }

      // パターン辞書内でマッチするパターンを検索
      int matchIndex = -1;
      for (int j = 0; j < patternCount; ++j)
      {
        if (patterns[j].equals(blockData))
        {
          matchIndex = j;
          break;
        }
      }

      if (matchIndex == -1)
      {
        std::cerr << "マッチするパターンが見つかりません" << std::endl;
        return false;
      }

      // インデックスをファイルに書き込む
      if (use8bit)
      {
        uint8_t index = static_cast<uint8_t>(matchIndex);
        indexFile.write(reinterpret_cast<char *>(&index), sizeof(uint8_t));
      }
      else
      {
        uint16_t index = static_cast<uint16_t>(matchIndex);
        indexFile.write(reinterpret_cast<char *>(&index), sizeof(uint16_t));
      }
    }

    blockFile.close();
    dictFile.close();
    indexFile.close();

    std::cout << "ブロックパターンのエンコード完了: " << indexDataPath << std::endl;
    return true;
  }

  bool PatternEncoder::decodePatterns8bit(const std::string &indexDataPath,
                                         const std::string &dictionaryPath,
                                         std::vector<uint8_t> &indices,
                                         std::vector<std::vector<uint8_t>> &patterns,
                                         int totalBlocks,
                                         int patternBytes)
  {
    return decodePatternsGeneric(indexDataPath, dictionaryPath, indices, patterns, totalBlocks, patternBytes);
  }

  template<typename IndexType>
  bool PatternEncoder::decodePatternsGeneric(const std::string &indexDataPath,
                                            const std::string &dictionaryPath,
                                            std::vector<IndexType> &indices,
                                            std::vector<std::vector<uint8_t>> &patterns,
                                            int totalBlocks,
                                            int patternBytes)
  {
    std::ifstream indexFile(indexDataPath, std::ios::binary);
    std::ifstream dictFile(dictionaryPath, std::ios::binary);

    if (!indexFile || !dictFile)
    {
      std::cerr << "ファイルのオープンに失敗しました" << std::endl;
      return false;
    }

    // パターン数を読み込む
    uint16_t patternCount;
    dictFile.read(reinterpret_cast<char *>(&patternCount), sizeof(uint16_t));

    // パターンデータを読み込む
    patterns.resize(patternCount);
    for (uint16_t i = 0; i < patternCount; ++i)
    {
      patterns[i].resize(patternBytes);
      dictFile.read(reinterpret_cast<char *>(patterns[i].data()), patternBytes);

      if (!dictFile)
      {
        std::cerr << "パターンデータの読み込みエラー" << std::endl;
        return false;
      }
    }

    // インデックスを読み込む
    indices.resize(totalBlocks);
    for (int i = 0; i < totalBlocks; ++i)
    {
      if (!indexFile.read(reinterpret_cast<char *>(&indices[i]), sizeof(IndexType)))
      {
        if (indexFile.eof() && i == totalBlocks - 1)
        {
          break; // 最後のブロックまで読み込んだ場合
        }
        std::cerr << "インデックスの読み込みエラー" << std::endl;
        return false;
      }

      if (indices[i] >= patternCount)
      {
        std::cerr << "インデックスが範囲外です: " << static_cast<int>(indices[i]) << std::endl;
        return false;
      }
    }

    indexFile.close();
    dictFile.close();
    return true;
  }

  // 既存の16bit版メソッド (修正)
  bool PatternEncoder::decodePatterns(const std::string &indexDataPath,
                                      const std::string &dictionaryPath,
                                      std::vector<uint16_t> &indices,
                                      std::vector<std::vector<uint8_t>> &patterns,
                                      int totalBlocks,
                                      int patternBytes)
  {
    return decodePatternsGeneric(indexDataPath, dictionaryPath, indices, patterns, totalBlocks, patternBytes);
  }

  // テンプレートの明示的インスタンス化
  template bool PatternEncoder::decodePatternsGeneric<uint8_t>(const std::string &indexDataPath,
                                                              const std::string &dictionaryPath,
                                                              std::vector<uint8_t> &indices,
                                                              std::vector<std::vector<uint8_t>> &patterns,
                                                              int totalBlocks,
                                                              int patternBytes);

  template bool PatternEncoder::decodePatternsGeneric<uint16_t>(const std::string &indexDataPath,
                                                               const std::string &dictionaryPath,
                                                               std::vector<uint16_t> &indices,
                                                               std::vector<std::vector<uint8_t>> &patterns,
                                                               int totalBlocks,
                                                               int patternBytes);

} // namespace compressor