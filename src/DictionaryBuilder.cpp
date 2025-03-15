#include "binary_image_compressor/compressor//DictionaryBuilder.h"
#include <iostream>
#include <fstream>
#include <algorithm>

namespace compressor
{

  bool DictionaryBuilder::buildDictionary(const std::string &patternDataPath,
                                          const std::string &dictionaryPath,
                                          int totalBlocks,
                                          int patternBytes,
                                          int patternBits)
  {
    std::cout << "パターン辞書構築中..." << std::endl;

    std::ifstream blockFile(patternDataPath, std::ios::binary);
    std::ofstream dictFile(dictionaryPath, std::ios::binary | std::ios::trunc);

    if (!blockFile || !dictFile)
    {
      std::cerr << "ファイルのオープンに失敗しました" << std::endl;
      return false;
    }

    // パターンカウントの初期値を書き込む（後で更新）
    uint16_t patternCount = 0;
    dictFile.write(reinterpret_cast<char *>(&patternCount), sizeof(uint16_t));

    // パターン辞書の構築
    std::vector<BinaryPattern> patterns;
    std::vector<uint8_t> blockData(patternBytes);

    // すべてのブロックを処理
    for (int i = 0; i < totalBlocks; ++i)
    {
      if (!blockFile.read(reinterpret_cast<char *>(blockData.data()), patternBytes))
      {
        if (blockFile.eof())
          break;
        std::cerr << "ブロックデータの読み込みエラー" << std::endl;
        return false;
      }

      // このパターンが辞書にすでに存在するか確認
      bool found = false;
      for (auto &pattern : patterns)
      {
        if (pattern.equals(blockData))
        {
          pattern.frequency++;
          found = true;
          break;
        }
      }

      // 新しいパターンを追加
      if (!found)
      {
        patterns.emplace_back(blockData, patternBits);
      }
    }

    // パターンを出現頻度で並べ替え（オプション）
    std::sort(patterns.begin(), patterns.end(),
              [](const BinaryPattern &a, const BinaryPattern &b)
              {
                return a.frequency > b.frequency;
              });

    // 辞書ファイルにパターンを書き込む
    for (const auto &pattern : patterns)
    {
      dictFile.write(reinterpret_cast<const char *>(&pattern.bitLength), sizeof(int));
      dictFile.write(reinterpret_cast<const char *>(&pattern.frequency), sizeof(int));
      dictFile.write(reinterpret_cast<const char *>(pattern.pattern.data()), pattern.pattern.size());
    }

    // パターン数を更新
    patternCount = static_cast<uint16_t>(patterns.size());
    dictFile.seekp(0);
    dictFile.write(reinterpret_cast<char *>(&patternCount), sizeof(uint16_t));

    blockFile.close();
    dictFile.close();

    std::cout << "パターン辞書構築完了: " << dictionaryPath << std::endl;
    std::cout << "- パターン数: " << patternCount << std::endl;

    return true;
  }

  bool DictionaryBuilder::loadDictionary(const std::string &dictionaryPath,
                                         std::vector<BinaryPattern> &patterns,
                                         uint16_t &patternCount,
                                         int patternBytes)
  {
    std::ifstream dictFile(dictionaryPath, std::ios::binary);
    if (!dictFile)
    {
      std::cerr << "辞書ファイルを開けません: " << dictionaryPath << std::endl;
      return false;
    }

    // パターン数を読み込む
    dictFile.read(reinterpret_cast<char *>(&patternCount), sizeof(uint16_t));
    patterns.clear();
    patterns.reserve(patternCount);

    // 各パターンを読み込む
    for (int i = 0; i < patternCount; ++i)
    {
      int bitLength, frequency;
      dictFile.read(reinterpret_cast<char *>(&bitLength), sizeof(int));
      dictFile.read(reinterpret_cast<char *>(&frequency), sizeof(int));

      std::vector<uint8_t> patternData(patternBytes);
      dictFile.read(reinterpret_cast<char *>(patternData.data()), patternBytes);

      if (!dictFile)
      {
        std::cerr << "辞書データの読み込みエラー" << std::endl;
        return false;
      }

      patterns.emplace_back(patternData, bitLength, frequency);
    }

    dictFile.close();
    return true;
  }

} // namespace compressor