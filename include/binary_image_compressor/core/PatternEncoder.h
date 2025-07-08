#ifndef PATTERN_ENCODER_H
#define PATTERN_ENCODER_H

#include <string>
#include <vector>
#include "binary_image_compressor/model/BinaryPattern.h"

namespace compressor
{

  // パターンエンコードクラス
  class PatternEncoder
  {
  public:
    // ブロックパターンのエンコード (16bit indices)
    bool encodePatterns(const std::string &patternDataPath,
                        const std::string &dictionaryPath,
                        const std::string &indexDataPath,
                        int totalBlocks,
                        int patternBytes);

    // ブロックパターンのエンコード (8bit indices)
    bool encodePatterns8bit(const std::string &patternDataPath,
                           const std::string &dictionaryPath,
                           const std::string &indexDataPath,
                           int totalBlocks,
                           int patternBytes);

    // ブロックパターンのエンコード (汎用)
    bool encodePatterns(const std::string &patternDataPath,
                        const std::string &dictionaryPath,
                        const std::string &indexDataPath,
                        int totalBlocks,
                        int patternBytes,
                        bool use8bit);

    // インデックスからの復元 (16bit indices)
    bool decodePatterns(const std::string &indexDataPath,
                        const std::string &dictionaryPath,
                        std::vector<uint16_t> &indices,
                        std::vector<std::vector<uint8_t>> &patterns,
                        int totalBlocks,
                        int patternBytes);

    // インデックスからの復元 (8bit indices)
    bool decodePatterns8bit(const std::string &indexDataPath,
                           const std::string &dictionaryPath,
                           std::vector<uint8_t> &indices,
                           std::vector<std::vector<uint8_t>> &patterns,
                           int totalBlocks,
                           int patternBytes);

    // インデックスからの復元 (汎用)
    template<typename IndexType>
    bool decodePatternsGeneric(const std::string &indexDataPath,
                              const std::string &dictionaryPath,
                              std::vector<IndexType> &indices,
                              std::vector<std::vector<uint8_t>> &patterns,
                              int totalBlocks,
                              int patternBytes);
  };

} // namespace compressor

#endif // PATTERN_ENCODER_H