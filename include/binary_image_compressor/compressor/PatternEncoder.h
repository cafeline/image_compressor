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
    // ブロックパターンのエンコード
    bool encodePatterns(const std::string &patternDataPath,
                        const std::string &dictionaryPath,
                        const std::string &indexDataPath,
                        int totalBlocks,
                        int patternBytes);

    // インデックスからの復元
    bool decodePatterns(const std::string &indexDataPath,
                        const std::string &dictionaryPath,
                        std::vector<uint16_t> &indices,
                        std::vector<std::vector<uint8_t>> &patterns,
                        int totalBlocks,
                        int patternBytes);
  };

} // namespace compressor

#endif // PATTERN_ENCODER_H