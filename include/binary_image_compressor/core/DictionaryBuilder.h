#ifndef DICTIONARY_BUILDER_H
#define DICTIONARY_BUILDER_H

#include <string>
#include <vector>
#include "binary_image_compressor/model/BinaryPattern.h"

namespace compressor
{

  // パターン辞書構築クラス
  class DictionaryBuilder
  {
  public:
    // パターン辞書の構築
    bool buildDictionary(const std::string &patternDataPath,
                         const std::string &dictionaryPath,
                         int totalBlocks,
                         int patternBytes,
                         int patternBits);

    // 辞書からパターンを読み込む
    bool loadDictionary(const std::string &dictionaryPath,
                        std::vector<BinaryPattern> &patterns,
                        uint16_t &patternCount,
                        int patternBytes);
  };

} // namespace compressor

#endif // DICTIONARY_BUILDER_H