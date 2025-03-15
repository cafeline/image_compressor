#ifndef BLOCK_PROCESSOR_H
#define BLOCK_PROCESSOR_H

#include <string>
#include <vector>
#include "binary_image_compressor/model/ImageHeader.h"

namespace compressor
{

  // ブロック処理を担当するクラス
  class BlockProcessor
  {
  public:
    BlockProcessor(int blockRowSize, int blockColumnSize);

    // ブロック分割処理
    bool divideIntoBlocks(const std::string &binarizedPath,
                          const std::string &patternDataPath,
                          const ImageHeader &header);

    // ブロック数の計算
    void calculateBlockCount(const ImageHeader &header,
                             int &rowBlocks,
                             int &colBlocks,
                             int &totalBlocks) const;

    // パターンサイズの計算
    void calculatePatternSize(int &patternBits, int &patternBytes) const;

    // 復元画像の生成
    bool reconstructImage(const std::vector<uint16_t> &indices,
                          const std::vector<std::vector<uint8_t>> &patterns,
                          const ImageHeader &header,
                          std::vector<uint8_t> &imageData) const;

  private:
    int blockRowSize;
    int blockColumnSize;
  };

} // namespace compressor

#endif // BLOCK_PROCESSOR_H