#ifndef BLOCK_SIZE_RESULT_H
#define BLOCK_SIZE_RESULT_H

namespace compressor {

// 最適なブロックサイズの結果
struct BlockSizeResult {
    int blockSize;
    float compressionRatio;

    BlockSizeResult(int size = 8, float ratio = 0.0);
};

} // namespace compressor

#endif // BLOCK_SIZE_RESULT_H