#include "binary_image_compressor/model/BlockSizeResult.h"

namespace compressor
{

  BlockSizeResult::BlockSizeResult(int size, float ratio) : blockSize(size),
                                                            compressionRatio(ratio)
  {
  }

} // namespace compressor