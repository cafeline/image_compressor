#ifndef IMAGE_IO_H
#define IMAGE_IO_H

#include <string>
#include <vector>
#include "binary_image_compressor/model/ImageHeader.h"

namespace compressor
{

  // 画像入出力を扱うクラス
  class ImageIO
  {
  public:
    // PGMファイルのヘッダー解析
    static bool parseHeader(const std::string &filePath,
                            ImageHeader &header,
                            std::vector<char> &headerData);

    // 画像の2値化処理
    static bool binarizeImage(const std::string &inputPath,
                              const std::string &outputPath,
                              const ImageHeader &header,
                              const std::vector<char> &headerData,
                              int threshold);

    // 復元画像の保存
    static bool saveDecompressedImage(const std::string &outputPath,
                                      const std::vector<char> &headerData,
                                      const std::vector<uint8_t> &imageData);
  };

} // namespace compressor

#endif // IMAGE_IO_H