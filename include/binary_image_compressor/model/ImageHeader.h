#ifndef IMAGE_HEADER_H
#define IMAGE_HEADER_H

#include <cstddef>
#include <string>

namespace compressor
{

  // 画像ヘッダー情報の構造体
  struct ImageHeader
  {
    int width;
    int height;
    size_t start;     // ヘッダー部分の長さ
    bool isPGMBinary; // P5形式（バイナリ）ならtrue、P2形式（ASCII）ならfalse

    ImageHeader() : width(0), height(0), start(0), isPGMBinary(true) {}
  };

} // namespace compressor

#endif // IMAGE_HEADER_H