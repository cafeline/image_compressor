#ifndef IMAGE_HEADER_H
#define IMAGE_HEADER_H

#include <vector>

namespace compressor {

// 画像ヘッダー情報の構造体
struct ImageHeader {
    int width;
    int height;
    int start; // ヘッダー部分の長さ
};

} // namespace compressor

#endif // IMAGE_HEADER_H