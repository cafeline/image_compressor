#ifndef BINARY_PATTERN_H
#define BINARY_PATTERN_H

#include <vector>
#include <cstdint>

namespace compressor {

// 2値画像圧縮のためのパターン辞書エントリ
class BinaryPattern {
public:
    std::vector<uint8_t> pattern; // パターンデータ（ビット配列）
    int bitLength;                // パターンのビット長
    int frequency;                // 出現頻度

    BinaryPattern(int size = 64);
    BinaryPattern(const std::vector<uint8_t> &data, int length, int freq = 1);

    // パターン比較メソッド
    bool equals(const std::vector<uint8_t>& other) const;
};

} // namespace compressor

#endif // BINARY_PATTERN_H