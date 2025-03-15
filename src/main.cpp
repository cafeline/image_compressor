#include "binary_image_compressor/compressor//BinaryImageCompressor.h"
#include <iostream>

int main(int argc, char *argv[])
{
  // コマンドライン引数の解析
  if (argc < 3)
  {
    std::cout << "使用法: " << argv[0] << " <入力ファイル> <出力ファイル> [ブロックサイズ] [閾値]" << std::endl;
    return 1;
  }

  std::string inputFile = argv[1];
  std::string outputFile = argv[2];

  int blockSize = 8;
  if (argc > 3)
  {
    blockSize = std::stoi(argv[3]);
  }

  int threshold = 128;
  if (argc > 4)
  {
    threshold = std::stoi(argv[4]);
  }

  // 圧縮器のインスタンス化（名前空間を使用）
  compressor::BinaryImageCompressor compressor(inputFile, outputFile, blockSize, threshold);

  // 自動ブロックサイズの検出（オプション）
  if (blockSize <= 0)
  {
    compressor::BlockSizeResult result = compressor.findOptimalBlockSize();
    compressor.setBlockSize(result.blockSize);
  }

  // 圧縮の実行
  if (!compressor.compress())
  {
    std::cerr << "圧縮処理に失敗しました" << std::endl;
    return 1;
  }

  // 復元の実行
  if (!compressor.decompress())
  {
    std::cerr << "復元処理に失敗しました" << std::endl;
    return 1;
  }

  std::cout << "処理が正常に完了しました" << std::endl;
  return 0;
}