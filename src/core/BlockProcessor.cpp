#include "binary_image_compressor/core/BlockProcessor.h"
#include <iostream>
#include <fstream>
#include <thread>

namespace compressor
{

  BlockProcessor::BlockProcessor(int blockRowSize, int blockColumnSize) : blockRowSize(blockRowSize),
                                                                          blockColumnSize(blockColumnSize)
  {
  }

  void BlockProcessor::calculateBlockCount(const ImageHeader &header,
                                           int &rowBlocks,
                                           int &colBlocks,
                                           int &totalBlocks) const
  {
    rowBlocks = (header.height + blockRowSize - 1) / blockRowSize;
    colBlocks = (header.width + blockColumnSize - 1) / blockColumnSize;
    totalBlocks = rowBlocks * colBlocks;
  }

  void BlockProcessor::calculatePatternSize(int &patternBits, int &patternBytes) const
  {
    patternBits = blockRowSize * blockColumnSize;
    patternBytes = (patternBits + 7) / 8;
  }

  bool BlockProcessor::divideIntoBlocks(const std::string &binarizedPath,
                                        const std::string &patternDataPath,
                                        const ImageHeader &header)
  {
    std::cout << "ブロック分割処理中..." << std::endl;

    std::ifstream input(binarizedPath, std::ios::binary);
    if (!input)
    {
      std::cerr << "2値化画像ファイルを開けません: " << binarizedPath << std::endl;
      return false;
    }

    // ヘッダー部分をスキップ
    input.seekg(header.start);

    // 画像データの読み込み
    std::vector<uint8_t> imageData(header.width * header.height);
    if (!input.read(reinterpret_cast<char *>(imageData.data()), imageData.size()))
    {
      std::cerr << "画像データの読み込みエラー" << std::endl;
      return false;
    }

    input.close();

    // ブロック数の計算
    int rowBlocks, colBlocks, totalBlocks;
    calculateBlockCount(header, rowBlocks, colBlocks, totalBlocks);

    // パターンサイズの計算を並列処理前に行い、ラムダ関数でアクセスできるようにする
    int patternBits, patternBytes;
    calculatePatternSize(patternBits, patternBytes);

    // 並列処理のためのブロッククラスタリング
    const int numThreads = std::thread::hardware_concurrency();
    const int blocksPerThread = (rowBlocks + numThreads - 1) / numThreads;

    std::vector<std::thread> threads;
    std::vector<std::vector<uint8_t>> threadResults(numThreads);

    for (int t = 0; t < numThreads; ++t) {
      threads.emplace_back([&, t, patternBytes]() {
        int startRow = t * blocksPerThread;
        int endRow = std::min(startRow + blocksPerThread, rowBlocks);

        // このスレッドが処理するブロックのパターンを蓄積
        std::vector<uint8_t> localPatterns;

        // 各ブロックを処理
        for (int i = startRow; i < endRow; ++i)
        {
          for (int j = 0; j < colBlocks; ++j)
          {
            std::vector<uint8_t> blockPattern(patternBytes, 0);

            // ブロック内の各ピクセルを処理
            for (int r = 0; r < blockRowSize; ++r)
            {
              int row = i * blockRowSize + r;
              if (row >= header.height)
                continue;

              for (int c = 0; c < blockColumnSize; ++c)
              {
                int col = j * blockColumnSize + c;
                if (col >= header.width)
                  continue;

                // ピクセル値を取得（255または0）
                uint8_t pixelValue = imageData[row * header.width + col];

                // ビットパターンに変換（255→1, 0→0）
                if (pixelValue == 255)
                {
                  int bitPos = r * blockColumnSize + c;
                  int bytePos = bitPos / 8;
                  int bitOffset = 7 - (bitPos % 8);
                  blockPattern[bytePos] |= (1 << bitOffset);
                }
              }
            }

            // パターンをファイルに書き込む
            localPatterns.insert(localPatterns.end(), blockPattern.begin(), blockPattern.end());
          }
        }

        // 結果を保存
        threadResults[t] = std::move(localPatterns);
      });
    }

    // すべてのスレッドが完了するのを待機
    for (auto& thread : threads) {
      if (thread.joinable()) {
        thread.join();
      }
    }

    // 結果を統合
    std::ofstream patternFile(patternDataPath, std::ios::binary);
    for (const auto& result : threadResults) {
      patternFile.write(reinterpret_cast<const char*>(result.data()), result.size());
    }

    patternFile.close();
    std::cout << "ブロック分割完了: " << patternDataPath << std::endl;

    return true;
  }

  bool BlockProcessor::reconstructImage(const std::vector<uint16_t> &indices,
                                        const std::vector<std::vector<uint8_t>> &patterns,
                                        const ImageHeader &header,
                                        std::vector<uint8_t> &imageData) const
  {
    // 画像バッファの準備
    imageData.resize(header.width * header.height, 0);

    // ブロック数の計算
    int rowBlocks, colBlocks, totalBlocks;
    calculateBlockCount(header, rowBlocks, colBlocks, totalBlocks);

    if (indices.size() != static_cast<size_t>(rowBlocks * colBlocks))
    {
      std::cerr << "インデックス数がブロック数と一致しません" << std::endl;
      return false;
    }

    // 各ブロックを処理
    int blockIndex = 0;
    for (int i = 0; i < rowBlocks; ++i)
    {
      for (int j = 0; j < colBlocks; ++j)
      {
        if (blockIndex >= static_cast<int>(indices.size()))
        {
          std::cerr << "インデックスの範囲外エラー" << std::endl;
          return false;
        }

        uint16_t patternIndex = indices[blockIndex++];
        if (patternIndex >= patterns.size())
        {
          std::cerr << "パターンインデックスの範囲外エラー" << std::endl;
          return false;
        }

        const std::vector<uint8_t> &patternData = patterns[patternIndex];

        // ブロックをイメージデータに復元
        for (int r = 0; r < blockRowSize; ++r)
        {
          int row = i * blockRowSize + r;
          if (row >= header.height)
            continue;

          for (int c = 0; c < blockColumnSize; ++c)
          {
            int col = j * blockColumnSize + c;
            if (col >= header.width)
              continue;

            // ビットパターンから画素値を復元（1なら255、0なら0）
            int bitPos = r * blockColumnSize + c;
            int bytePos = bitPos / 8;
            int bitOffset = 7 - (bitPos % 8);

            if (static_cast<size_t>(bytePos) < patternData.size() &&
                (patternData[bytePos] & (1 << bitOffset)))
            {
              imageData[row * header.width + col] = 255;
            }
            else
            {
              imageData[row * header.width + col] = 0;
            }
          }
        }
      }
    }

    return true;
  }

} // namespace compressor