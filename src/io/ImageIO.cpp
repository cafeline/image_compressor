#include "binary_image_compressor/io/ImageIO.h"
#include <iostream>
#include <fstream>

namespace compressor
{

  bool ImageIO::parseHeader(const std::string &filePath,
                            ImageHeader &header,
                            std::vector<char> &headerData)
  {
    std::ifstream inputFile(filePath, std::ios::binary);
    if (!inputFile)
    {
      std::cerr << "入力ファイルを開けません: " << filePath << std::endl;
      return false;
    }

    // ヘッダー読み込み処理
    std::string line;
    headerData.clear();
    char c;

    try {
      // マジックナンバーの確認（P5またはP2）
      inputFile >> line;
      if (line != "P5" && line != "P2")
      {
        std::cerr << "無効なPGMファイル形式: " << line << std::endl;
        return false;
      }
      // 改行文字を取得
      inputFile.get(c);

      // コメント行のスキップ
      while (inputFile.peek() == '#')
      {
        std::getline(inputFile, line);
      }

      std::cout << "デバッグ: 次の行を解析します" << std::endl;
      // 幅と高さの読み込み
      int width, height;
      inputFile >> width >> height;
      std::cout << "デバッグ: 幅=" << width << ", 高さ=" << height << std::endl;

      if (width <= 0 || height <= 0)
      {
        std::cerr << "無効な画像サイズ: " << width << "x" << height << std::endl;
        return false;
      }

      // 最大輝度値の読み込み
      int maxval;
      inputFile >> maxval;
      std::cout << "デバッグ: 最大輝度値=" << maxval << std::endl;

      if (maxval <= 0 || maxval > 255)
      {
        std::cerr << "無効な最大輝度値: " << maxval << std::endl;
        return false;
      }

      // ヘッダー情報の設定
      header.width = width;
      header.height = height;
      // ヘッダーの開始位置を設定
      std::streampos headerEnd = inputFile.tellg();
      inputFile.get(c); // データ部分の直前の1バイトを読み込む
      header.start = static_cast<size_t>(headerEnd) + 1;

      // ここまでの内容をヘッダーデータとして保存
      inputFile.seekg(0, std::ios::beg);
      headerData.resize(static_cast<size_t>(headerEnd) + 1);
      inputFile.read(headerData.data(), headerData.size());

      return true;
    } catch (const std::exception& e) {
      std::cerr << "PGMヘッダー解析エラー: " << e.what() << std::endl;
      return false;
    }
  }

  bool ImageIO::binarizeImage(const std::string &inputPath,
                              const std::string &outputPath,
                              const ImageHeader &header,
                              const std::vector<char> &headerData,
                              int threshold)
  {
    std::cout << "画像の2値化を実行中..." << std::endl;

    std::ifstream input(inputPath, std::ios::binary);
    if (!input)
    {
      std::cerr << "入力ファイルを開けません: " << inputPath << std::endl;
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

    // 2値化処理
    for (auto &pixel : imageData)
    {
      pixel = (pixel >= threshold) ? 255 : 0;
    }

    // 出力ファイルの作成
    std::ofstream output(outputPath, std::ios::binary);
    if (!output)
    {
      std::cerr << "出力ファイルを作成できません: " << outputPath << std::endl;
      return false;
    }

    // ヘッダーの書き込み
    output.write(headerData.data(), headerData.size());

    // 2値化画像データの書き込み
    output.write(reinterpret_cast<char *>(imageData.data()), imageData.size());

    output.close();
    std::cout << "2値化完了: " << outputPath << std::endl;

    return true;
  }

  bool ImageIO::saveDecompressedImage(const std::string &outputPath,
                                      const std::vector<char> &headerData,
                                      const std::vector<uint8_t> &imageData)
  {
    std::ofstream outputFile(outputPath, std::ios::binary);

    if (!outputFile)
    {
      std::cerr << "出力ファイルを作成できません: " << outputPath << std::endl;
      return false;
    }

    // ヘッダー情報を出力ファイルに書き込む
    outputFile.write(headerData.data(), headerData.size());

    // 画像データを書き込む
    outputFile.write(reinterpret_cast<const char *>(imageData.data()), imageData.size());

    if (!outputFile)
    {
      std::cerr << "画像データの書き込みエラー" << std::endl;
      return false;
    }

    outputFile.close();
    std::cout << "復元画像の保存完了: " << outputPath << std::endl;

    return true;
  }

} // namespace compressor