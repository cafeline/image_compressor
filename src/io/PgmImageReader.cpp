#include "binary_image_compressor/io/PgmImageReader.h"
#include <iostream>

namespace compressor
{
  PgmImageReader::PgmImageReader() : filePath("")
  {
  }

  PgmImageReader::PgmImageReader(const std::string &filePath) : filePath(filePath)
  {
  }

  PgmImageReader::~PgmImageReader()
  {
    if (fileStream.is_open())
    {
      fileStream.close();
    }
  }

  void PgmImageReader::setFilePath(const std::string &path)
  {
    filePath = path;
    if (fileStream.is_open())
    {
      fileStream.close();
    }
  }

  std::string PgmImageReader::getFilePath() const
  {
    return filePath;
  }

  bool PgmImageReader::openFile()
  {
    if (filePath.empty())
    {
      std::cerr << "ファイルパスが設定されていません" << std::endl;
      return false;
    }

    if (fileStream.is_open())
    {
      fileStream.close();
    }

    fileStream.open(filePath, std::ios::binary);
    if (!fileStream)
    {
      std::cerr << "入力ファイルを開けません: " << filePath << std::endl;
      return false;
    }

    return true;
  }

  bool PgmImageReader::readHeader(ImageHeader &header, std::vector<char> &headerData)
  {
    return parsePgmHeader(header, headerData);
  }

  bool PgmImageReader::parsePgmHeader(ImageHeader &header, std::vector<char> &headerData)
  {
    if (!openFile())
    {
      return false;
    }

    // ヘッダー読み込み処理
    std::string line;
    headerData.clear();
    char c;

    try
    {
      // マジックナンバーの確認（P5またはP2）
      fileStream >> line;
      std::cout << "PGMマジックナンバー: " << line << std::endl;
      if (line != "P5" && line != "P2")
      {
        std::cerr << "無効なPGMファイル形式: " << line << std::endl;
        return false;
      }
      header.isPGMBinary = (line == "P5");

      // 改行文字を取得
      fileStream.get(c);

      // コメント行のスキップ
      while (fileStream.peek() == '#')
      {
        std::getline(fileStream, line);
        std::cout << "PGMコメント行: " << line << std::endl;
      }

      // 幅と高さの読み込み
      int width, height;
      fileStream >> width >> height;
      std::cout << "PGM画像サイズ: " << width << "x" << height << std::endl;

      if (width <= 0 || height <= 0)
      {
        std::cerr << "無効な画像サイズ: " << width << "x" << height << std::endl;
        return false;
      }

      // 最大輝度値の読み込み
      int maxval;
      fileStream >> maxval;
      std::cout << "PGM最大輝度値: " << maxval << std::endl;

      if (maxval <= 0 || maxval > 255)
      {
        std::cerr << "無効な最大輝度値: " << maxval << std::endl;
        return false;
      }

      // ヘッダー情報の設定
      header.width = width;
      header.height = height;

      // ヘッダーの終わりの位置を記録
      std::streampos headerEnd = fileStream.tellg();
      fileStream.get(c); // データ部分の直前の1バイトを読み込む
      header.start = static_cast<size_t>(headerEnd) + 1;
      std::cout << "PGMヘッダーサイズ: " << header.start << " バイト" << std::endl;

      // ここまでの内容をヘッダーデータとして保存
      fileStream.seekg(0, std::ios::beg);
      headerData.resize(static_cast<size_t>(headerEnd) + 1);
      fileStream.read(headerData.data(), headerData.size());

      // P2形式の場合、P5形式のヘッダーを生成
      if (!header.isPGMBinary)
      {
        std::string p5Header = "P5\n";
        p5Header += std::to_string(width) + " " + std::to_string(height) + "\n";
        p5Header += std::to_string(maxval) + "\n";
        headerData.assign(p5Header.begin(), p5Header.end());
        header.start = headerData.size();
        std::cout << "ASCII PGMをバイナリPGMに変換するためのヘッダーを生成: " << header.start << " バイト" << std::endl;
      }

      return true;
    }
    catch (const std::exception &e)
    {
      std::cerr << "PGMヘッダー解析エラー: " << e.what() << std::endl;
      return false;
    }
  }

  bool PgmImageReader::readImageData(std::vector<uint8_t> &imageData)
  {
    if (!openFile())
    {
      return false;
    }

    // ヘッダー情報を読み込む（サイズ情報が必要）
    ImageHeader header;
    std::vector<char> headerData;
    if (!parsePgmHeader(header, headerData))
    {
      std::cerr << "ヘッダー解析に失敗しました" << std::endl;
      return false;
    }

    // 画像データの読み込み
    imageData.resize(header.width * header.height);

    if (header.isPGMBinary)
    {
      // バイナリPGM形式 (P5)
      fileStream.seekg(header.start);
      if (!fileStream.read(reinterpret_cast<char *>(imageData.data()), imageData.size()))
      {
        std::cerr << "画像データの読み込みエラー" << std::endl;
        return false;
      }
    }
    else
    {
      // ASCII PGM形式 (P2)
      std::cout << "ASCII PGM形式を読み込んでいます" << std::endl;

      // ヘッダー部分をスキップ
      fileStream.seekg(header.start);

      // ASCII形式ではピクセル値が空白で区切られたテキストとして格納されている
      for (size_t i = 0; i < imageData.size(); ++i)
      {
        int pixelValue;
        fileStream >> pixelValue;

        if (fileStream.fail())
        {
          std::cerr << "ピクセルデータの読み込みエラー（位置: " << i << "）" << std::endl;
          return false;
        }

        imageData[i] = static_cast<uint8_t>(pixelValue);
      }
    }

    return true;
  }

} // namespace compressor