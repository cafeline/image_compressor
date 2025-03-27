#include "binary_image_compressor/io/ImageIO.h"
#include "binary_image_compressor/io/FileSystem.h"
#include "binary_image_compressor/io/ImageReader.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <sstream>

namespace compressor
{

  bool ImageIO::parseHeader(const std::string &filePath,
                            ImageHeader &header,
                            std::vector<char> &headerData)
  {
    // ファイル拡張子の確認
    std::string extension = FileSystem::getFileExtension(filePath);
    if (extension == "yaml" || extension == "yml")
    {
      return parseYamlHeader(filePath, header, headerData);
    }
    else
    {
      return parsePgmHeader(filePath, header, headerData);
    }
  }

  bool ImageIO::parseYamlHeader(const std::string &filePath,
                                ImageHeader &header,
                                std::vector<char> &headerData)
  {
    try
    {
      // YAMLファイルから参照先のPGMファイルパスを取得
      std::string pgmPath = FileSystem::getPgmPathFromYaml(filePath);
      std::cout << "PGMファイルの絶対パス: " << pgmPath << std::endl;

      // PGMファイルのヘッダーを解析
      return parsePgmHeader(pgmPath, header, headerData);
    }
    catch (const std::exception &e)
    {
      std::cerr << "YAMLファイルの解析エラー: " << e.what() << std::endl;
      return false;
    }
  }

  bool ImageIO::parsePgmHeader(const std::string &filePath,
                               ImageHeader &header,
                               std::vector<char> &headerData)
  {
    auto reader = createImageReader(filePath);
    if (!reader)
    {
      std::cerr << "対応するリーダーを作成できません: " << filePath << std::endl;
      return false;
    }

    return reader->readHeader(header, headerData);
  }

  bool ImageIO::readYamlImageData(const std::string &filePath,
                                  std::vector<uint8_t> &imageData)
  {
    auto reader = createImageReader(filePath);
    if (!reader)
    {
      std::cerr << "対応するリーダーを作成できません: " << filePath << std::endl;
      return false;
    }

    return reader->readImageData(imageData);
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