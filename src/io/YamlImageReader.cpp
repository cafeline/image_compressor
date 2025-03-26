#include "binary_image_compressor/io/YamlImageReader.h"
#include "binary_image_compressor/io/FileSystem.h"
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace compressor
{
  YamlImageReader::YamlImageReader() : filePath(""), pgmReader(nullptr), pgmFilePath("")
  {
  }

  YamlImageReader::YamlImageReader(const std::string &path) : filePath(path), pgmReader(nullptr), pgmFilePath("")
  {
    loadPgmPathFromYaml();
  }

  YamlImageReader::~YamlImageReader() = default;

  void YamlImageReader::setFilePath(const std::string &path)
  {
    filePath = path;
    loadPgmPathFromYaml();
  }

  std::string YamlImageReader::getFilePath() const
  {
    return filePath;
  }

  std::string YamlImageReader::getPgmFilePath() const
  {
    return pgmFilePath;
  }

  bool YamlImageReader::loadPgmPathFromYaml()
  {
    if (filePath.empty())
    {
      std::cerr << "YAMLファイルパスが設定されていません" << std::endl;
      return false;
    }

    try
    {
      pgmFilePath = FileSystem::getPgmPathFromYaml(filePath);
      if (pgmFilePath.empty())
      {
        std::cerr << "YAMLファイルからPGMパスを取得できませんでした" << std::endl;
        return false;
      }

      // PgmImageReaderのインスタンスを作成
      pgmReader = std::make_unique<PgmImageReader>(pgmFilePath);
      return true;
    }
    catch (const std::exception &e)
    {
      std::cerr << "YAMLファイルの解析エラー: " << e.what() << std::endl;
      return false;
    }
  }

  bool YamlImageReader::readHeader(ImageHeader &header, std::vector<char> &headerData)
  {
    if (!pgmReader)
    {
      if (!loadPgmPathFromYaml())
      {
        return false;
      }
    }

    return pgmReader->readHeader(header, headerData);
  }

  bool YamlImageReader::readImageData(std::vector<uint8_t> &imageData)
  {
    if (!pgmReader)
    {
      if (!loadPgmPathFromYaml())
      {
        return false;
      }
    }

    return pgmReader->readImageData(imageData);
  }

} // namespace compressor