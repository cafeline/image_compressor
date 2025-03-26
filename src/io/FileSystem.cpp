#include "binary_image_compressor/io/FileSystem.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace compressor
{

  bool FileSystem::exists(const std::string &path)
  {
    return fs::exists(path);
  }

  bool FileSystem::createDirectory(const std::string &path)
  {
    std::error_code ec;
    if (fs::exists(path, ec))
    {
      return true; // 既に存在する
    }

    return fs::create_directory(path, ec);
  }

  uintmax_t FileSystem::getFileSize(const std::string &path)
  {
    std::error_code ec;
    uintmax_t size = fs::file_size(path, ec);
    if (ec)
    {
      std::cerr << "ファイルサイズの取得に失敗しました: " << path << " - " << ec.message() << std::endl;
      return 0;
    }
    return size;
  }

  std::string FileSystem::getPgmPathFromYaml(const std::string &yamlPath)
  {
    try
    {
      YAML::Node config = YAML::LoadFile(yamlPath);
      std::string pgmFile = config["image"].as<std::string>();
      std::cout << "YAMLファイルから読み込んだPGMファイル: " << pgmFile << std::endl;

      // PGMファイルの絶対パスを構築
      fs::path yaml(yamlPath);
      fs::path pgm = yaml.parent_path() / pgmFile;

      return pgm.string();
    }
    catch (const std::exception &e)
    {
      std::cerr << "YAMLファイルの解析エラー: " << e.what() << std::endl;
      throw; // 例外を再スロー
    }
  }

  uintmax_t FileSystem::getInputFileSize(const std::string &filePath)
  {
    std::error_code ec;
    uintmax_t fileSize = 0;

    // 入力ファイルがYAMLの場合、参照先のPGMファイルのサイズを取得
    std::string extension = getFileExtension(filePath);
    if (extension == "yaml" || extension == "yml")
    {
      try
      {
        // YAMLファイルから参照先のPGMファイルパスを取得
        std::string pgmPath = getPgmPathFromYaml(filePath);

        // PGMファイルのサイズを取得
        fileSize = getFileSize(pgmPath);
        if (fileSize == 0)
        {
          std::cerr << "PGMファイルのサイズを取得できません: " << pgmPath << std::endl;
          return 0;
        }
        std::cout << "参照先PGMファイル: " << pgmPath << ", サイズ: " << fileSize << " バイト" << std::endl;
      }
      catch (const std::exception &e)
      {
        std::cerr << "YAMLファイルの解析エラー: " << e.what() << std::endl;
        return 0;
      }
    }
    else
    {
      // 通常のファイルの場合は直接そのサイズを取得
      fileSize = getFileSize(filePath);
      if (fileSize == 0)
      {
        std::cerr << "入力ファイルのサイズを取得できません: " << filePath << std::endl;
        return 0;
      }
    }

    return fileSize;
  }

  std::string FileSystem::generateTempFilePath(const std::string &baseName, const std::string &suffix)
  {
    fs::path temp("temp");

    // 一時ディレクトリが存在しない場合は作成
    createDirectory(temp.string());

    // 一時ファイルパスを生成
    fs::path tempFile = temp / (baseName + suffix);

    return tempFile.string();
  }

  std::string FileSystem::getFileExtension(const std::string &filePath)
  {
    size_t pos = filePath.find_last_of(".");
    if (pos != std::string::npos)
    {
      return filePath.substr(pos + 1);
    }
    return "";
  }

} // namespace compressor