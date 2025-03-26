#include "binary_image_compressor/io/ImageReader.h"
#include "binary_image_compressor/io/PgmImageReader.h"
#include "binary_image_compressor/io/YamlImageReader.h"
#include "binary_image_compressor/io/FileSystem.h"
#include <iostream>

namespace compressor
{
  std::unique_ptr<ImageReader> createImageReader(const std::string &filePath)
  {
    // ファイル拡張子の確認
    std::string extension = FileSystem::getFileExtension(filePath);

    // 拡張子に基づいて適切なリーダーを作成
    if (extension == "yaml" || extension == "yml")
    {
      std::cout << "YAMLファイル形式が検出されました: " << filePath << std::endl;
      return std::make_unique<YamlImageReader>(filePath);
    }
    else if (extension == "pgm")
    {
      std::cout << "PGMファイル形式が検出されました: " << filePath << std::endl;
      return std::make_unique<PgmImageReader>(filePath);
    }
    else
    {
      std::cerr << "未対応のファイル形式です: " << extension << std::endl;
      return nullptr;
    }
  }
} // namespace compressor