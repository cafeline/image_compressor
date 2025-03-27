#ifndef IMAGE_IO_H
#define IMAGE_IO_H

#include <string>
#include <vector>
#include <cstdint>
#include "binary_image_compressor/model/ImageHeader.h"

namespace compressor
{

  /**
   * @brief 画像入出力を担当するクラス
   *
   * 画像ファイルの読み込みと保存を担当します。
   * 実際の画像処理はImageProcessorクラスが担当します。
   */
  class ImageIO
  {
  public:
    /**
     * @brief 入力画像のヘッダーを解析する
     * @param filePath 入力ファイルパス
     * @param header 出力用ヘッダー構造体
     * @param headerData ヘッダーデータのバイト列
     * @return 解析に成功した場合はtrue
     */
    static bool parseHeader(const std::string &filePath,
                            ImageHeader &header,
                            std::vector<char> &headerData);

    /**
     * @brief YAMLファイルのヘッダーを解析する
     * @param filePath YAMLファイルパス
     * @param header 出力用ヘッダー構造体
     * @param headerData ヘッダーデータのバイト列
     * @return 解析に成功した場合はtrue
     */
    static bool parseYamlHeader(const std::string &filePath,
                                ImageHeader &header,
                                std::vector<char> &headerData);

    /**
     * @brief PGMファイルのヘッダーを解析する
     * @param filePath PGMファイルパス
     * @param header 出力用ヘッダー構造体
     * @param headerData ヘッダーデータのバイト列
     * @return 解析に成功した場合はtrue
     */
    static bool parsePgmHeader(const std::string &filePath,
                               ImageHeader &header,
                               std::vector<char> &headerData);

    /**
     * @brief YAMLファイルから画像データを読み込む
     * @param filePath YAMLファイルパス
     * @param imageData 出力用画像データバッファ
     * @return 読み込みに成功した場合はtrue
     */
    static bool readYamlImageData(const std::string &filePath,
                                  std::vector<uint8_t> &imageData);

    /**
     * @brief 復元した画像を保存する
     * @param outputPath 出力ファイルパス
     * @param headerData ヘッダーデータ
     * @param imageData 画像データ
     * @return 保存に成功した場合はtrue
     */
    static bool saveDecompressedImage(const std::string &outputPath,
                                      const std::vector<char> &headerData,
                                      const std::vector<uint8_t> &imageData);
  };

} // namespace compressor

#endif // IMAGE_IO_H