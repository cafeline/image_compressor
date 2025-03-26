#ifndef IMAGE_READER_H
#define IMAGE_READER_H

#include <string>
#include <vector>
#include <memory>
#include "binary_image_compressor/model/ImageHeader.h"

namespace compressor
{
  /**
   * @brief 画像読み込みのための抽象インターフェース
   *
   * 様々な画像形式を統一的に扱うためのインターフェース。
   * 具象クラスはこのインターフェースを実装して特定の画像形式を処理します。
   */
  class ImageReader
  {
  public:
    /**
     * @brief ヘッダー情報を読み込む
     * @param header 出力用ヘッダー構造体
     * @param headerData ヘッダーデータのバイト列
     * @return 読み込みに成功した場合はtrue
     */
    virtual bool readHeader(ImageHeader &header, std::vector<char> &headerData) = 0;

    /**
     * @brief 画像データを読み込む
     * @param imageData 出力用画像データバッファ
     * @return 読み込みに成功した場合はtrue
     */
    virtual bool readImageData(std::vector<uint8_t> &imageData) = 0;

    /**
     * @brief ファイルパスを設定する
     * @param filePath 画像ファイルのパス
     */
    virtual void setFilePath(const std::string &filePath) = 0;

    /**
     * @brief 現在設定されているファイルパスを取得する
     * @return ファイルパス
     */
    virtual std::string getFilePath() const = 0;

    /**
     * @brief デストラクタ
     */
    virtual ~ImageReader() = default;
  };

  /**
   * @brief ImageReaderファクトリ関数
   * @param filePath 画像ファイルのパス
   * @return 適切なImageReader実装
   */
  std::unique_ptr<ImageReader> createImageReader(const std::string &filePath);

} // namespace compressor

#endif // IMAGE_READER_H