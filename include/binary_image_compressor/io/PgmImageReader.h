#ifndef PGM_IMAGE_READER_H
#define PGM_IMAGE_READER_H

#include "binary_image_compressor/io/ImageReader.h"
#include <fstream>

namespace compressor
{
  /**
   * @brief PGM形式の画像を読み込むクラス
   */
  class PgmImageReader : public ImageReader
  {
  private:
    std::string filePath;
    std::ifstream fileStream;

  public:
    /**
     * @brief コンストラクタ
     */
    PgmImageReader();

    /**
     * @brief パスを指定するコンストラクタ
     * @param filePath 画像ファイルのパス
     */
    explicit PgmImageReader(const std::string &filePath);

    /**
     * @brief デストラクタ
     */
    ~PgmImageReader() override;

    /**
     * @brief ファイルパスを設定する
     * @param filePath 画像ファイルのパス
     */
    void setFilePath(const std::string &filePath) override;

    /**
     * @brief 現在設定されているファイルパスを取得する
     * @return ファイルパス
     */
    std::string getFilePath() const override;

    /**
     * @brief ヘッダー情報を読み込む
     * @param header 出力用ヘッダー構造体
     * @param headerData ヘッダーデータのバイト列
     * @return 読み込みに成功した場合はtrue
     */
    bool readHeader(ImageHeader &header, std::vector<char> &headerData) override;

    /**
     * @brief 画像データを読み込む
     * @param imageData 出力用画像データバッファ
     * @return 読み込みに成功した場合はtrue
     */
    bool readImageData(std::vector<uint8_t> &imageData) override;

  private:
    /**
     * @brief ファイルを開く
     * @return 成功した場合はtrue
     */
    bool openFile();

    /**
     * @brief PGMヘッダーを解析する
     * @param header 出力用ヘッダー構造体
     * @param headerData ヘッダーデータのバイト列
     * @return 成功した場合はtrue
     */
    bool parsePgmHeader(ImageHeader &header, std::vector<char> &headerData);
  };

} // namespace compressor

#endif // PGM_IMAGE_READER_H