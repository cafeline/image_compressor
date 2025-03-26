#ifndef YAML_IMAGE_READER_H
#define YAML_IMAGE_READER_H

#include "binary_image_compressor/io/ImageReader.h"
#include "binary_image_compressor/io/PgmImageReader.h"
#include <memory>

namespace compressor
{
  /**
   * @brief YAML形式の画像設定ファイルを読み込むクラス
   *
   * YAMLファイルからPGMファイルへのパスを取得し、
   * 実際の画像読み込みはPgmImageReaderに委譲します。
   */
  class YamlImageReader : public ImageReader
  {
  private:
    std::string filePath;
    std::unique_ptr<PgmImageReader> pgmReader;
    std::string pgmFilePath;

  public:
    /**
     * @brief コンストラクタ
     */
    YamlImageReader();

    /**
     * @brief パスを指定するコンストラクタ
     * @param filePath YAMLファイルのパス
     */
    explicit YamlImageReader(const std::string &filePath);

    /**
     * @brief デストラクタ
     */
    ~YamlImageReader() override;

    /**
     * @brief ファイルパスを設定する
     * @param filePath YAMLファイルのパス
     */
    void setFilePath(const std::string &filePath) override;

    /**
     * @brief 現在設定されているファイルパス（YAML）を取得する
     * @return ファイルパス
     */
    std::string getFilePath() const override;

    /**
     * @brief 内部で使用しているPGMファイルのパスを取得する
     * @return PGMファイルパス
     */
    std::string getPgmFilePath() const;

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
     * @brief YAMLファイルからPGMファイルのパスを取得する
     * @return 成功した場合はtrue
     */
    bool loadPgmPathFromYaml();
  };

} // namespace compressor

#endif // YAML_IMAGE_READER_H