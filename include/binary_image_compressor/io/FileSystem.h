#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

#include <string>
#include <filesystem>
#include <vector>
#include <cstdint>

namespace fs = std::filesystem;
namespace compressor
{
  /**
   * @brief ファイル操作を担当するユーティリティクラス
   *
   * ファイルの存在確認、サイズ取得、ディレクトリ作成などの一般的なファイル操作を提供します。
   */
  class FileSystem
  {
  public:
    /**
     * @brief ファイルの存在を確認する
     * @param path 確認するファイルパス
     * @return ファイルが存在する場合はtrue
     */
    static bool exists(const std::string &path);

    /**
     * @brief ディレクトリを作成する
     * @param path 作成するディレクトリパス
     * @return 作成に成功した場合はtrue
     */
    static bool createDirectory(const std::string &path);

    /**
     * @brief ファイルサイズを取得する
     * @param path 対象ファイルのパス
     * @return ファイルサイズ（バイト単位）、エラー時は0
     */
    static uintmax_t getFileSize(const std::string &path);

    /**
     * @brief YAMLファイルから参照先のPGMファイルパスを取得する
     * @param yamlPath YAMLファイルのパス
     * @return PGMファイルの絶対パス
     */
    static std::string getPgmPathFromYaml(const std::string &yamlPath);

    /**
     * @brief 入力ファイルのサイズを取得する（YAMLの場合は参照先PGMファイルのサイズ）
     * @param filePath 対象ファイルのパス
     * @return ファイルサイズ（バイト単位）、エラー時は0
     */
    static uintmax_t getInputFileSize(const std::string &filePath);

    /**
     * @brief 一時ファイルパスを生成する
     * @param baseName 基本ファイル名
     * @param suffix サフィックス（拡張子を含む）
     * @return 生成された一時ファイルパス
     */
    static std::string generateTempFilePath(const std::string &baseName, const std::string &suffix);

    /**
     * @brief ファイル拡張子を取得する
     * @param filePath ファイルパス
     * @return 拡張子（ドットを含まない）
     */
    static std::string getFileExtension(const std::string &filePath);
  };

} // namespace compressor

#endif // FILE_SYSTEM_H