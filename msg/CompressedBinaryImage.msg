std_msgs/Header header

# 元画像情報
uint32 original_width   # 元画像の幅
uint32 original_height  # 元画像の高さ
uint8 block_size        # 使用したブロックサイズ
float32 compression_ratio  # 圧縮率（%）

# 辞書情報
uint16 pattern_count    # パターン数
uint8[] pattern_data    # 全パターンデータ（連結形式）
uint8 pattern_bytes     # 1パターンあたりのバイト数

# インデックスデータ
bool use_8bit_indices   # true: 8bit indices, false: 16bit indices
uint8[] block_indices_8bit   # 8bitインデックス (use_8bit_indices=trueの場合使用)
uint16[] block_indices_16bit # 16bitインデックス (use_8bit_indices=falseの場合使用)