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
uint16[] block_indices  # 各ブロックのパターンインデックス