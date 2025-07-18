#include "binary_image_compressor/model/BinaryPattern.h"
#include <algorithm>

namespace compressor
{

  BinaryPattern::BinaryPattern(int size) : pattern(std::vector<uint8_t>((size + 7) / 8, 0))
  {
  }

  BinaryPattern::BinaryPattern(const std::vector<uint8_t> &data) : pattern(data)
  {
  }

  bool BinaryPattern::equals(const std::vector<uint8_t> &other) const
  {
    if (pattern.size() != other.size())
    {
      return false;
    }
    return std::equal(pattern.begin(), pattern.end(), other.begin());
  }

} // namespace compressor