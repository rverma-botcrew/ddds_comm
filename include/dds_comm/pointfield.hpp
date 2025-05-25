#pragma once

#include <string>
#include <cstdint>

namespace dds_msgs
{
namespace msg
{

class PointField
{
public:
  static constexpr uint8_t INT8 = 1;
  static constexpr uint8_t UINT8 = 2;
  static constexpr uint8_t INT16 = 3;
  static constexpr uint8_t UINT16 = 4;
  static constexpr uint8_t INT32 = 5;
  static constexpr uint8_t UINT32 = 6;
  static constexpr uint8_t FLOAT32 = 7;
  static constexpr uint8_t FLOAT64 = 8;
  
  std::string name; 
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;

  PointField() = default;

  PointField(const std::string& name_, uint32_t offset_, uint8_t datatype_, uint32_t count_)
      : name(name_), offset(offset_), datatype(datatype_), count(count_) {}
};

} // namespace msg

} // namespace dds_msgs