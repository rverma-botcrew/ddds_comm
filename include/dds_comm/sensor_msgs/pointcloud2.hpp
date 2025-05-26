#pragma once

#include <vector>
#include <cstdint>

#include "header.hpp"
#include "pointfield.hpp"

namespace dds_msgs
{

namespace sensor_msgs
{

class PointCloud2
{
public:
  dds_msgs::std_msgs::Header header;
  uint32_t height;
  uint32_t width;

  std::vector<PointField> fields;

  bool is_bigendian;

  uint32_t point_step;
  uint32_t row_step;

  std::vector<uint8_t> data;

  bool is_dense;

  PointCloud2() = default;

  PointCloud2(const dds_msgs::std_msgs::Header& header_,
              uint32_t height_,
              uint32_t width_,
              const std::vector<PointField>& fields_,
              bool is_bigendian_,
              uint32_t point_step_,
              uint32_t row_step_,
              const std::vector<uint8_t>& data_,
              bool is_dense_)
      : header(header_),
        height(height_),
        width(width_),
        fields(fields_),
        is_bigendian(is_bigendian_),
        point_step(point_step_),
        row_step(row_step_),
        data(data_),
        is_dense(is_dense_) {}
};

} // namespace sensor_msgs

} // namespace dds_msgs