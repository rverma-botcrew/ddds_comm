#pragma once

#include <vector>
#include <cstdint>

#include "myheader.hpp"
#include "mypointfield.hpp"


class MyPointCloud2
{
public:
  MyHeader header;
  uint32_t height;
  uint32_t width;

  std::vector<MyPointField> fields;

  bool is_bigendian;

  uint32_t point_step;
  uint32_t row_step;

  std::vector<uint8_t> data;

  bool is_dense;

  MyPointCloud2() = default;

  MyPointCloud2(const MyHeader& header_,
              uint32_t height_,
              uint32_t width_,
              const std::vector<MyPointField>& fields_,
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

