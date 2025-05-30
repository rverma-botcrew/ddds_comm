#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "header.hpp"

namespace dds_msgs
{

namespace sensor_msgs
{

class Image
{
public:
dds_msgs::std_msgs::Header header;
uint32_t height;
uint32_t width;
std::string encoding;
uint8_t is_bigendian;
uint32_t step;
std::vector<uint8_t> data;
};
} // namespace sensor_msgs
} // namespace dds_msgs
