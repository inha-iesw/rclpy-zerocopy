// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef TYPED__NATIVE_FIXTURE_HPP_
#define TYPED__NATIVE_FIXTURE_HPP_

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include "rclpy_agnocast_test_msgs/msg/containers.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace typed_fixture
{
using rclpy_agnocast_test_msgs::msg::Containers;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

inline void expect(bool condition, const char * field)
{
  if (!condition) {throw std::runtime_error(field);}
}

template<typename C, typename T>
void sequence(const C & value, std::initializer_list<T> expected, const char * field)
{
  expect(value.size() == expected.size(), field);
  expect(std::equal(value.begin(), value.end(), expected.begin()), field);
}

template<typename T>
void fill_header(T & value)
{
  value.header.stamp.sec = 123;
  value.header.stamp.nanosec = 456;
  value.header.frame_id = "typed_interop_frame_with_heap_storage";
}

template<typename T>
void validate_header(const T & value)
{
  expect(value.header.stamp.sec == 123, "stamp.sec");
  expect(value.header.stamp.nanosec == 456, "stamp.nanosec");
  expect(value.header.frame_id == "typed_interop_frame_with_heap_storage", "frame_id");
}

template<typename C>
void fill_bytes(C & value, std::size_t size)
{
  value.resize(size);
  for (std::size_t index = 0; index < size; ++index) {
    value[index] = static_cast<std::uint8_t>((7 * index + 3) & 255);
  }
}

template<typename C>
void validate_bytes(const C & value, std::size_t size)
{
  expect(value.size() == size, "data.size");
  for (std::size_t index = 0; index < size; ++index) {
    expect(value[index] == ((7 * index + 3) & 255), "data.element");
  }
}

inline void fill(Image & value)
{
  fill_header(value);
  value.height = 2;
  value.width = 3;
  value.encoding = "rgb8";
  value.is_bigendian = 0;
  value.step = 9;
  fill_bytes(value.data, 18);
}

inline void validate(const Image & value)
{
  validate_header(value);
  expect(value.height == 2 && value.width == 3, "image.shape");
  expect(value.encoding == "rgb8", "encoding");
  expect(value.is_bigendian == 0 && value.step == 9, "image.layout");
  validate_bytes(value.data, 18);
}

inline void fill(PointCloud2 & value)
{
  fill_header(value);
  value.height = 1;
  value.width = 3;
  value.fields.resize(2);
  for (std::size_t index = 0; index < value.fields.size(); ++index) {
    value.fields[index].name = index == 0 ? "x" : "y";
    value.fields[index].offset = static_cast<std::uint32_t>(index * 4);
    value.fields[index].datatype = 7;
    value.fields[index].count = 1;
  }
  value.is_bigendian = false;
  value.point_step = 8;
  value.row_step = 24;
  value.is_dense = true;
  fill_bytes(value.data, 24);
}

inline void validate(const PointCloud2 & value)
{
  validate_header(value);
  expect(value.height == 1 && value.width == 3, "cloud.shape");
  expect(value.fields.size() == 2, "fields.size");
  for (std::size_t index = 0; index < value.fields.size(); ++index) {
    const auto & field = value.fields[index];
    expect(field.name == (index == 0 ? "x" : "y"), "fields.name");
    expect(field.offset == index * 4 && field.datatype == 7 && field.count == 1, "fields.layout");
  }
  expect(!value.is_bigendian && value.is_dense, "cloud.flags");
  expect(value.point_step == 8 && value.row_step == 24, "cloud.steps");
  validate_bytes(value.data, 24);
}

inline void fill(Containers & value)
{
  value.fixed = {1.25, -2.5, 0.0, 4.0};
  value.fixed_flags = {true, false, true, false};
  value.bounded = {0.5F, -1.5F, 2.5F};
  value.values = {0, 1, 127, 128, 255};
  value.bounded_flags = {false, true, false};
  value.flags = {true, false, true, false, true};
  value.text = "typed π buffer payload";
  value.bounded_text = "µ";
  value.wide_text = u"𝄞 typed wide payload";
  value.bounded_wide_text = u"한글";
  value.texts = {"abcdefabcdefabcdefabcdef", "π"};
  value.bounded_texts = {"0123456789abcdef", "x"};
  value.wide_texts = {u"큰큰큰큰큰큰큰큰큰큰큰큰", u""};
  value.bounded_wide_texts = {u"한글", u"Z"};
}

inline void validate(const Containers & value)
{
  sequence(value.fixed, {1.25, -2.5, 0.0, 4.0}, "fixed");
  sequence(value.fixed_flags, {true, false, true, false}, "fixed_flags");
  sequence(value.bounded, {0.5F, -1.5F, 2.5F}, "bounded");
  sequence(value.values, {0, 1, 127, 128, 255}, "values");
  sequence(value.bounded_flags, {false, true, false}, "bounded_flags");
  sequence(value.flags, {true, false, true, false, true}, "flags");
  expect(value.text == "typed π buffer payload", "text");
  expect(value.bounded_text == "µ", "bounded_text");
  expect(value.wide_text == u"𝄞 typed wide payload", "wide_text");
  expect(value.bounded_wide_text == u"한글", "bounded_wide_text");
  sequence(value.texts, {"abcdefabcdefabcdefabcdef", "π"}, "texts");
  sequence(value.bounded_texts, {"0123456789abcdef", "x"}, "bounded_texts");
  sequence(value.wide_texts, {u"큰큰큰큰큰큰큰큰큰큰큰큰", u""}, "wide_texts");
  sequence(value.bounded_wide_texts, {u"한글", u"Z"}, "bounded_wide_texts");
}

template<typename S>
std::size_t string_storage(const S & value)
{
  const auto data = reinterpret_cast<std::uintptr_t>(value.data());
  const auto object = reinterpret_cast<std::uintptr_t>(&value);
  if (data >= object && data < object + sizeof(S)) {return 0;}
  return (value.capacity() + 1) * sizeof(typename S::value_type);
}

template<typename C>
std::size_t strings_storage(const C & value)
{
  auto bytes = value.capacity() * sizeof(typename C::value_type);
  for (const auto & text : value) {
    bytes += string_storage(text);
  }
  return bytes;
}

inline std::size_t accounted(const Image & value)
{
  return sizeof(Image) + string_storage(value.header.frame_id) +
         string_storage(value.encoding) + value.data.capacity();
}

inline std::size_t accounted(const PointCloud2 & value)
{
  auto bytes = sizeof(PointCloud2) + string_storage(value.header.frame_id) +
    value.fields.capacity() * sizeof(sensor_msgs::msg::PointField) + value.data.capacity();
  for (const auto & field : value.fields) {
    bytes += string_storage(field.name);
  }
  return bytes;
}

inline std::size_t accounted(const Containers & value)
{
  return sizeof(Containers) + value.bounded.capacity() * sizeof(float) + value.values.capacity() +
         (value.bounded_flags.capacity() + 7) / 8 + (value.flags.capacity() + 7) / 8 +
         string_storage(value.text) + string_storage(value.bounded_text) +
         string_storage(value.wide_text) + string_storage(value.bounded_wide_text) +
         strings_storage(value.texts) + strings_storage(value.bounded_texts) +
         strings_storage(value.wide_texts) + strings_storage(value.bounded_wide_texts);
}

template<typename T>
std::uintptr_t data_address(const T & value)
{
  return reinterpret_cast<std::uintptr_t>(value.data.data());
}

inline std::uintptr_t data_address(const Containers & value)
{
  return reinterpret_cast<std::uintptr_t>(value.values.data());
}
}  // namespace typed_fixture
#endif  // TYPED__NATIVE_FIXTURE_HPP_
