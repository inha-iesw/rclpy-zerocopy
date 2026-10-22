// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "rclpy_agnocast/types/field_ops.hpp"

namespace
{
using rclpy_agnocast::types::array_ops;
using rclpy_agnocast::types::BufferSpan;
using rclpy_agnocast::types::error_status;
using rclpy_agnocast::types::ErrorCode;
using rclpy_agnocast::types::FieldRef;
using rclpy_agnocast::types::scalar_ops;
using rclpy_agnocast::types::ScalarValue;
using rclpy_agnocast::types::sequence_ops;
using rclpy_agnocast::types::Status;
using rclpy_agnocast::types::string_ops;

TEST(FieldOperations, NarrowIntegerRejectsOverflowWithoutChangingValue)
{
  std::int8_t value = -12;
  FieldRef ref{&value, scalar_ops<std::int8_t>()};
  ScalarValue input{};
  input.signed_integer = 128;
  EXPECT_EQ(ref.ops->write(ref, input).code, ErrorCode::kValue);
  EXPECT_EQ(value, -12);
  input.signed_integer = -128;
  EXPECT_EQ(ref.ops->write(ref, input).code, ErrorCode::kOk);
  EXPECT_EQ(value, -128);
}

TEST(FieldOperations, BoundedSequenceRejectsBeforeMutationAndAccountsRetainedCapacity)
{
  std::vector<float> value;
  const auto ops = sequence_ops<std::vector<float>>(scalar_ops<float>(), 8);
  FieldRef ref{&value, &ops};
  EXPECT_EQ(ops.resize(ref, 8).code, ErrorCode::kOk);
  EXPECT_EQ(ops.resize(ref, 9).code, ErrorCode::kValue);
  EXPECT_EQ(value.size(), 8U);
  EXPECT_EQ(ops.resize(ref, 0).code, ErrorCode::kOk);
  std::size_t bytes = 0;
  EXPECT_EQ(ops.dynamic_bytes(ref, &bytes).code, ErrorCode::kOk);
  EXPECT_EQ(bytes, value.capacity() * sizeof(float));
}

TEST(FieldOperations, PackedBoolUsesElementAccessAndRejectsBuffer)
{
  std::vector<bool> value(3, false);
  const auto ops = sequence_ops<std::vector<bool>>(scalar_ops<bool>(), 8);
  FieldRef ref{&value, &ops}, child;
  ASSERT_EQ(ops.child(ref, 1, &child).code, ErrorCode::kOk);
  ScalarValue input{};
  input.boolean = true;
  ASSERT_EQ(child.ops->write(child, input).code, ErrorCode::kOk);
  EXPECT_TRUE(value[1]);
  EXPECT_FALSE(value[0]);
  BufferSpan span;
  EXPECT_EQ(ops.buffer(ref, &span).code, ErrorCode::kUnsupported);
  std::size_t bytes = 0;
  ASSERT_EQ(ops.dynamic_bytes(ref, &bytes).code, ErrorCode::kOk);
  EXPECT_EQ(bytes, (value.capacity() + 7) / 8);
}

TEST(FieldOperations, FixedBoolArrayExportsNativeElements)
{
  std::array<bool, 4> value{};
  const auto ops = array_ops<decltype(value)>(scalar_ops<bool>());
  FieldRef ref{&value, &ops};
  BufferSpan span;
  ASSERT_EQ(ops.buffer(ref, &span).code, ErrorCode::kOk);
  EXPECT_EQ(span.data, value.data());
  EXPECT_EQ(span.size, 4U);
  EXPECT_STREQ(span.format, "?");
  EXPECT_EQ(ops.resize, nullptr);
}

TEST(FieldOperations, StringsChargeExternalCapacityIncludingTerminatorButNotInlineStorage)
{
  std::string value;
  const auto ops = string_ops<std::string>();
  FieldRef ref{&value, &ops};
  std::size_t bytes = 999;
  ASSERT_EQ(ops.dynamic_bytes(ref, &bytes).code, ErrorCode::kOk);
  EXPECT_EQ(bytes, 0U);  // Installed libstdc++ stores this default string inline.
  ASSERT_EQ(ops.resize(ref, 1024).code, ErrorCode::kOk);
  ASSERT_EQ(ops.dynamic_bytes(ref, &bytes).code, ErrorCode::kOk);
  EXPECT_EQ(bytes, value.capacity() + 1);
}

TEST(FieldOperations, NestedStringsAccountOnlyLiveChildrenPlusRetainedVectorAllocation)
{
  std::vector<std::u16string> value(3, std::u16string(64, u'x'));
  const auto strings = string_ops<std::u16string>();
  const auto ops = sequence_ops<decltype(value)>(&strings, 4);
  FieldRef ref{&value, &ops};
  ASSERT_EQ(ops.resize(ref, 1).code, ErrorCode::kOk);
  std::size_t bytes = 0;
  ASSERT_EQ(ops.dynamic_bytes(ref, &bytes).code, ErrorCode::kOk);
  EXPECT_EQ(
    bytes, value.capacity() * sizeof(std::u16string) +
    (value.front().capacity() + 1) * sizeof(char16_t));
}

TEST(FieldOperations, NestedAccountingPreservesTheProvidersMemoryFailureStatus)
{
  std::vector<std::string> value(1);
  auto child = string_ops<std::string>();
  child.dynamic_bytes = [] (FieldRef, std::size_t *) noexcept->Status {
    return error_status(ErrorCode::kMemory, "nested accounting allocation failed");
  };
  const auto ops = sequence_ops<decltype(value)>(&child);
  std::size_t bytes = 123;
  const auto status = ops.dynamic_bytes({&value, &ops}, &bytes);
  EXPECT_EQ(ErrorCode::kMemory, status.code);
  EXPECT_STREQ("nested accounting allocation failed", status.message);
  EXPECT_EQ(123U, bytes);
}
}  // namespace
