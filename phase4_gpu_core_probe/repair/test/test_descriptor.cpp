#include <gtest/gtest.h>

#include <limits>

#include "../transport/descriptor.hpp"

namespace
{
g3_transport::Message valid_message()
{
  g3_transport::Message message;
  message.core.byte_size = 64;
  message.allocation_bytes = 64;
  message.dtype_code = 1;
  message.dtype_bits = 8;
  message.dtype_lanes = 1;
  message.shape = {64};
  message.strides = {1};
  return message;
}

TEST(GpuDescriptor, DenseUint8AndFloat32HaveCorrectByteSpan)
{
  auto message = valid_message();
  EXPECT_EQ(g3_transport::tensor_bytes(message), 64u);
  message.dtype_code = 2;
  message.dtype_bits = 32;
  message.shape = {4, 4};
  message.strides = {4, 1};
  EXPECT_EQ(g3_transport::tensor_bytes(message), 64u);
}

TEST(GpuDescriptor, AllocationIdentityAndSpanMustAgree)
{
  auto message = valid_message();
  message.allocation_bytes = 128;
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
  message = valid_message();
  message.byte_offset = 1;
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
  message = valid_message();
  message.dtype_code = 2;
  message.dtype_bits = 32;
  message.shape = {1};
  message.byte_offset = 2;
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
}

TEST(GpuDescriptor, UnsupportedDtypeAndLanesAreRejected)
{
  for (const unsigned code : {3, 4, 255}) {
    auto message = valid_message();
    message.dtype_code = code;
    EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
  }
  auto message = valid_message();
  message.dtype_lanes = 2;
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
  message = valid_message();
  message.dtype_code = 2;
  message.dtype_bits = 8;
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
  message.dtype_bits = 12;
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
}

TEST(GpuDescriptor, NegativeExtentsAndNoncontiguousStridesAreRejected)
{
  auto message = valid_message();
  message.shape = {-1};
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
  message = valid_message();
  message.strides = {2};
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
  message.strides = {1, 1};
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
}

TEST(GpuDescriptor, EmptyShapeDoesNotHideStrideOverflow)
{
  auto message = valid_message();
  message.shape = {0, std::numeric_limits<int64_t>::max(), 2};
  message.strides.clear();
  EXPECT_THROW(g3_transport::tensor_bytes(message), std::invalid_argument);
}

TEST(GpuDescriptor, EmptyTensorMayPointToAllocationEnd)
{
  auto message = valid_message();
  message.shape = {0};
  message.byte_offset = 64;
  EXPECT_EQ(g3_transport::tensor_bytes(message), 0u);
}
}  // namespace
