#include "rclpy_agnocast_msgs/msg/shared_buffer.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

namespace
{

using SharedBuffer = rclpy_agnocast_msgs::msg::SharedBuffer;

static_assert(std::is_same_v<decltype(SharedBuffer::sequence), std::uint64_t>);
static_assert(std::is_same_v<decltype(SharedBuffer::data), std::vector<std::uint8_t>>);
static_assert(rosidl_generator_traits::is_message<SharedBuffer>::value);
static_assert(!rosidl_generator_traits::has_fixed_size<SharedBuffer>::value);
static_assert(!rosidl_generator_traits::has_bounded_size<SharedBuffer>::value);

TEST(SharedBufferGeneratedType, FieldsMatchSchema)
{
  SharedBuffer message;

  EXPECT_EQ(std::uint64_t{0}, message.sequence);
  EXPECT_TRUE(message.data.empty());

  const std::vector<std::uint8_t> expected_data{0U, 1U, 127U, 128U, 255U};
  message.sequence = std::numeric_limits<std::uint64_t>::max();
  message.data = expected_data;

  EXPECT_EQ(std::numeric_limits<std::uint64_t>::max(), message.sequence);
  EXPECT_EQ(expected_data, message.data);
}

TEST(SharedBufferGeneratedType, RosidlNameMatchesAgnocastRegistrationName)
{
  EXPECT_STREQ(
    rosidl_generator_traits::name<SharedBuffer>(),
    "rclpy_agnocast_msgs/msg/SharedBuffer");
}

}  // namespace
