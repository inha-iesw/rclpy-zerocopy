// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "rclpy_agnocast/types/adapter.hpp"
#include "rclpy_agnocast/types/field_ops.hpp"

namespace
{
using rclpy_agnocast::types::ADAPTER_ABI_VERSION;
using rclpy_agnocast::types::compiled_abi_tag;
using rclpy_agnocast::types::message_ops;
using rclpy_agnocast::types::NativeEndpoint;
using rclpy_agnocast::types::ProviderDescriptor;
using rclpy_agnocast::types::Status;
using rclpy_agnocast::types::status_from_exception;
using rclpy_agnocast::types::TypeOps;

Status fail_creation(void *, const char *, std::size_t, NativeEndpoint * endpoint) noexcept
{
  *endpoint = {};
  try {
    throw std::runtime_error("provider-local factory failure");
  } catch (...) {
    return status_from_exception();
  }
}

Status unused_subscription(
  void *, const char *, std::size_t, void *, void *,
  rclpy_agnocast::types::NativeReceive,
  rclpy_agnocast::types::NativeSubscriptionEndpoint *) noexcept
{return {rclpy_agnocast::types::ErrorCode::kUnsupported, "fixture has no subscription"};}

const auto root = message_ops<std::int64_t>(nullptr, 0);
const TypeOps operations{&root, fail_creation, unused_subscription};
const ProviderDescriptor provider{
  ADAPTER_ABI_VERSION, "lifetime_test_provider", "lifetime_test_provider._plugin",
  compiled_abi_tag(), std::string(64, 'a'),
  {{"lifetime_test/msg/Root", std::string(64, 'b'), "[]", sizeof(std::int64_t),
    alignof(std::int64_t), &operations}}
};
}  // namespace

/**
 * @brief Expose the unloadable test provider without registering it implicitly.
 * @return Borrowed descriptor valid only while this library is loaded or process-pinned.
 */
extern "C" __attribute__((visibility("default")))
const rclpy_agnocast::types::ProviderDescriptor * test_provider_descriptor()
{
  return &provider;
}
