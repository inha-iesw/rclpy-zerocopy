// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include "rclpy_agnocast/types/operations.hpp"
#include <new>

namespace rclpy_agnocast
{
namespace types
{
void check_status(const Status & status)
{
  switch (status.code) {
    case ErrorCode::kOk: return;
    case ErrorCode::kMemory: throw std::bad_alloc();
    case ErrorCode::kValue: throw std::invalid_argument(status.message);
    case ErrorCode::kIndex: throw std::out_of_range(status.message);
    case ErrorCode::kUnsupported: throw UnsupportedOperation(status.message);
    default: throw std::runtime_error(status.message);
  }
}
}  // namespace types
}  // namespace rclpy_agnocast
