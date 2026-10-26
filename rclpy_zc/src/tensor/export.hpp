// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef TENSOR__EXPORT_HPP_
#define TENSOR__EXPORT_HPP_

#include <cstdint>
#include <memory>
#include <vector>
#include "../../third_party/dlpack/dlpack.h"
#include "../types/lease_root.hpp"
#include "../types/root.hpp"

namespace rclpy_agnocast
{
namespace tensor
{
struct TensorType
{
  const char * name;
  DLDataType dtype;
  std::size_t size;
  std::size_t item_size;
  const char * format;
};

// Validate the native element contract without acquiring an export or freezing structure.
TensorType describe(const types::FieldInfo & info);

// The returned manager owns its metadata and native root only; its deleter needs no GIL.
// Without a capsule pin the caller must invoke its deleter exactly once, either through
// one consumer or on failure. Repeated deleter calls are safe only while a pin is held.
DLManagedTensor * export_dlpack(
  std::shared_ptr<types::RootOwner> root, const types::FieldPath & path,
  std::uint64_t generation, std::vector<std::int64_t> shape);
DLManagedTensor * export_dlpack(
  std::shared_ptr<types::RootLeaseOwner> root, const types::FieldPath & path,
  std::uint64_t generation, std::vector<std::int64_t> shape);

// Retain before exposing the manager to a consumer. Release is the pin holder's final
// access: it releases unconsumed storage BEFORE dropping the metadata pin. The wrapper
// destructor must be preserved by the consumer, including after renaming the capsule.
// Native payload ownership ends at the first deleter call, independently of the pin.
// A consumer that calls the deleter has exhausted the capsule even if it failed to rename it.
void retain_capsule(DLManagedTensor * managed) noexcept;
void release_capsule(DLManagedTensor * managed, bool unconsumed) noexcept;
#ifdef BUILD_TESTING
// Metadata lifetime is independent of payload exports and retained queue budget.
std::size_t live_managers() noexcept;
#endif
}  // namespace tensor
}  // namespace rclpy_agnocast
#endif  // TENSOR__EXPORT_HPP_
