// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef TYPES__LEASE_ROOT_HPP_
#define TYPES__LEASE_ROOT_HPP_

#include <memory>
#include <string>
#include "root.hpp"
#include "../subscriber/lease.hpp"

namespace rclpy_agnocast
{
namespace types
{
// Immutable receiver ownership has no publisher lifecycle or creator-thread restriction.
// Every field proxy and exporter retains this owner, hence one queue budget reservation.
class RootLeaseOwner
{
public:
  explicit RootLeaseOwner(std::shared_ptr<LeaseBufferOwner> owner);
  FieldInfo info(const FieldPath & path, std::uint64_t generation) const;
  ScalarValue read(const FieldPath & path, std::uint64_t generation) const;
  std::string text_bytes(const FieldPath & path, std::uint64_t generation) const;
  BufferSpan buffer(const FieldPath & path, std::uint64_t generation) const;
  // Raw-pointer consumers require OS-enforced foreign RO storage, beyond buffer flags.
  BufferSpan foreign_readonly_buffer(const FieldPath & path, std::uint64_t generation) const;
  std::size_t accounted_bytes() const noexcept {return owner_->buffer().size;}
#ifdef BUILD_TESTING
  const void * native_root() const noexcept {return owner_->buffer().typed_root;}
#endif

private:
  FieldRef resolve(const FieldPath & path, std::uint64_t generation) const;
  const std::shared_ptr<LeaseBufferOwner> owner_;
};
}  // namespace types
}  // namespace rclpy_agnocast
#endif  // TYPES__LEASE_ROOT_HPP_
