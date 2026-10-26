// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include "lease_root.hpp"
#include <stdexcept>
#include <utility>
#include "../subscriber/provenance.hpp"

namespace rclpy_agnocast
{
namespace types
{
RootLeaseOwner::RootLeaseOwner(std::shared_ptr<LeaseBufferOwner> owner)
: owner_(std::move(owner))
{
  if (!owner_ || !owner_->buffer().typed_root || !owner_->buffer().descriptor ||
    !owner_->buffer().descriptor->ops || !owner_->buffer().descriptor->ops->root)
  {
    throw std::invalid_argument("received storage has no typed root descriptor");
  }
}

FieldRef RootLeaseOwner::resolve(const FieldPath & path, std::uint64_t generation) const
{
  if (generation != 0) {throw std::runtime_error("field proxy has a stale structural generation");}
  const auto & received = owner_->buffer();
  // FieldRef predates receive access and carries void*. Only non-mutating child/size/read/
  // buffer callbacks are reachable here; no write/resize callback receives this reference.
  FieldRef ref{const_cast<void *>(received.typed_root), received.descriptor->ops->root};
  for (auto index : path) {
    if (!ref.ops->child) {throw std::invalid_argument("scalar has no child fields");}
    FieldRef next;
    check_status(ref.ops->child(ref, index, &next));
    ref = next;
  }
  return ref;
}

FieldInfo RootLeaseOwner::info(const FieldPath & path, std::uint64_t generation) const
{
  const auto ref = resolve(path, generation);
  std::size_t size = ref.ops->member_count;
  if (ref.ops->size) {check_status(ref.ops->size(ref, &size));}
  return {ref.ops, size, 0};
}

ScalarValue RootLeaseOwner::read(const FieldPath & path, std::uint64_t generation) const
{
  const auto ref = resolve(path, generation);
  if (!ref.ops->read) {throw std::invalid_argument("field is not a scalar");}
  ScalarValue value{};
  check_status(ref.ops->read(ref, &value));
  return value;
}

std::string RootLeaseOwner::text_bytes(const FieldPath & path, std::uint64_t generation) const
{
  const auto ref = resolve(path, generation);
  if (ref.ops->kind != FieldKind::kString && ref.ops->kind != FieldKind::kWString) {
    throw std::invalid_argument("to_text requires a string field");
  }
  const auto span = buffer(path, generation);
  if (!span.size) {return std::string();}
  return std::string(static_cast<const char *>(span.data), span.size * span.item_size);
}

BufferSpan RootLeaseOwner::buffer(const FieldPath & path, std::uint64_t generation) const
{
  const auto ref = resolve(path, generation);
  if (!ref.ops->buffer) {throw std::invalid_argument("field does not support a buffer");}
  BufferSpan span;
  check_status(ref.ops->buffer(ref, &span));
  if (!span.item_size || !span.format || (span.size && !span.data) ||
    span.size > static_cast<std::size_t>(PTRDIFF_MAX) / span.item_size)
  {
    throw BufferBusy("invalid or oversized native buffer");
  }
  return span;
}

BufferSpan RootLeaseOwner::foreign_readonly_buffer(
  const FieldPath & path, std::uint64_t generation) const
{
  const auto span = buffer(path, generation);
  const auto & received = owner_->buffer();
  require_readonly_mapping(
    received.typed_root, received.descriptor->native_size, span.data, span.size * span.item_size);
  return span;
}
}  // namespace types
}  // namespace rclpy_agnocast
