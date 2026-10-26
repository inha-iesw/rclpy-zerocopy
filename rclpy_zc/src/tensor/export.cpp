// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include "export.hpp"
#include <atomic>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <utility>

namespace rclpy_agnocast
{
namespace tensor
{
namespace
{
struct NativeType
{
  const char * name;
  const char * format;
  types::ScalarKind kind;
  std::uint8_t code;
  std::uint8_t bits;
};
constexpr NativeType supported[] = {
  {"bool", "?", types::ScalarKind::kBool, kDLBool, 8},
  {"int8", "b", types::ScalarKind::kSigned, kDLInt, 8},
  {"uint8", "B", types::ScalarKind::kUnsigned, kDLUInt, 8},
  {"int16", "h", types::ScalarKind::kSigned, kDLInt, 16},
  {"uint16", "H", types::ScalarKind::kUnsigned, kDLUInt, 16},
  {"int32", "i", types::ScalarKind::kSigned, kDLInt, 32},
  {"uint32", "I", types::ScalarKind::kUnsigned, kDLUInt, 32},
  {"int64", "q", types::ScalarKind::kSigned, kDLInt, 64},
  {"uint64", "Q", types::ScalarKind::kUnsigned, kDLUInt, 64},
  {"float32", "f", types::ScalarKind::kFloat, kDLFloat, 32},
  {"float64", "d", types::ScalarKind::kFloat, kDLFloat, 64},
};

struct Payload
{
  std::shared_ptr<types::RootOwner> producer;
  std::shared_ptr<types::RootLeaseOwner> subscriber;
  bool acquired{false};

  ~Payload()
  {
    if (acquired) {producer->release_export();}
  }
};

#ifdef BUILD_TESTING
std::atomic<std::size_t> manager_count{0};
#endif

struct ManagedTensor
{
  DLManagedTensor managed{};
  std::vector<std::int64_t> shape;
  std::optional<Payload> payload{std::in_place};
  // One consumer reference, plus a capsule pin until its Python wrapper is destroyed.
  std::atomic<std::size_t> references{1};
  std::atomic<bool> released{false};

#ifdef BUILD_TESTING
  ManagedTensor() {++manager_count;}
  ~ManagedTensor()
  {
    --manager_count;
  }
#endif

  void release_reference() noexcept
  {
    if (references.fetch_sub(1, std::memory_order_acq_rel) == 1) {delete this;}
  }
  void release_storage() noexcept
  {
    // A consumer can call its deleter and then throw before renaming the capsule.
    // Its wrapper still pins this metadata, so subsequent cleanup is safe and idempotent.
    if (released.exchange(true, std::memory_order_acq_rel)) {return;}
    payload.reset();
    release_reference();
  }
};

std::unique_ptr<ManagedTensor> prepare(TensorType type, std::vector<std::int64_t> shape)
{
  if (shape.size() > static_cast<std::size_t>(INT32_MAX)) {
    throw std::invalid_argument("Tensor rank exceeds DLPack range");
  }
  std::int64_t product = 1;
  std::int64_t stride_volume = 1;
  for (auto dimension : shape) {
    // Zero elements still require representable contiguous strides in the consumer.
    const auto extent = dimension > 0 ? dimension : 1;
    if (dimension < 0 || stride_volume > INT64_MAX / extent) {
      throw std::invalid_argument("Tensor shape is negative or overflows int64");
    }
    stride_volume *= extent;
    product *= dimension;
  }
  if (static_cast<std::uint64_t>(product) != type.size ||
    type.size > static_cast<std::size_t>(PTRDIFF_MAX) / type.item_size)
  {
    throw std::invalid_argument("Tensor shape product must equal native field size");
  }
  auto result = std::make_unique<ManagedTensor>();
  result->shape = std::move(shape);
  result->managed.dl_tensor = {
    nullptr, {kDLCPU, 0}, static_cast<std::int32_t>(result->shape.size()), type.dtype,
    result->shape.data(), nullptr, 0};
  result->managed.manager_ctx = result.get();
  result->managed.deleter = [](DLManagedTensor * value) {
      static_cast<ManagedTensor *>(value->manager_ctx)->release_storage();
    };
  return result;
}

DLManagedTensor * finish(std::unique_ptr<ManagedTensor> value, types::BufferSpan span, TensorType type)
{
  if (span.size != type.size || span.item_size != type.item_size ||
    !span.format || std::strcmp(span.format, type.format) != 0 || (span.size && !span.data) ||
    (reinterpret_cast<std::uintptr_t>(span.data) % type.item_size) != 0)
  {
    throw std::invalid_argument("Tensor storage has inconsistent metadata or alignment");
  }
  // The DLPack empty-array contract uses null data, never a writable dummy allocation.
  value->managed.dl_tensor.data = span.size ? span.data : nullptr;
  auto result = &value->managed;
  value.release();
  return result;
}
}  // namespace

void retain_capsule(DLManagedTensor * managed) noexcept
{
  static_cast<ManagedTensor *>(managed->manager_ctx)->references.fetch_add(
    1, std::memory_order_relaxed);
}

void release_capsule(DLManagedTensor * managed, bool unconsumed) noexcept
{
  auto * manager = static_cast<ManagedTensor *>(managed->manager_ctx);
  if (unconsumed) {manager->release_storage();}
  manager->release_reference();
}

#ifdef BUILD_TESTING
std::size_t live_managers() noexcept {return manager_count.load();}
#endif

TensorType describe(const types::FieldInfo & info)
{
  using types::FieldKind;
  const auto & field = *info.ops;
  if ((field.kind != FieldKind::kArray && field.kind != FieldKind::kSequence) ||
    !field.element || field.element->kind != FieldKind::kScalar)
  {
    throw types::UnsupportedOperation("Tensor requires a numeric array or sequence field");
  }
  if (field.kind == FieldKind::kSequence && field.element->scalar == types::ScalarKind::kBool) {
    throw types::UnsupportedOperation("boolean sequences do not support Tensor export");
  }
  const auto & element = *field.element;
  for (const auto & candidate : supported) {
    if (element.format && std::strcmp(element.format, candidate.format) == 0 &&
      element.item_size == candidate.bits / 8 && element.scalar == candidate.kind)
    {
      return {candidate.name, {candidate.code, candidate.bits, 1}, info.size,
        element.item_size, element.format};
    }
  }
  throw types::UnsupportedOperation("Tensor dtype must have a supported native-endian format");
}

DLManagedTensor * export_dlpack(
  std::shared_ptr<types::RootOwner> root, const types::FieldPath & path,
  std::uint64_t generation, std::vector<std::int64_t> shape)
{
  const auto type = describe(root->info(path, generation));
  auto result = prepare(type, std::move(shape));
  auto & payload = *result->payload;
  payload.producer = std::move(root);
  const auto span = payload.producer->acquire_export(path, generation);
  payload.acquired = true;
  return finish(std::move(result), span, type);
}

DLManagedTensor * export_dlpack(
  std::shared_ptr<types::RootLeaseOwner> root, const types::FieldPath & path,
  std::uint64_t generation, std::vector<std::int64_t> shape)
{
  const auto type = describe(root->info(path, generation));
  auto result = prepare(type, std::move(shape));
  auto & payload = *result->payload;
  payload.subscriber = std::move(root);
  const auto span = payload.subscriber->foreign_readonly_buffer(path, generation);
  return finish(std::move(result), span, type);
}
}  // namespace tensor
}  // namespace rclpy_agnocast
