// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef RCLPY_AGNOCAST__TYPES__FIELD_OPS_HPP_
#define RCLPY_AGNOCAST__TYPES__FIELD_OPS_HPP_

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>
#include "rclpy_agnocast/types/operations.hpp"

namespace rclpy_agnocast
{
namespace types
{
/**
 * @brief Copy provider-local error text without allocation.
 * @param[in] code Error category.
 * @param[in] text Non-null borrowed diagnostic, truncated to fit the owned result.
 * @return Owned status.
 */
inline Status error_status(ErrorCode code, const char * text) noexcept
{
  Status result;
  result.code = code;
  std::strncpy(result.message, text, sizeof(result.message) - 1);
  return result;
}

/** @brief Translate the active exception inside a provider catch block. @return Owned status. */
inline Status status_from_exception() noexcept
{
  try {
    throw;
  } catch (const std::bad_alloc &) {
    return error_status(ErrorCode::kMemory, "native allocation failed");
  } catch (const std::out_of_range & error) {
    return error_status(ErrorCode::kIndex, error.what());
  } catch (const std::length_error & error) {
    return error_status(ErrorCode::kValue, error.what());
  } catch (const std::invalid_argument & error) {
    return error_status(ErrorCode::kValue, error.what());
  } catch (const std::exception & error) {
    return error_status(ErrorCode::kRuntime, error.what());
  } catch (...) {
    return error_status(ErrorCode::kRuntime, "unknown native provider error");
  }
}

namespace field_detail
{
template<typename F>
Status protect(F && function) noexcept
{
  try {
    function(); return {};
  } catch (...) {
    return status_from_exception();
  }
}
inline std::size_t add(std::size_t a, std::size_t b)
{
  if (b > std::numeric_limits<std::size_t>::max() - a) {
    throw std::length_error("accounted storage overflow");
  }
  return a + b;
}
inline std::size_t multiply(std::size_t a, std::size_t b)
{
  if (b && a > std::numeric_limits<std::size_t>::max() / b) {
    throw std::length_error("accounted storage overflow");
  }
  return a * b;
}
inline Status zero_bytes(FieldRef, std::size_t * bytes) noexcept {*bytes = 0; return {};}

template<typename T>
constexpr ScalarKind scalar_kind()
{
  if constexpr (std::is_same_v<T, bool>) {return ScalarKind::kBool;}
  if constexpr (std::is_floating_point_v<T>) {return ScalarKind::kFloat;}
  if constexpr (std::is_signed_v<T>) {return ScalarKind::kSigned;}
  return ScalarKind::kUnsigned;
}
template<typename T>
const char * format()
{
  if constexpr (std::is_same_v<T, bool>) {return "?";}
  if constexpr (std::is_floating_point_v<T>) {return sizeof(T) == 4 ? "f" : "d";}
  if constexpr (std::is_signed_v<T>) {
    return sizeof(T) == 1 ? "b" : sizeof(T) == 2 ? "h" : sizeof(T) == 4 ? "i" : "q";
  }
  return sizeof(T) == 1 ? "B" : sizeof(T) == 2 ? "H" : sizeof(T) == 4 ? "I" : "Q";
}
template<typename T>
Status read_scalar(FieldRef ref, ScalarValue * value) noexcept
{
  const auto input = *static_cast<T *>(ref.object);
  if constexpr (std::is_same_v<T, bool>) {
    value->boolean = input;
  } else if constexpr (std::is_floating_point_v<T>) {
    value->real = input;
  } else if constexpr (std::is_signed_v<T>) {value->signed_integer = input;} else {
    value->unsigned_integer = input;
  }
  return {};
}
template<typename T>
Status write_scalar(FieldRef ref, ScalarValue value) noexcept
{
  if constexpr (std::is_same_v<T, bool>) {
    *static_cast<T *>(ref.object) = value.boolean;
  } else if constexpr (std::is_floating_point_v<T>) {
    if (std::isfinite(value.real) && std::abs(value.real) > std::numeric_limits<T>::max()) {
      return error_status(ErrorCode::kValue, "floating point value exceeds native range");
    }
    *static_cast<T *>(ref.object) = static_cast<T>(value.real);
  } else if constexpr (std::is_signed_v<T>) {
    if (value.signed_integer < std::numeric_limits<T>::min() ||
      value.signed_integer > std::numeric_limits<T>::max())
    {
      return error_status(ErrorCode::kValue, "integer exceeds native range");
    }
    *static_cast<T *>(ref.object) = static_cast<T>(value.signed_integer);
  } else {
    if (value.unsigned_integer > std::numeric_limits<T>::max()) {
      return error_status(ErrorCode::kValue, "integer exceeds native range");
    }
    *static_cast<T *>(ref.object) = static_cast<T>(value.unsigned_integer);
  }
  return {};
}
template<typename C>
constexpr bool packed_bool = std::is_same_v<typename C::value_type, bool>&&
  !std::is_reference_v<typename C::reference>;

template<typename C>
Status container_size(FieldRef ref, std::size_t * size) noexcept
{*size = static_cast<C *>(ref.object)->size(); return {};}

template<typename C>
Status container_child(FieldRef ref, std::size_t index, FieldRef * result) noexcept
{
  auto & value = *static_cast<C *>(ref.object);
  if (index >= value.size()) {return error_status(ErrorCode::kIndex, "field index out of range");}
  if constexpr (packed_bool<C>) {
    static const FieldOps element = [] {
        FieldOps ops{FieldKind::kScalar};
        ops.scalar = ScalarKind::kBool;
        ops.item_size = sizeof(bool);
        ops.read = [] (FieldRef item, ScalarValue * output) noexcept->Status {
          output->boolean = (*static_cast<C *>(item.object))[item.index]; return {};
        };
        ops.write = [] (FieldRef item, ScalarValue input) noexcept->Status {
          (*static_cast<C *>(item.object))[item.index] = input.boolean; return {};
        };
        ops.dynamic_bytes = zero_bytes;
        return ops;
      }();
    *result = {ref.object, &element, index};
  } else {
    *result = {&value[index], ref.ops->element, 0};
  }
  return {};
}
template<typename C>
Status container_buffer(FieldRef ref, BufferSpan * result) noexcept
{
  if constexpr (packed_bool<C>) {
    return error_status(ErrorCode::kUnsupported, "packed boolean sequence has no buffer");
  } else {
    if (ref.ops->element->kind != FieldKind::kScalar) {
      return error_status(ErrorCode::kUnsupported, "only scalar containers expose buffers");
    }
    auto & value = *static_cast<C *>(ref.object);
    *result = {value.empty() ? nullptr : std::addressof(value[0]), value.size(),
      ref.ops->element->item_size, ref.ops->element->format};
    return {};
  }
}
template<typename C>
Status container_resize(FieldRef ref, std::size_t size) noexcept
{
  if (size > ref.ops->bound) {return error_status(ErrorCode::kValue, "container bound exceeded");}
  return protect([&] {static_cast<C *>(ref.object)->resize(size);});
}
template<typename C>
Status nested_bytes(FieldRef ref, std::size_t * result) noexcept
{
  auto & value = *static_cast<C *>(ref.object);
  std::size_t total = 0;
  if constexpr (!packed_bool<C>) {
    if (ref.ops->element->kind != FieldKind::kScalar) {
      for (auto & child : value) {
        std::size_t bytes = 0;
        const auto status = ref.ops->element->dynamic_bytes({&child, ref.ops->element}, &bytes);
        if (status.code != ErrorCode::kOk) {return status;}
        if (bytes > std::numeric_limits<std::size_t>::max() - total) {
          return error_status(ErrorCode::kValue, "accounted storage overflow");
        }
        total += bytes;
      }
    }
  }
  *result = total;
  return {};
}
template<typename C>
FieldOps container_ops(const FieldOps * element)
{
  FieldOps ops{FieldKind::kArray};
  ops.item_size = sizeof(C);
  ops.element = element;
  ops.size = container_size<C>;
  ops.child = container_child<C>;
  ops.buffer = container_buffer<C>;
  ops.dynamic_bytes = nested_bytes<C>;
  return ops;
}
}  // namespace field_detail

/** @brief Return process-lifetime typed scalar operations. @return Immutable table. */
template<typename T>
const FieldOps * scalar_ops()
{
  static_assert(
    std::is_arithmetic_v<T>&& sizeof(T) <= 8,
    "typed fields support native bool, 8/16/32/64-bit integers and 32/64-bit floats");
  static const FieldOps ops = [] {
      FieldOps result{FieldKind::kScalar};
      result.scalar = field_detail::scalar_kind<T>();
      result.item_size = sizeof(T);
      result.format = field_detail::format<T>();
      result.read = field_detail::read_scalar<T>;
      result.write = field_detail::write_scalar<T>;
      result.dynamic_bytes = field_detail::zero_bytes;
      return result;
    }();
  return &ops;
}

/**
 * @brief Build typed fixed-array access.
 * @param[in] element Immutable operations retained for all root lifetimes.
 * @return Table value.
 */
template<typename C>
FieldOps array_ops(const FieldOps * element)
{return field_detail::container_ops<C>(element);}

/**
 * @brief Build a native sequence table with an IDL element-count bound.
 * @param[in] element Immutable operations retained for all root lifetimes.
 * @param[in] bound Maximum number of elements from IDL.
 * @return Table value.
 */
template<typename C>
FieldOps sequence_ops(
  const FieldOps * element,
  std::size_t bound = std::numeric_limits<std::size_t>::max())
{
  auto ops = field_detail::container_ops<C>(element);
  ops.kind = FieldKind::kSequence;
  ops.bound = bound;
  ops.resize = field_detail::container_resize<C>;
  ops.minimum_resize_bytes = [] (FieldRef, std::size_t size, std::size_t * bytes) noexcept->Status {
    return field_detail::protect(
      [&] {
        if constexpr (field_detail::packed_bool<C>) {
          *bytes = size / 8 + (size % 8 != 0);
        } else {
          *bytes = field_detail::multiply(size, sizeof(typename C::value_type));
        }
      });
  };
  ops.dynamic_bytes = [] (FieldRef ref, std::size_t * bytes) noexcept->Status {
    std::size_t nested = 0;
    const auto status = field_detail::nested_bytes<C>(ref, &nested);
    if (status.code != ErrorCode::kOk) {return status;}
    return field_detail::protect(
      [&] {
        const auto capacity = static_cast<C *>(ref.object)->capacity();
        std::size_t storage;
        if constexpr (field_detail::packed_bool<C>) {
          storage = capacity / 8;
          storage += capacity % 8 != 0;
        } else {
          storage = field_detail::multiply(capacity, sizeof(typename C::value_type));
        }
        *bytes = field_detail::add(storage, nested);
      });
  };
  return ops;
}

/**
 * @brief Build string operations using native code units; decoding is a separate explicit copy.
 * @param[in] bound Maximum code units from IDL, excluding the terminator.
 * @return Table accounting external capacity plus terminator, excluding inline SSO storage.
 */
template<typename C>
FieldOps string_ops(std::size_t bound = std::numeric_limits<std::size_t>::max())
{
  using Unit = typename C::value_type;
  using Scalar = std::conditional_t<sizeof(Unit) == 1, std::uint8_t, Unit>;
  auto ops = field_detail::container_ops<C>(scalar_ops<Scalar>());
  ops.kind = sizeof(Unit) == 1 ? FieldKind::kString : FieldKind::kWString;
  ops.bound = bound;
  ops.resize = field_detail::container_resize<C>;
  ops.minimum_resize_bytes =
    [] (FieldRef ref, std::size_t size, std::size_t * bytes) noexcept->Status {
    return field_detail::protect(
      [&] {
        const auto & value = *static_cast<C *>(ref.object);
        const auto data = reinterpret_cast<std::uintptr_t>(value.data());
        const auto object = reinterpret_cast<std::uintptr_t>(&value);
        const bool inline_storage = data >= object && data - object < sizeof(C);
        *bytes = inline_storage && size <= value.capacity() ? 0 :
        field_detail::multiply(field_detail::add(size, 1), sizeof(Unit));
      });
  };
  ops.dynamic_bytes = [] (FieldRef ref, std::size_t * bytes) noexcept->Status {
    return field_detail::protect(
      [&] {
        const auto & value = *static_cast<C *>(ref.object);
        const auto data = reinterpret_cast<std::uintptr_t>(value.data());
        const auto object = reinterpret_cast<std::uintptr_t>(&value);
        *bytes = data >= object && data - object < sizeof(C) ? 0 :
        field_detail::multiply(field_detail::add(value.capacity(), 1), sizeof(Unit));
      });
  };
  return ops;
}

/**
 * @brief Build native member access from generated member pointers.
 * @param[in] members Immutable process-lifetime members; null only for an empty message.
 * @param[in] count Number of members.
 * @return Table value.
 */
template<typename T>
FieldOps message_ops(const FieldMember * members, std::size_t count)
{
  FieldOps ops{FieldKind::kMessage};
  ops.item_size = sizeof(T);
  ops.members = members;
  ops.member_count = count;
  ops.child = [] (FieldRef ref, std::size_t index, FieldRef * result) noexcept->Status {
    if (index >= ref.ops->member_count) {return error_status(ErrorCode::kIndex, "unknown member");}
    const auto & member = ref.ops->members[index];
    *result = {member.get(ref.object), member.ops};
    return {};
  };
  ops.dynamic_bytes = [] (FieldRef ref, std::size_t * bytes) noexcept->Status {
    std::size_t total = 0;
    for (std::size_t i = 0; i < ref.ops->member_count; ++i) {
      const auto & member = ref.ops->members[i];
      std::size_t extra = 0;
      const auto status = member.ops->dynamic_bytes({member.get(ref.object), member.ops}, &extra);
      if (status.code != ErrorCode::kOk) {return status;}
      if (extra > std::numeric_limits<std::size_t>::max() - total) {
        return error_status(ErrorCode::kValue, "accounted storage overflow");
      }
      total += extra;
    }
    *bytes = total;
    return {};
  };
  return ops;
}

}  // namespace types
}  // namespace rclpy_agnocast
#endif  // RCLPY_AGNOCAST__TYPES__FIELD_OPS_HPP_
