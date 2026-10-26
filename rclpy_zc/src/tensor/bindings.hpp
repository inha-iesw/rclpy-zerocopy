// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef TENSOR__BINDINGS_HPP_
#define TENSOR__BINDINGS_HPP_
#include <pybind11/pybind11.h>
#include "export.hpp"

namespace rclpy_agnocast
{
namespace tensor
{
template<typename Root>
pybind11::dict tensor_info(
  const Root & root, const types::FieldPath & path, std::uint64_t generation)
{
  const auto type = describe(root.info(path, generation));
  pybind11::dict result;
  result["dtype"] = type.name;
  result["size"] = type.size;
  result["item_size"] = type.item_size;
  return result;
}
pybind11::capsule producer_capsule(
  std::shared_ptr<types::RootOwner> root, const types::FieldPath & path,
  std::uint64_t generation, pybind11::handle shape);
pybind11::capsule subscriber_capsule(
  std::shared_ptr<types::RootLeaseOwner> root, const types::FieldPath & path,
  std::uint64_t generation, pybind11::handle shape);
void discard_capsule(pybind11::handle capsule);
}  // namespace tensor
}  // namespace rclpy_agnocast
#endif  // TENSOR__BINDINGS_HPP_
