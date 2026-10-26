// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef SUBSCRIBER__PROVENANCE_HPP_
#define SUBSCRIBER__PROVENANCE_HPP_
#include <cstddef>

namespace rclpy_agnocast
{
// Reject unknown/same-process origins and writable/private/non-Agnocast mappings.
// Both complete spans must belong to the same foreign publisher's shared-memory file.
void require_readonly_mapping(
  const void * root, std::size_t root_bytes, const void * data, std::size_t data_bytes);
}  // namespace rclpy_agnocast
#endif  // SUBSCRIBER__PROVENANCE_HPP_
