// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef TYPES__NATIVE_PUBLISHER_HPP_
#define TYPES__NATIVE_PUBLISHER_HPP_

#include <cstddef>
#include <memory>
#include <string>

namespace rclpy_agnocast
{
class PublisherBackendFactory;
namespace types
{
struct TypeDescriptor;

/**
 * @brief Prepare generated endpoint configuration without creating native resources.
 * @param[in] node Non-null borrowed Agnocast node, retained by the worker's session owner.
 * @param[in] topic Resolved topic name copied before an active loan can start.
 * @param[in] depth Positive KeepLast depth.
 * @param[in] descriptor Registered metadata and process-pinned operations.
 * @return Factory whose create and endpoint operations run only on the publisher worker.
 */
std::unique_ptr<PublisherBackendFactory> native_publisher_factory(
  void * node, std::string topic, std::size_t depth,
  std::shared_ptr<const TypeDescriptor> descriptor);

}  // namespace types
}  // namespace rclpy_agnocast
#endif  // TYPES__NATIVE_PUBLISHER_HPP_
