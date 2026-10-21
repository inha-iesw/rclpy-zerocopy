// Copyright 2026 rclpy zerocopy maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

namespace rclpy_agnocast
{
struct SubscriberBudget;
class SubscriberQueue;

/** @brief Immutable payload span with ownership of its original native storage. */
struct ReceivedBuffer
{
  std::uint64_t sequence;
  const std::uint8_t * data;
  std::size_t size;
  std::shared_ptr<void> storage;
};

/** @brief Keep native storage and its admission reservation until the last buffer alias dies. */
class LeaseBufferOwner
{
public:
  LeaseBufferOwner(ReceivedBuffer buffer, std::shared_ptr<SubscriberBudget> budget);
  ~LeaseBufferOwner();
  LeaseBufferOwner(const LeaseBufferOwner &) = delete;
  LeaseBufferOwner & operator=(const LeaseBufferOwner &) = delete;
  const ReceivedBuffer & buffer() const noexcept {return buffer_;}

private:
  // A delivered owner pins the endpoint/session; queued owners must not form an endpoint cycle.
  std::shared_ptr<void> keepalive_;
  ReceivedBuffer buffer_;
  std::shared_ptr<SubscriberBudget> budget_;
  bool reserved_{false};
  friend class SubscriberQueue;
};

/** @brief Independently releasable wrapper; exported buffers retain the owner directly. */
class LeaseState
{
public:
  explicit LeaseState(std::shared_ptr<LeaseBufferOwner> owner);
  /** @brief Return the live owner, or throw std::runtime_error after release. */
  std::shared_ptr<LeaseBufferOwner> owner() const;
  /** @brief Invalidate the wrapper; repeated release is harmless and existing views survive. */
  void release() noexcept;

private:
  mutable std::mutex mutex_;
  std::shared_ptr<LeaseBufferOwner> owner_;
};
}  // namespace rclpy_agnocast
