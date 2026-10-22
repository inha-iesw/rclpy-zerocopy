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
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <vector>

#include "lease.hpp"

namespace rclpy_agnocast
{
/** @brief One locked snapshot; outstanding includes queued, delivered and exported storage. */
struct SubscriberStats
{
  std::uint64_t received_total{0};
  std::uint64_t delivered_total{0};
  std::uint64_t dropped_outstanding{0};
  std::uint64_t dropped_oversize{0};
  std::size_t queued_leases{0};
  std::size_t outstanding_leases{0};
  std::size_t outstanding_payload_bytes{0};
};

struct SubscriberBudget
{
  std::mutex mutex;
  SubscriberStats stats;
};

/** @brief Preallocated DropNewest ring; popping does not return count or byte reservations. */
class SubscriberQueue
{
public:
  SubscriberQueue(std::size_t max_count, std::size_t max_bytes);
  ~SubscriberQueue();
  SubscriberQueue(const SubscriberQueue &) = delete;
  SubscriberQueue & operator=(const SubscriberQueue &) = delete;

  /** @brief Move storage into the ring, or release it without waiting for capacity. */
  bool push(ReceivedBuffer buffer);
  /** @brief Pop a wrapper, pinning endpoint resources only after it leaves the queue. */
  std::shared_ptr<LeaseState> take(std::shared_ptr<void> keepalive = {});
  /** @brief Wait for a newer notification; zero means explicit stop, failures are rethrown. */
  std::uint64_t wait_for_wake(std::uint64_t generation);
  /** @brief Disarm only an empty queue, or notify again after an interrupted drain. */
  void finish_dispatch() noexcept;
  /** @brief Stop admission and wake the notifier with a native receive/executor failure. */
  void fail(std::exception_ptr failure) noexcept;
  /** @brief Rethrow the first recorded failure, including after explicit stop or queue cleanup. */
  void rethrow_failure() const;
  /** @brief Permanently stop admission/notifications; wait returns zero, storage releasable. */
  void stop_admission() noexcept;
  /** @brief Release all queued storage outside the accounting lock. */
  void clear() noexcept;
  /** @brief Stop admission and drain queued storage; false means delivered owners still exist. */
  bool drain_for_close() noexcept;
  SubscriberStats stats() const noexcept;

private:
  const std::size_t max_bytes_;
  std::shared_ptr<SubscriberBudget> budget_;
  std::vector<std::shared_ptr<LeaseBufferOwner>> ring_;
  std::size_t head_{0};
  bool accepting_{true};
  bool armed_{false};
  bool notifications_stopped_{false};
  std::uint64_t generation_{0};
  std::exception_ptr failure_;
  std::condition_variable wake_;
};
}  // namespace rclpy_agnocast
