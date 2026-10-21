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
#include <string>

#include "queue.hpp"

namespace agnocast
{
class Node;
}

namespace rclpy_agnocast
{
class SubscriptionState;
class SubscriptionHandle;

#ifdef BUILD_TESTING
/** @brief Actual typed receive/reset observations; a reset is counted after completion. */
struct SubscriberObservation
{
  std::uint64_t received{0};
  std::uint64_t released{0};
  std::int64_t callback_tid{0};
  std::int64_t release_tid{0};
  std::uintptr_t native_data_address{0};
};
#endif

/** @brief Own the native executor and endpoints without retaining Python handles. */
class SubscriberRuntime : public std::enable_shared_from_this<SubscriberRuntime>
{
public:
  /** @brief Retain the node and its initialized native session for endpoint lifetime. */
  SubscriberRuntime(std::shared_ptr<agnocast::Node> node, std::shared_ptr<void> native_session);
  ~SubscriberRuntime();
  /** @brief Create a bounded typed subscription on the native executor thread. */
  std::shared_ptr<SubscriptionHandle> create_subscription(
    const std::string & topic, std::size_t depth, std::size_t max_count, std::size_t max_bytes);
  /** @brief Stop admission without destroying endpoints, stopping execution or draining queues. */
  void quiesce() noexcept;
  /** @brief Stop execution and drain queues; return false while delivered storage is alive. */
  bool close();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  friend class SubscriptionHandle;
};

/** @brief A handle pins its manager; delivered storage separately pins its endpoint/session. */
class SubscriptionHandle
{
public:
  /** @brief Best-effort close; live leases retain native resources if close cannot finish. */
  ~SubscriptionHandle();
  /** @brief Return the oldest queued lease, or null; callback failures are rethrown here. */
  std::shared_ptr<LeaseState> take();
  /** @brief Read queued, delivered and exported storage accounting under one lock. */
  SubscriberStats stats() const;
  /** @brief Drain and detach this endpoint; throw a retryable error while a Lease is alive. */
  void close();
  /** @brief Return the native graph count; reject access after closing begins. */
  std::size_t publisher_count();
#ifdef BUILD_TESTING
  /** @brief Read actual receive/reset evidence; available only in test builds. */
  SubscriberObservation observation() const;
  /** @brief Stop the worker independently of runtime close, for lifecycle regression tests. */
  void test_stop_executor(bool fail);
  /** @brief Inject a spin failure ahead of the next queued control. */
  void test_fail_next_control();
#endif

private:
  SubscriptionHandle(
    std::shared_ptr<SubscriberRuntime> runtime, std::shared_ptr<SubscriptionState> state);
  std::shared_ptr<SubscriberRuntime> runtime_;
  std::shared_ptr<SubscriptionState> state_;
  friend class SubscriberRuntime;
};
}  // namespace rclpy_agnocast
