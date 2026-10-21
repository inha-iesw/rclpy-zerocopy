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

#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "agnocast/agnocast_executor.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"

namespace agnocast
{
class Node;
}

namespace rclpy_agnocast
{
/** @brief Serialize endpoint controls with callbacks in the existing Agnocast spin loop. */
class SubscriberExecutor final : public agnocast::AgnocastOnlySingleThreadedExecutor
{
public:
  /** @brief Register a manual control group and start the native executor thread. */
  explicit SubscriberExecutor(const std::shared_ptr<agnocast::Node> & node);
  ~SubscriberExecutor();
  /** @brief Execute synchronously on the worker; reject worker reentry and stopped execution. */
  void invoke(std::function<void()> operation);
  /** @brief Run cleanup on the worker, or join exited spin and run unstarted cleanup here. */
  void invoke_or_join(std::function<void()> operation);
  /** @brief Remove a manually added group and discard its prefetched callbacks. */
  void detach(const rclcpp::CallbackGroup::SharedPtr & group);
  /** @brief Cancel, join and destroy pending tasks so all waiting controls become ready. */
  void stop() noexcept;
  /** @brief Rethrow a failure escaping the native spin loop. */
  void check_failure() const;
#ifdef BUILD_TESTING
  /** @brief End spin without changing manager state, optionally through a native exception. */
  void test_stop(bool fail);
  /** @brief End spin after the next control is queued but before it starts. */
  void test_fail_next_control();
#endif

private:
  void run() noexcept;
  void throw_if_stopped_locked() const;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  mutable std::mutex control_mutex_;
  bool stopped_{false};
  std::exception_ptr failure_;
  std::thread thread_;
#ifdef BUILD_TESTING
  bool fail_next_control_{false};
#endif
};
}  // namespace rclpy_agnocast
