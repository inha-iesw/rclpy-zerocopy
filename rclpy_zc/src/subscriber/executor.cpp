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

#include "executor.hpp"

#include <algorithm>
#include <future>
#include <stdexcept>
#include <utility>

#include "queue.hpp"
#include "agnocast/node/agnocast_node.hpp"

namespace rclpy_agnocast
{
SubscriberExecutor::SubscriberExecutor(const std::shared_ptr<agnocast::Node> & node)
: control_group_(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false))
{
  add_callback_group(control_group_, node->get_node_base_interface());
  thread_ = std::thread([this]() {run();});
}

SubscriberExecutor::~SubscriberExecutor() {stop();}

void SubscriberExecutor::invoke(std::function<void()> operation)
{
  if (thread_.get_id() == std::this_thread::get_id()) {
    throw std::logic_error("subscriber executor cannot synchronously reenter its worker");
  }
  auto task = std::make_shared<std::packaged_task<void()>>(std::move(operation));
  auto result = task->get_future();
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    throw_if_stopped_locked();
    auto callable = std::make_shared<std::function<void()>>([task]() {(*task)();});
    std::lock_guard<std::mutex> ready_lock(ready_agnocast_executables_mutex_);
    // Controls precede prefetched data so a hot subscription cannot starve close().
    ready_agnocast_executables_.insert(
      ready_agnocast_executables_.begin(), {std::move(callable), control_group_});
#ifdef BUILD_TESTING
    if (fail_next_control_) {
      fail_next_control_ = false;
      auto failure = std::make_shared<std::function<void()>>(
        []() {
          throw std::runtime_error("injected subscriber spin failure");
        });
      ready_agnocast_executables_.insert(
        ready_agnocast_executables_.begin(), {std::move(failure), control_group_});
    }
#endif
  }
  // The queued callable is the sole promise owner. If spin fails before it runs, clearing
  // the ready queue produces broken_promise instead of leaving this caller blocked.
  task.reset();
  result.get();
}

void SubscriberExecutor::invoke_or_join(std::function<void()> operation)
{
  bool started = false;
  try {
    invoke([&]() {started = true; operation();});
    return;
  } catch (...) {
    // future completion synchronizes started. Never replay an operation's own error (e.g. busy).
    if (started) {throw;}
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (!stopped_) {throw;}
  }
  // The manager serializes controls and join. No callback can retain an endpoint after join.
  if (thread_.joinable()) {thread_.join();}
  operation();
}

void SubscriberExecutor::detach(const rclcpp::CallbackGroup::SharedPtr & group)
{
  remove_callback_group(group);
  std::lock_guard<std::mutex> lock(ready_agnocast_executables_mutex_);
  auto & ready = ready_agnocast_executables_;
  ready.erase(
    std::remove_if(
      ready.begin(), ready.end(), [&group](const auto & item) {
        return item.callback_group == group;
      }), ready.end());
}

void SubscriberExecutor::run() noexcept
{
  std::exception_ptr failure;
  try {
    spin();
  } catch (...) {
    failure = std::current_exception();
  }
  std::lock_guard<std::mutex> lock(control_mutex_);
  stopped_ = true;
  failure_ = failure;
  {
    std::lock_guard<std::mutex> ready_lock(ready_agnocast_executables_mutex_);
    ready_agnocast_executables_.clear();
  }
  // Never acquire the manager lock here: its close path holds it while joining this worker.
  const auto terminal = failure ? failure :
    std::make_exception_ptr(std::runtime_error("subscriber executor is stopped"));
  for (const auto & weak : queues_) {
    if (auto queue = weak.lock()) {queue->fail(terminal);}
  }
}

void SubscriberExecutor::watch_queue(const std::shared_ptr<SubscriberQueue> & queue)
{
  std::lock_guard<std::mutex> lock(control_mutex_);
  throw_if_stopped_locked();
  queues_.erase(
    std::remove_if(queues_.begin(), queues_.end(), [](const auto & weak) {return weak.expired();}),
    queues_.end());
  queues_.push_back(queue);
}

void SubscriberExecutor::stop() noexcept
{
  cancel();
  if (thread_.joinable()) {thread_.join();}
}

void SubscriberExecutor::check_failure() const
{
  std::lock_guard<std::mutex> lock(control_mutex_);
  throw_if_stopped_locked();
}

void SubscriberExecutor::throw_if_stopped_locked() const
{
  if (failure_) {std::rethrow_exception(failure_);}
  if (stopped_) {throw std::runtime_error("subscriber executor is stopped");}
}

#ifdef BUILD_TESTING
void SubscriberExecutor::test_fail_next_control()
{
  std::lock_guard<std::mutex> lock(control_mutex_);
  fail_next_control_ = true;
}

void SubscriberExecutor::test_stop(const bool fail)
{
  if (fail) {
    test_fail_next_control();
    try {
      invoke([]() {});
    } catch (const std::future_error &) {
      // run() has discarded the unexecuted control and fulfilled its broken promise.
    }
  } else {
    cancel();
  }
  if (thread_.joinable()) {thread_.join();}
}
#endif
}  // namespace rclpy_agnocast
