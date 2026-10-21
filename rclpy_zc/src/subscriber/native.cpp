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

#include "native.hpp"

#ifdef BUILD_TESTING
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "executor.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "rclpy_agnocast_msgs/msg/shared_buffer.hpp"

namespace rclpy_agnocast
{
namespace
{
using Message = rclpy_agnocast_msgs::msg::SharedBuffer;
using NativeMessage = agnocast::ipc_shared_ptr<const Message>;
using NativeSubscription = agnocast::Subscription<Message>;

#ifdef BUILD_TESTING
struct ObservationState
{
  std::mutex mutex;
  SubscriberObservation value;

  void received(const NativeMessage & message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    ++value.received;
    value.callback_tid = syscall(SYS_gettid);
    value.native_data_address = reinterpret_cast<std::uintptr_t>(message->data.data());
  }
  void released()
  {
    std::lock_guard<std::mutex> lock(mutex);
    ++value.released;
    value.release_tid = syscall(SYS_gettid);
  }
};
#endif

struct NativeStorage
{
  NativeMessage message;
#ifdef BUILD_TESTING
  std::shared_ptr<ObservationState> observation;
  ~NativeStorage()
  {
    message.reset();
    observation->released();
  }
#endif
};
}  // namespace

class SubscriptionState
{
public:
  SubscriptionState(std::shared_ptr<void> session, std::size_t count, std::size_t bytes)
  : native_session(std::move(session)), queue(count, bytes) {}

  void receive(NativeMessage message) noexcept
  {
#ifdef BUILD_TESTING
    observation->received(message);
#endif
    try {
      auto storage = std::make_shared<NativeStorage>();
#ifdef BUILD_TESTING
      storage->observation = observation;
#endif
      storage->message = std::move(message);
      const auto & value = *storage->message;
      queue.push({value.sequence, value.data.data(), value.data.size(), std::move(storage)});
    } catch (...) {
      auto failure = std::current_exception();
      // If holder allocation failed, the parameter still owns the original native reference.
      if (message) {
        message.reset();
#ifdef BUILD_TESTING
        observation->released();
#endif
      }
      queue.stop_admission();
      std::lock_guard<std::mutex> lock(failure_mutex);
      if (!callback_failure) {callback_failure = failure;}
    }
  }

  // Declaration order ensures queued pointers and endpoints die before their native session.
  std::shared_ptr<void> native_session;
  rclcpp::CallbackGroup::SharedPtr group;
  NativeSubscription::SharedPtr subscription;
  SubscriberQueue queue;
  bool attached{false};
  bool closing{false};
  bool closed{false};
  std::mutex failure_mutex;
  std::exception_ptr callback_failure;
#ifdef BUILD_TESTING
  std::shared_ptr<ObservationState> observation{std::make_shared<ObservationState>()};
#endif
};

class SubscriberRuntime::Impl
{
public:
  Impl(std::shared_ptr<agnocast::Node> value, std::shared_ptr<void> session)
  : native_session(std::move(session)), node(std::move(value)),
    executor(std::make_unique<SubscriberExecutor>(node)) {}

  void detach(const std::shared_ptr<SubscriptionState> & state)
  {
    if (state->attached) {
      executor->detach(state->group);
      state->attached = false;
    }
  }

  void release_endpoint(const std::shared_ptr<SubscriptionState> & state)
  {
    state->subscription.reset();
    state->group.reset();
    state->native_session.reset();
    state->closed = true;
  }

  std::shared_ptr<void> native_session;
  std::shared_ptr<agnocast::Node> node;
  std::mutex mutex;
  std::vector<std::shared_ptr<SubscriptionState>> states;
  std::unique_ptr<SubscriberExecutor> executor;
  bool accepting{true};
};

SubscriberRuntime::SubscriberRuntime(
  std::shared_ptr<agnocast::Node> node, std::shared_ptr<void> native_session)
: impl_(std::make_unique<Impl>(std::move(node), std::move(native_session))) {}

SubscriberRuntime::~SubscriberRuntime() {(void)close();}

std::shared_ptr<SubscriptionHandle> SubscriberRuntime::create_subscription(
  const std::string & topic, const std::size_t depth,
  const std::size_t max_count, const std::size_t max_bytes)
{
  // The handle's rollback destructor must run after the manager lock is released.
  std::unique_ptr<SubscriptionHandle> handle;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->accepting) {throw std::runtime_error("subscriber runtime is closing");}
  auto state = std::make_shared<SubscriptionState>(impl_->native_session, max_count, max_bytes);
  handle.reset(new SubscriptionHandle(shared_from_this(), state));
  impl_->states.push_back(state);
  try {
    impl_->executor->invoke(
      [this, state, topic, depth]() {
        try {
          state->group = impl_->node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
          agnocast::SubscriptionOptions options;
          options.callback_group = state->group;
          const std::weak_ptr<SubscriptionState> weak = state;
          state->subscription = impl_->node->create_subscription<Message>(
            topic, depth, [weak](NativeMessage message) noexcept {
            if (auto target = weak.lock()) {target->receive(std::move(message));}
          }, options);
          impl_->executor->add_callback_group(state->group, impl_->node->get_node_base_interface());
          state->attached = true;
        } catch (...) {
          state->subscription.reset();
          state->group.reset();
          throw;
        }
      });
  } catch (...) {
    impl_->states.pop_back();
    throw;
  }
  return std::shared_ptr<SubscriptionHandle>(std::move(handle));
}

void SubscriberRuntime::quiesce() noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->accepting = false;
  for (const auto & state : impl_->states) {
    state->queue.stop_admission();
  }
}

bool SubscriberRuntime::close()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->accepting = false;
  for (const auto & state : impl_->states) {
    state->closing = true;
    state->queue.stop_admission();
  }
  if (impl_->executor) {impl_->executor->stop();}
  bool idle = true;
  for (const auto & state : impl_->states) {
    impl_->detach(state);
    if (!state->queue.drain_for_close()) {idle = false;}
  }
  if (!idle) {return false;}
  for (const auto & state : impl_->states) {
    impl_->release_endpoint(state);
  }
  impl_->states.clear();
  impl_->executor.reset();
  impl_->node.reset();
  impl_->native_session.reset();
  return true;
}

SubscriptionHandle::SubscriptionHandle(
  std::shared_ptr<SubscriberRuntime> runtime, std::shared_ptr<SubscriptionState> state)
: runtime_(std::move(runtime)), state_(std::move(state)) {}

SubscriptionHandle::~SubscriptionHandle()
{
  try {
    close();
  } catch (...) {
    // A delivered owner retains its endpoint/session until the last alias is released.
  }
}

std::shared_ptr<LeaseState> SubscriptionHandle::take()
{
  auto & impl = *runtime_->impl_;
  std::lock_guard<std::mutex> lock(impl.mutex);
  if (!impl.accepting || state_->closing) {throw std::runtime_error("subscription is closing");}
  impl.executor->check_failure();
  {
    std::lock_guard<std::mutex> failure_lock(state_->failure_mutex);
    if (state_->callback_failure) {std::rethrow_exception(state_->callback_failure);}
  }
  return state_->queue.take(state_);
}

SubscriberStats SubscriptionHandle::stats() const {return state_->queue.stats();}

void SubscriptionHandle::close()
{
  auto & impl = *runtime_->impl_;
  std::lock_guard<std::mutex> lock(impl.mutex);
  if (state_->closed) {return;}
  state_->closing = true;
  state_->queue.stop_admission();
  auto close_endpoint = [&impl, this]() {
      if (!state_->queue.drain_for_close()) {
        const auto stats = state_->queue.stats();
        throw std::runtime_error(
                "subscription close: active Lease/view=" +
                std::to_string(stats.outstanding_leases) + ", payload_bytes=" +
                std::to_string(stats.outstanding_payload_bytes));
      }
      impl.detach(state_);
      impl.release_endpoint(state_);
    };
  impl.executor->invoke_or_join(close_endpoint);
  impl.states.erase(std::remove(impl.states.begin(), impl.states.end(), state_), impl.states.end());
}

std::size_t SubscriptionHandle::publisher_count()
{
  auto & impl = *runtime_->impl_;
  std::lock_guard<std::mutex> lock(impl.mutex);
  if (!impl.accepting || state_->closing) {throw std::runtime_error("subscription is closing");}
  std::size_t count = 0;
  impl.executor->invoke([this, &count]() {count = state_->subscription->get_publisher_count();});
  return count;
}

#ifdef BUILD_TESTING
void SubscriptionHandle::test_stop_executor(const bool fail)
{
  std::lock_guard<std::mutex> lock(runtime_->impl_->mutex);
  runtime_->impl_->executor->test_stop(fail);
}

void SubscriptionHandle::test_fail_next_control()
{
  std::lock_guard<std::mutex> lock(runtime_->impl_->mutex);
  runtime_->impl_->executor->test_fail_next_control();
}

SubscriberObservation SubscriptionHandle::observation() const
{
  std::lock_guard<std::mutex> lock(state_->observation->mutex);
  return state_->observation->value;
}
#endif
}  // namespace rclpy_agnocast
