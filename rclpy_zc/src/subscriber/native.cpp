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
#include <limits>
#include <mutex>
#include <new>
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
    received(reinterpret_cast<std::uintptr_t>(message->data.data()));
  }
  void received(std::uintptr_t address)
  {
    std::lock_guard<std::mutex> lock(mutex);
    ++value.received;
    value.callback_tid = syscall(SYS_gettid);
    value.native_data_address = address;
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

struct TypedSubscriptionStorage
{
  types::NativeSubscriptionEndpoint endpoint;
  ~TypedSubscriptionStorage() {reset();}
  void reset() noexcept
  {
    if (endpoint.object) {endpoint.destroy(endpoint.object);}
    endpoint = {};
  }
};
}  // namespace

class SubscriptionState
{
public:
  SubscriptionState(
    std::shared_ptr<void> session, std::size_t count, std::size_t bytes,
    std::size_t max_message_bytes = 0,
    std::shared_ptr<const types::TypeDescriptor> type = {})
  : native_session(std::move(session)), descriptor(std::move(type)),
    queue(count, bytes, max_message_bytes) {}

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
      fail_receive(failure);
    }
  }

  void fail_receive(std::exception_ptr failure) noexcept
  {
    queue.fail(std::move(failure));
  }

  static void receive_typed(
    void * context, types::NativeReceived message, types::Status status) noexcept
  {
    auto & self = *static_cast<SubscriptionState *>(context);
    try {
      types::check_status(status);
#ifdef BUILD_TESTING
      self.observation->received(reinterpret_cast<std::uintptr_t>(message.root));
      auto release = [callback = message.release, observation = self.observation](void * owner) {
          callback(owner);
          observation->released();
        };
#else
      auto release = message.release;
#endif
      // shared_ptr calls release even if its control-block allocation fails.
      auto storage = std::shared_ptr<void>(message.owner, release);
#ifdef BUILD_TESTING
      // Exercise rollback after successful adoption, not shared_ptr allocator fault injection.
      if (self.fail_next_typed_receive) {
        self.fail_next_typed_receive = false;
        throw std::bad_alloc();
      }
#endif
      const auto & descriptor = *self.descriptor;
      std::size_t dynamic = 0;
      types::check_status(
        descriptor.ops->root->dynamic_bytes(
          {const_cast<void *>(message.root), descriptor.ops->root}, &dynamic));
      if (dynamic > std::numeric_limits<std::size_t>::max() - descriptor.native_size) {
        throw std::invalid_argument("accounted storage overflow");
      }
      ReceivedBuffer value{++self.identity, nullptr, descriptor.native_size + dynamic,
        std::move(storage), message.root, self.descriptor};
      self.queue.push(std::move(value));
    } catch (...) {
      self.fail_receive(std::current_exception());
    }
  }

  // Declaration order ensures queued pointers and endpoints die before their native session.
  std::shared_ptr<void> native_session;
  rclcpp::CallbackGroup::SharedPtr group;
  NativeSubscription::SharedPtr subscription;
  TypedSubscriptionStorage typed_subscription;
  std::shared_ptr<const types::TypeDescriptor> descriptor;
  SubscriberQueue queue;
  std::uint64_t identity{0};
  bool attached{false};
  bool closing{false};
  bool closed{false};
#ifdef BUILD_TESTING
  std::shared_ptr<ObservationState> observation{std::make_shared<ObservationState>()};
  bool fail_next_typed_receive{false};
#endif
};

class SubscriberRuntime::Impl
{
public:
  Impl(std::shared_ptr<agnocast::Node> value, std::shared_ptr<void> session)
  : native_session(std::move(session)), node(std::move(value)),
    executor(std::make_unique<SubscriberExecutor>(node)) {}

  void attach(
    const std::shared_ptr<SubscriptionState> & state,
    const std::string & topic, const std::size_t depth)
  {
    executor->watch_queue(std::shared_ptr<SubscriberQueue>(state, &state->queue));
    executor->invoke(
      [this, state, topic, depth]() {
        try {
          state->group = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
          if (state->descriptor) {
            types::check_status(
              state->descriptor->ops->subscribe(
                node.get(), topic.c_str(), depth, &state->group, state.get(),
                &SubscriptionState::receive_typed, &state->typed_subscription.endpoint));
          } else {
            agnocast::SubscriptionOptions options;
            options.callback_group = state->group;
            const std::weak_ptr<SubscriptionState> weak = state;
            state->subscription = node->create_subscription<Message>(
              topic, depth, [weak](NativeMessage message) noexcept {
              if (auto target = weak.lock()) {target->receive(std::move(message));}
            }, options);
          }
          executor->add_callback_group(state->group, node->get_node_base_interface());
          state->attached = true;
        } catch (...) {
          state->subscription.reset();
          state->typed_subscription.reset();
          state->group.reset();
          throw;
        }
      });
  }

  // Callers hold mutex; keep endpoint closing ahead of executor/receive failures.
  void require_open(const SubscriptionState & state) const
  {
    if (!accepting || state.closing) {throw std::runtime_error("subscription is closing");}
  }

  bool begin_close(SubscriptionState & state)
  {
    if (state.closed) {return false;}
    state.closing = true;
    state.queue.stop_admission();
    return true;
  }

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
    state->typed_subscription.reset();
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
  return create_typed_subscription({}, topic, depth, max_count, max_bytes, 0);
}

std::shared_ptr<SubscriptionHandle> SubscriberRuntime::create_typed_subscription(
  std::shared_ptr<const types::TypeDescriptor> descriptor, const std::string & topic,
  const std::size_t depth, const std::size_t max_count, const std::size_t max_bytes,
  const std::size_t max_message_bytes)
{
  // The handle's rollback destructor must run after the manager lock is released.
  std::unique_ptr<SubscriptionHandle> handle;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->accepting) {throw std::runtime_error("subscriber runtime is closing");}
  auto state = std::make_shared<SubscriptionState>(
    impl_->native_session, max_count, max_bytes, max_message_bytes, std::move(descriptor));
  handle.reset(new SubscriptionHandle(shared_from_this(), state));
  impl_->states.push_back(state);
  try {
    impl_->attach(state, topic, depth);
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

void SubscriberRuntime::prepare_close()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->accepting = false;
  for (const auto & state : impl_->states) {
    state->closing = true;
    state->queue.stop_admission();
  }
  if (impl_->executor) {impl_->executor->stop();}
}

bool SubscriberRuntime::close()
{
  prepare_close();
  std::lock_guard<std::mutex> lock(impl_->mutex);
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
  impl.require_open(*state_);
  impl.executor->check_failure();
  state_->queue.rethrow_failure();
  return state_->queue.take(state_);
}

SubscriberStats SubscriptionHandle::stats() const {return state_->queue.stats();}

std::uint64_t SubscriptionHandle::wait_for_wake(const std::uint64_t generation)
{
  return state_->queue.wait_for_wake(generation);
}

void SubscriptionHandle::finish_dispatch() noexcept {state_->queue.finish_dispatch();}

void SubscriptionHandle::clear_pending() noexcept {state_->queue.clear();}

void SubscriptionHandle::stop_delivery() noexcept {state_->queue.stop_admission();}

void SubscriptionHandle::prepare_close()
{
  auto & impl = *runtime_->impl_;
  std::lock_guard<std::mutex> lock(impl.mutex);
  if (!impl.begin_close(*state_)) {return;}
  // Keep the group attached until final close so new arrivals still release their references.
  impl.executor->invoke_or_join([]() {});
}

void SubscriptionHandle::close()
{
  auto & impl = *runtime_->impl_;
  std::lock_guard<std::mutex> lock(impl.mutex);
  if (!impl.begin_close(*state_)) {return;}
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
  impl.require_open(*state_);
  std::size_t count = 0;
  impl.executor->invoke(
    [this, &count]() {
      if (state_->descriptor) {
        auto & endpoint = state_->typed_subscription.endpoint;
        types::check_status(endpoint.publisher_count(endpoint.object, &count));
      } else {
        count = state_->subscription->get_publisher_count();
      }
    });
  return count;
}

#ifdef BUILD_TESTING
void SubscriptionHandle::test_fail_receive()
{
  std::lock_guard<std::mutex> lock(runtime_->impl_->mutex);
  runtime_->impl_->executor->invoke(
    [this]() {
      state_->fail_receive(
        std::make_exception_ptr(std::runtime_error("injected subscriber receive failure")));
    });
}

void SubscriptionHandle::test_stop_executor(const bool fail)
{
  std::lock_guard<std::mutex> lock(runtime_->impl_->mutex);
  runtime_->impl_->executor->test_stop(fail);
}

void SubscriptionHandle::test_fail_next_typed_receive()
{
  auto & impl = *runtime_->impl_;
  std::lock_guard<std::mutex> lock(impl.mutex);
  impl.require_open(*state_);
  if (!state_->descriptor) {throw std::invalid_argument("typed subscription required");}
  impl.executor->invoke([this]() {state_->fail_next_typed_receive = true;});
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
