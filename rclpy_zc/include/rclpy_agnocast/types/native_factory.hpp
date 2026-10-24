// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef RCLPY_AGNOCAST__TYPES__NATIVE_FACTORY_HPP_
#define RCLPY_AGNOCAST__TYPES__NATIVE_FACTORY_HPP_

#include <cstddef>
#include <memory>
#include <utility>
#include "agnocast/node/agnocast_node.hpp"
#include "rclpy_agnocast/types/field_ops.hpp"
#include "rclpy_agnocast/types/operations.hpp"

namespace rclpy_agnocast
{
namespace types
{
namespace native_factory_detail
{

template<typename Message>
struct Endpoint
{
  using Publisher = agnocast::Publisher<Message>;
  explicit Endpoint(typename Publisher::SharedPtr value)
  : publisher(std::move(value)) {}

  static Status borrow(void * object, void ** output) noexcept
  {
    *output = nullptr;
    auto & self = *static_cast<Endpoint *>(object);
    if (self.loan) {return {ErrorCode::kRuntime, "publisher already has a native loan"};}
    try {
      self.loan = self.publisher->borrow_loaned_message();
      *output = self.loan.get();
      return {};
    } catch (...) {
      self.loan.reset();
      return status_from_exception();
    }
  }

  static Status publish(void * object) noexcept
  {
    auto & self = *static_cast<Endpoint *>(object);
    if (!self.loan) {return {ErrorCode::kRuntime, "publisher has no native loan"};}
    try {
      self.publisher->publish(std::move(self.loan));
      self.loan.reset();
      return {};
    } catch (...) {
      self.loan.reset();
      return status_from_exception();
    }
  }

  static void cancel(void * object) noexcept
  {
    static_cast<Endpoint *>(object)->loan.reset();
  }

  static void destroy(void * object) noexcept
  {
    delete static_cast<Endpoint *>(object);
  }

  typename Publisher::SharedPtr publisher;
  agnocast::ipc_shared_ptr<Message> loan;
};

template<typename Message>
struct SubscriptionEndpoint
{
  using Subscription = agnocast::Subscription<Message>;
  typename Subscription::SharedPtr subscription;

  static void destroy(void * object) noexcept
  {
    delete static_cast<SubscriptionEndpoint *>(object);
  }
  static Status publisher_count(void * object, std::size_t * output) noexcept
  {
    return field_detail::protect(
      [&] {
        *output = static_cast<SubscriptionEndpoint *>(object)->subscription->get_publisher_count();
      });
  }
};

}  // namespace native_factory_detail

/**
 * @brief Construct a typed Agnocast endpoint without exposing provider exceptions or RTTI.
 *
 * Every endpoint operation, including cancellation and destruction, must execute on the
 * publisher worker. A successful borrow owns one native root until publish or cancel;
 * publish releases that ownership even when it reports an error. The runtime must retain
 * the node and pin this provider's code until the endpoint and all root references are gone.
 *
 * @tparam Message Native ROS message type selected by generated schema operations.
 * @param[in] node Non-null borrowed agnocast::Node pointer; runtime retains ownership.
 * @param[in] topic Non-null borrowed topic name, used only during construction.
 * @param[in] depth Positive native publisher queue depth.
 * @param[out] output Non-null endpoint slot; empty on failure, caller-owned on success.
 * @return Success or an owned status copied from an exception caught inside the provider.
 */
template<typename Message>
Status create_endpoint(
  void * node, const char * topic, std::size_t depth, NativeEndpoint * output) noexcept
{
  *output = {};
  using Endpoint = native_factory_detail::Endpoint<Message>;
  try {
    auto publisher = static_cast<agnocast::Node *>(node)->create_publisher<Message>(
      topic, depth, agnocast::PublisherOptions{});
    auto endpoint = std::make_unique<Endpoint>(std::move(publisher));
    *output = {endpoint.get(), &Endpoint::borrow, &Endpoint::publish,
      &Endpoint::cancel, &Endpoint::destroy};
    endpoint.release();
    return {};
  } catch (...) {
    return status_from_exception();
  }
}

/**
 * @brief Construct a typed subscription whose native callback transfers immutable ownership.
 *
 * @param[in] node Non-null borrowed Agnocast node retained by the runtime.
 * @param[in] topic Non-null borrowed resolved topic name used during construction.
 * @param[in] depth Positive KeepLast queue depth.
 * @param[in] callback_group Non-null borrowed pointer to a CallbackGroup::SharedPtr to copy.
 * @param[in] context Non-null callback context retained until detach and destroy complete.
 * @param[in] receive No-throw receiver taking ownership of each delivered native reference.
 * @param[out] output Non-null endpoint slot, empty on failure and caller-owned on success.
 * @return Owned status; construction executes on the native subscriber executor thread.
 * @note Destruction may run on the closing caller after executor join and reference release.
 */
template<typename Message>
Status create_subscription_endpoint(
  void * node, const char * topic, std::size_t depth, void * callback_group,
  void * context, NativeReceive receive, NativeSubscriptionEndpoint * output) noexcept
{
  *output = {};
  using Endpoint = native_factory_detail::SubscriptionEndpoint<Message>;
  using Pointer = agnocast::ipc_shared_ptr<const Message>;
  try {
    agnocast::SubscriptionOptions options;
    options.callback_group = *static_cast<rclcpp::CallbackGroup::SharedPtr *>(callback_group);
    auto endpoint = std::make_unique<Endpoint>();
    endpoint->subscription = static_cast<agnocast::Node *>(node)->create_subscription<Message>(
      topic, depth, [context, receive](Pointer message) noexcept {
          try {
            auto owner = std::make_unique<Pointer>(std::move(message));
            NativeReceived value{owner->get(), owner.get(), [] (void * pointer) noexcept {
                delete static_cast<Pointer *>(pointer);
              }};
            owner.release();
            receive(context, value, {});
          } catch (...) {
            receive(context, {}, status_from_exception());
          }
        }, options);
    *output = {endpoint.get(), &Endpoint::destroy, &Endpoint::publisher_count};
    endpoint.release();
    return {};
  } catch (...) {
    return status_from_exception();
  }
}

}  // namespace types
}  // namespace rclpy_agnocast
#endif  // RCLPY_AGNOCAST__TYPES__NATIVE_FACTORY_HPP_
