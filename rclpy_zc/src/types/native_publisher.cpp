// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0

#include "native_publisher.hpp"

#include <stdexcept>
#include <utility>

#include "rclpy_agnocast/publisher_worker.hpp"
#include "rclpy_agnocast/types/adapter.hpp"
#include "rclpy_agnocast/types/operations.hpp"
#include "../publisher/observation.hpp"

namespace rclpy_agnocast
{
namespace types
{
namespace
{
class NativePublisherBackend final : public PublisherBackend
{
public:
  explicit NativePublisherBackend(std::shared_ptr<const TypeDescriptor> descriptor)
  : descriptor_(std::move(descriptor)) {}

  ~NativePublisherBackend() override
  {
    if (endpoint_.object) {endpoint_.destroy(endpoint_.object);}
  }

  void initialize(void * node, const char * topic, const std::size_t depth)
  {
    check_status(descriptor_->ops->create(node, topic, depth, &endpoint_));
#ifdef BUILD_TESTING
    observation::record(observation::Event::kCreate);
#endif
  }

  BorrowedStorage borrow(std::size_t) override
  {
    throw std::logic_error("typed publisher requires borrow_message");
  }

  void * borrow_root() override
  {
    void * root = nullptr;
#ifdef BUILD_TESTING
    observation::record(observation::Event::kBorrow);
#endif
    const auto status = endpoint_.borrow(endpoint_.object, &root);
#ifdef BUILD_TESTING
    observation::record(
      status.code == ErrorCode::kOk ?
      observation::Event::kBorrowed : observation::Event::kBorrowFailed,
      reinterpret_cast<std::uintptr_t>(root));
#endif
    check_status(status);
    return root;
  }

  void publish() override
  {
#ifdef BUILD_TESTING
    observation::record(observation::Event::kPublish);
#endif
    const auto status = endpoint_.publish(endpoint_.object);
#ifdef BUILD_TESTING
    observation::record(observation::Event::kPublished);
#endif
    check_status(status);
  }
  void cancel() noexcept override
  {
#ifdef BUILD_TESTING
    observation::record(observation::Event::kCancel);
#endif
    endpoint_.cancel(endpoint_.object);
#ifdef BUILD_TESTING
    observation::record(observation::Event::kCancelled);
#endif
  }

private:
  std::shared_ptr<const TypeDescriptor> descriptor_;
  NativeEndpoint endpoint_;
};

class NativePublisherFactory final : public PublisherBackendFactory
{
public:
  NativePublisherFactory(
    void * node, std::string topic, const std::size_t depth,
    std::shared_ptr<const TypeDescriptor> descriptor)
  : node_(node), topic_(std::move(topic)), depth_(depth), descriptor_(std::move(descriptor)) {}

  std::unique_ptr<PublisherBackend> create() override
  {
    auto backend = std::make_unique<NativePublisherBackend>(descriptor_);
    backend->initialize(node_, topic_.c_str(), depth_);
    return backend;
  }

private:
  void * node_;
  std::string topic_;
  std::size_t depth_;
  std::shared_ptr<const TypeDescriptor> descriptor_;
};
}  // namespace

std::unique_ptr<PublisherBackendFactory> native_publisher_factory(
  void * node, std::string topic, const std::size_t depth,
  std::shared_ptr<const TypeDescriptor> descriptor)
{
  return std::make_unique<NativePublisherFactory>(
    node, std::move(topic), depth, std::move(descriptor));
}

}  // namespace types
}  // namespace rclpy_agnocast
