// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include <pybind11/pybind11.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include "../subscriber/queue.hpp"
#include "../types/lease_root.hpp"
#include "export.hpp"
#include "rclpy_agnocast/types/field_ops.hpp"

namespace py = pybind11;
namespace rclpy_agnocast
{
namespace tensor
{
namespace
{
struct Root {std::array<std::int32_t, 6> values;};
const types::FieldOps * root_ops()
{
  static const auto values = types::array_ops<decltype(Root::values)>(
    types::scalar_ops<std::int32_t>());
  static const types::FieldMember members[] = {
    {"values", &values, [](void * root) noexcept -> void * {
        return &static_cast<Root *>(root)->values;
      }}
  };
  static const auto root = types::message_ops<Root>(members, 1);
  return &root;
}

// Real mmap protection exercises the production provenance check without a kernel device.
class TestTensorSubscription
{
public:
  TestTensorSubscription(
    std::string name, std::size_t count, std::size_t retained, std::size_t maximum, bool writable)
  : name_(std::move(name)), writable_(writable), queue_(count, retained, maximum)
  {
    static const types::TypeOps ops{root_ops(), nullptr};
    descriptor_ = std::make_shared<types::TypeDescriptor>(types::TypeDescriptor{
          "tensor_test/msg/Root", std::string(64, 'a'), "[]", sizeof(Root), alignof(Root), &ops});
  }
  bool push()
  {
    const auto fd = shm_open(name_.c_str(), writable_ ? O_RDWR : O_RDONLY, 0);
    if (fd < 0) {throw std::runtime_error("test shm_open failed");}
    struct stat info{};
    if (fstat(fd, &info) != 0 || info.st_size < static_cast<off_t>(sizeof(Root))) {
      ::close(fd);
      throw std::runtime_error("test shared root is truncated");
    }
    auto * mapping = mmap(
      nullptr, sizeof(Root), PROT_READ | (writable_ ? PROT_WRITE : 0), MAP_SHARED, fd, 0);
    ::close(fd);
    if (mapping == MAP_FAILED) {throw std::runtime_error("test mmap failed");}
    auto storage = std::shared_ptr<void>(mapping, [released = released_](void * pointer) {
        munmap(pointer, sizeof(Root));
        ++*released;
      });
    address_ = reinterpret_cast<std::uintptr_t>(static_cast<Root *>(mapping)->values.data());
    return queue_.push({++sequence_, nullptr, sizeof(Root), std::move(storage), mapping, descriptor_});
  }
  std::shared_ptr<types::RootLeaseOwner> take()
  {
    auto lease = queue_.take();
    return lease ? std::make_shared<types::RootLeaseOwner>(lease->owner()) : nullptr;
  }
  SubscriberStats stats() const {return queue_.stats();}
  std::size_t releases() const {return released_->load();}
  std::uintptr_t address() const {return address_;}
  void clear() {queue_.clear();}
  void close()
  {
    if (!queue_.drain_for_close()) {throw std::runtime_error("active root or view");}
  }

private:
  const std::string name_;
  const bool writable_;
  SubscriberQueue queue_;
  std::shared_ptr<const types::TypeDescriptor> descriptor_;
  std::shared_ptr<std::atomic<std::size_t>> released_{
    std::make_shared<std::atomic<std::size_t>>(0)};
  std::uint64_t sequence_{0};
  std::uintptr_t address_{0};
};
}  // namespace

void bind_testing(py::module_ & module)
{
  module.def("_test_tensor_live_managers", &live_managers);
  py::class_<TestTensorSubscription>(module, "_TestTensorSubscription")
  .def(py::init<std::string, std::size_t, std::size_t, std::size_t, bool>())
  .def("push", &TestTensorSubscription::push)
  .def("take", &TestTensorSubscription::take)
  .def("clear_pending", &TestTensorSubscription::clear)
  .def("close", &TestTensorSubscription::close)
  .def_property_readonly("stats", &TestTensorSubscription::stats)
  .def_property_readonly("release_count", &TestTensorSubscription::releases)
  .def_property_readonly("last_data_address", &TestTensorSubscription::address)
  .def_property_readonly("last_accounted_bytes", [](const TestTensorSubscription &) {
      return sizeof(Root);
    });
}
}  // namespace tensor
}  // namespace rclpy_agnocast
