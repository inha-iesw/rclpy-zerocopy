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

#include <pybind11/pybind11.h>

#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "queue.hpp"
#include "native.hpp"

namespace py = pybind11;
namespace rclpy_agnocast
{
namespace
{
class TestSubscription
{
public:
  TestSubscription(std::size_t count, std::size_t bytes)
  : queue_(count, bytes) {}

  bool push(std::uint64_t sequence, const std::string & bytes)
  {
    auto storage = std::shared_ptr<std::vector<std::uint8_t>>(
      new std::vector<std::uint8_t>(bytes.begin(), bytes.end()),
      [released = released_, tid = release_tid_](auto * value) {
        delete value;
        *tid = syscall(SYS_gettid);
        ++*released;
      });
    address_ = reinterpret_cast<std::uintptr_t>(storage->data());
    return queue_.push({sequence, storage->data(), storage->size(), std::move(storage)});
  }
  std::shared_ptr<LeaseState> take() {return queue_.take();}
  std::uint64_t wait_for_wake(std::uint64_t generation)
  {
    return queue_.wait_for_wake(generation);
  }
  void finish_dispatch() {queue_.finish_dispatch();}
  void clear_pending() {queue_.clear();}
  void prepare_close() {queue_.stop_admission();}
  void fail()
  {
    queue_.fail(std::make_exception_ptr(std::runtime_error("injected receive failure")));
  }
  SubscriberStats stats() const {return queue_.stats();}
  std::size_t release_count() const {return released_->load();}
  std::int64_t release_tid() const {return release_tid_->load();}
  std::uintptr_t address() const {return address_;}
  void close()
  {
    if (!queue_.drain_for_close()) {throw std::runtime_error("active Lease or view");}
  }

private:
  SubscriberQueue queue_;
  std::shared_ptr<std::atomic<std::size_t>> released_{
    std::make_shared<std::atomic<std::size_t>>(0)};
  std::uintptr_t address_{0};
  std::shared_ptr<std::atomic<std::int64_t>> release_tid_{
    std::make_shared<std::atomic<std::int64_t>>(0)};
};

bool wait_holding_gil(
  const std::function<bool()> & complete, const unsigned timeout_ms, const int ready_fd)
{
  // This binding keeps the GIL throughout the wait; neither native sleep nor write releases it.
  // Announce readiness here, after entering the GIL-held region, to synchronize the producer.
  constexpr char ready[] = "EVENT name=BLOCKING\n";
  if (write(ready_fd, ready, sizeof(ready) - 1) != sizeof(ready) - 1) {
    throw std::runtime_error("could not announce GIL-held wait");
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!complete()) {
    if (std::chrono::steady_clock::now() >= deadline) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}
}  // namespace

void bind_subscriber_testing(py::module_ & module)
{
  py::class_<TestSubscription>(module, "_TestSubscription")
  .def(py::init<std::size_t, std::size_t>())
  .def("push", &TestSubscription::push)
  .def("take", &TestSubscription::take)
  .def(
    "wait_for_wake", &TestSubscription::wait_for_wake,
    py::call_guard<py::gil_scoped_release>())
  .def("finish_dispatch", &TestSubscription::finish_dispatch)
  .def("clear_pending", &TestSubscription::clear_pending)
  .def("prepare_close", &TestSubscription::prepare_close)
  .def("stop_delivery", &TestSubscription::prepare_close)
  .def("fail", &TestSubscription::fail)
  .def("close", &TestSubscription::close)
  .def_property_readonly("stats", &TestSubscription::stats)
  .def_property_readonly("release_count", &TestSubscription::release_count)
  .def_property_readonly("release_tid", &TestSubscription::release_tid)
  .def_property_readonly("last_data_address", &TestSubscription::address);

  module.def(
    "_test_wait_for_receipts_holding_gil", [](TestSubscription & endpoint,
    const std::size_t count, const unsigned timeout_ms, const int ready_fd) {
      return wait_holding_gil(
        [&]() {return endpoint.stats().received_total >= count;}, timeout_ms, ready_fd);
    });
  module.def(
    "_test_wait_for_receipts_holding_gil", [](SubscriptionHandle & endpoint,
    const std::size_t count, const unsigned timeout_ms, const int ready_fd) {
      return wait_holding_gil(
        [&]() {
          const auto stats = endpoint.stats();
          const auto observation = endpoint.observation();
          return stats.received_total >= count && observation.received == stats.received_total &&
          observation.received - observation.released == stats.outstanding_leases;
        }, timeout_ms, ready_fd);
    });
}
}  // namespace rclpy_agnocast
