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

#include "buffer_owner.hpp"
#include "lease.hpp"
#include "native.hpp"
#include "queue.hpp"

namespace py = pybind11;
namespace rclpy_agnocast
{
#ifdef BUILD_TESTING
void bind_subscriber_testing(py::module_ & module);
#endif

void bind_subscriber(py::module_ & module)
{
  if (register_readonly_buffer_export_type(module.ptr()) != 0) {throw py::error_already_set();}
  py::class_<SubscriberStats>(module, "SubscriberStats")
  .def_readonly("received_total", &SubscriberStats::received_total)
  .def_readonly("delivered_total", &SubscriberStats::delivered_total)
  .def_readonly("dropped_outstanding", &SubscriberStats::dropped_outstanding)
  .def_readonly("dropped_oversize", &SubscriberStats::dropped_oversize)
  .def_readonly("queued_leases", &SubscriberStats::queued_leases)
  .def_readonly("outstanding_leases", &SubscriberStats::outstanding_leases)
  .def_readonly("outstanding_payload_bytes", &SubscriberStats::outstanding_payload_bytes);

  auto lease_binding = py::class_<LeaseState, std::shared_ptr<LeaseState>>(
    module, "Lease", "Read-only SharedBuffer ownership; release consumers before source views.");
  lease_binding
  .def_property_readonly(
    "sequence", [](const LeaseState & lease) {
      return lease.owner()->buffer().sequence;
    }, "Return the message sequence; a released Lease raises RuntimeError.")
  .def_property_readonly(
    "buffer", [](const LeaseState & lease) {
      auto * raw = create_readonly_buffer_export(lease.owner());
      if (!raw) {throw py::error_already_set();}
      auto exporter = py::reinterpret_steal<py::object>(raw);
      auto * view = PyMemoryView_FromObject(exporter.ptr());
      if (!view) {throw py::error_already_set();}
      return py::reinterpret_steal<py::memoryview>(view);
    }, "Return a read-only view retaining storage independently of Lease.release().")
  .def(
    "release", &LeaseState::release,
    "Invalidate this wrapper; existing buffer aliases keep their native storage.")
  .def(
    "__enter__", [](const std::shared_ptr<LeaseState> & lease) {
      (void)lease->owner();
      return lease;
    })
  .def(
    "__exit__", [](LeaseState & lease, py::object, py::object, py::object) {
      lease.release();
      return false;
    });

  auto subscription = py::class_<SubscriptionHandle, std::shared_ptr<SubscriptionHandle>>(
    module, "SubscriptionHandle");
  subscription
  .def("take", &SubscriptionHandle::take, py::call_guard<py::gil_scoped_release>())
  .def("close", &SubscriptionHandle::close, py::call_guard<py::gil_scoped_release>())
  .def_property_readonly("stats", &SubscriptionHandle::stats)
  .def_property_readonly(
    "publisher_count", [](SubscriptionHandle & handle) {
      py::gil_scoped_release release;
      return handle.publisher_count();
    });
#ifdef BUILD_TESTING
  lease_binding.def_property_readonly(
    "_native_data_address", [](const LeaseState & lease) {
      return reinterpret_cast<std::uintptr_t>(lease.owner()->buffer().data);
    });
  py::class_<SubscriberObservation>(module, "_SubscriberObservation")
  .def_readonly("received", &SubscriberObservation::received)
  .def_readonly("released", &SubscriberObservation::released)
  .def_readonly("callback_tid", &SubscriberObservation::callback_tid)
  .def_readonly("release_tid", &SubscriberObservation::release_tid)
  .def_readonly("native_data_address", &SubscriberObservation::native_data_address);
  subscription.def_property_readonly("_observation", &SubscriptionHandle::observation);
  subscription.def(
    "_test_stop_executor", &SubscriptionHandle::test_stop_executor,
    py::call_guard<py::gil_scoped_release>());
  subscription.def("_test_fail_next_control", &SubscriptionHandle::test_fail_next_control);
  bind_subscriber_testing(module);
#endif
}
}  // namespace rclpy_agnocast
