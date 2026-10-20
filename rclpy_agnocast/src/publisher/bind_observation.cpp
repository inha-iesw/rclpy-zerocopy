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

#include "observation.hpp"
#include "rclpy_agnocast/bindings.hpp"
#include "rclpy_agnocast/loan.hpp"

namespace rclpy_agnocast
{
void bind_observation(pybind11::module_ & module)
{
  namespace py = pybind11;
  namespace obs = observation;
  module.def("_test_enable_observation", &obs::enable);
  module.def("_test_disable_observation", &obs::disable);
  module.def("_test_is_shared", &obs::is_shared);
  module.def(
    "_test_fail_allocation", [](const std::string & stage, const std::size_t ordinal) {
      if (stage != "borrow" && stage != "resize") {
        throw py::value_error("stage must be borrow or resize");
      }
      obs::arm_failure(stage == "borrow" ? obs::Stage::kBorrow : obs::Stage::kResize, ordinal);
    }, py::arg("stage"), py::arg("ordinal") = 0);
  module.def(
    "_test_replay_publish", [](const std::shared_ptr<LoanState> & loan) {
      PublisherWorkerResult result;
      {
        py::gil_scoped_release release;
        result = loan->test_replay_publish();
      }
      binding_detail::throw_worker_error("replayed publish", result);
    });
  module.def("_test_publisher_snapshot", []() {
      const auto value = obs::snapshot();
      py::dict result;
#define FIELD(name) result[#name] = value.name
      FIELD(worker_tid); FIELD(borrow_tid); FIELD(resize_tid); FIELD(publish_tid);
      FIELD(cancel_tid); FIELD(destroy_tid); FIELD(borrow_entries); FIELD(borrow_complete);
      FIELD(resize_entries); FIELD(publish_complete); FIELD(cancel_complete);
      FIELD(unpublished_destroy); FIELD(terminal_enqueues); FIELD(async_cancel_enqueues);
      FIELD(last_enqueue_tid); FIELD(terminal_tls); FIELD(borrow_tls); FIELD(data_address);
      FIELD(message_address); FIELD(backend_epoch); FIELD(filling_allocations);
      FIELD(worker_allocations); FIELD(shared_allocations); FIELD(injected_failures);
      FIELD(caller_tid); FIELD(caller_tls);
      FIELD(failure_live_allocations); FIELD(failure_tracked_allocations);
      FIELD(failure_tracking_overflow);
#undef FIELD
      return result;
    });
}
}  // namespace rclpy_agnocast
