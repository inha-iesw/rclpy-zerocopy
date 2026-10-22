#include "rclpy_agnocast/bindings.hpp"

#include "rclpy_agnocast/buffer_owner.hpp"
#include "rclpy_agnocast/runtime.hpp"
#include "subscriber/native.hpp"

#include <pybind11/pybind11.h>

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

namespace py = pybind11;

namespace rclpy_agnocast
{
void bind_subscriber(py::module_ & module);
#ifdef BUILD_TESTING
void bind_observation(py::module_ & module);
#endif
namespace binding_detail
{

void throw_worker_error(const char * operation, const PublisherWorkerResult & result)
{
  if (result.failure) {
    std::rethrow_exception(result.failure);
  }
  if (result.error == PublisherWorkerError::kNone) {
    return;
  }

  const std::string prefix = std::string(operation) + ": ";
  switch (result.error) {
    case PublisherWorkerError::kBusy:
      throw std::runtime_error(prefix + "active Loan=1 keeps the publisher worker busy");
    case PublisherWorkerError::kAlreadyExists:
      throw std::runtime_error(prefix + "publisher identity already exists");
    case PublisherWorkerError::kNotFound:
      throw std::runtime_error(prefix + "publisher is closed or unknown");
    case PublisherWorkerError::kStaleGeneration:
      throw std::runtime_error(prefix + "Loan generation is stale");
    case PublisherWorkerError::kStopped:
      throw std::runtime_error(prefix + "publisher worker is stopped");
    case PublisherWorkerError::kInvalidStorage:
      throw std::runtime_error(prefix + "native publisher returned invalid storage");
    case PublisherWorkerError::kOperationFailed:
      throw std::runtime_error(prefix + "native publisher operation failed");
    case PublisherWorkerError::kNone:
      return;
  }
  throw std::runtime_error(prefix + "unknown publisher worker failure");
}

}  // namespace binding_detail

namespace
{

void bind_runtime(py::module_ & module)
{
  py::enum_<RuntimeState>(module, "RuntimeState")
  .value("OPEN", RuntimeState::kOpen)
  .value("QUIESCING", RuntimeState::kQuiescing)
  .value("CLOSED", RuntimeState::kClosed);

  py::class_<RuntimeCore, std::shared_ptr<RuntimeCore>>(module, "RuntimeCore")
  .def_static("create", [](const std::string & node_name) {
      py::gil_scoped_release release;
      return RuntimeCore::create(node_name);
    }, py::arg("node_name"))
  .def("create_publisher", [](
      const std::shared_ptr<RuntimeCore> & runtime, const std::string & topic,
      const std::size_t depth, const std::size_t max_payload_bytes)
    {
      py::gil_scoped_release release;
      return runtime->create_publisher(topic, depth, max_payload_bytes);
    }, py::arg("topic"), py::arg("depth"), py::arg("max_payload_bytes"))
  .def("create_typed_publisher", [](
      const std::shared_ptr<RuntimeCore> & runtime, const std::string & type,
      const std::string & topic, std::size_t depth, std::size_t maximum) {
      py::gil_scoped_release release;
      return runtime->create_typed_publisher(type, topic, depth, maximum);
    }, py::arg("ros_type"), py::arg("topic"), py::arg("depth"), py::arg("max_storage_bytes"))
  .def("create_subscription", [](
      const std::shared_ptr<RuntimeCore> & runtime, const std::string & topic,
      const std::size_t depth, const std::size_t count, const std::size_t bytes)
    {
      py::gil_scoped_release release;
      return runtime->create_subscription(topic, depth, count, bytes);
    }, py::arg("topic"), py::arg("depth"), py::arg("max_outstanding_leases"),
    py::arg("max_payload_bytes"))
  .def("prepare_close", [](const std::shared_ptr<RuntimeCore> & runtime) {
      PublisherWorkerResult result{};
      {
        py::gil_scoped_release release;
        result = runtime->prepare_close();
      }
      binding_detail::throw_worker_error("runtime close", result);
    })
  .def("close", [](const std::shared_ptr<RuntimeCore> & runtime) {
      PublisherWorkerResult result{};
      {
        py::gil_scoped_release release;
        result = runtime->close();
      }
      binding_detail::throw_worker_error("runtime close", result);
    })
  .def_property_readonly("state", &RuntimeCore::state);
}

}  // namespace
}  // namespace rclpy_agnocast

namespace rclpy_agnocast
{
void bind_types(py::module_ & module);
}

PYBIND11_MODULE(_native, module)
{
  module.doc() = "Typed Agnocast CPU zero-copy native adapter";
  rclpy_agnocast::bind_types(module);
  if (rclpy_agnocast::register_writable_buffer_export_type(module.ptr()) != 0) {
    throw py::error_already_set();
  }
  rclpy_agnocast::bind_publisher(module);
  rclpy_agnocast::bind_subscriber(module);
  rclpy_agnocast::bind_runtime(module);
#ifdef BUILD_TESTING
  rclpy_agnocast::bind_observation(module);
#endif
}
