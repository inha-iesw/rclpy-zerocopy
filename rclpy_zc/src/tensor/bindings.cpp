// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include "bindings.hpp"
#include <utility>

namespace py = pybind11;
namespace rclpy_agnocast
{
namespace tensor
{
namespace
{
std::vector<std::int64_t> dimensions(py::handle shape)
{
  std::vector<std::int64_t> result;
  for (auto dimension : py::reinterpret_borrow<py::iterable>(shape)) {
    if (PyBool_Check(dimension.ptr()) || !PyIndex_Check(dimension.ptr())) {
      throw py::type_error("Tensor shape dimensions must be integers");
    }
    auto index = py::reinterpret_steal<py::object>(PyNumber_Index(dimension.ptr()));
    if (!index) {throw py::error_already_set();}
    auto value = PyLong_AsLongLong(index.ptr());
    if (PyErr_Occurred()) {
      PyErr_Clear();
      throw py::value_error("Tensor shape dimension exceeds int64");
    }
    result.push_back(value);
  }
  return result;
}
py::capsule capsule(DLManagedTensor * managed)
{
  auto * value = PyCapsule_New(
    managed, "dltensor", [](PyObject * object) {
      const bool unconsumed = PyCapsule_IsValid(object, "dltensor");
      const char * name = unconsumed ? "dltensor" : "used_dltensor";
      if (PyCapsule_IsValid(object, name)) {
        release_capsule(
          static_cast<DLManagedTensor *>(PyCapsule_GetPointer(object, name)), unconsumed);
      }
    });
  if (!value) {
    managed->deleter(managed);
    throw py::error_already_set();
  }
  retain_capsule(managed);
  return py::reinterpret_steal<py::capsule>(value);
}
}  // namespace

void discard_capsule(py::handle object)
{
  if (PyCapsule_IsValid(object.ptr(), "dltensor")) {
    auto * managed = static_cast<DLManagedTensor *>(PyCapsule_GetPointer(object.ptr(), "dltensor"));
    if (PyCapsule_SetName(object.ptr(), "used_dltensor") != 0) {throw py::error_already_set();}
    managed->deleter(managed);
  }
}

py::capsule producer_capsule(
  std::shared_ptr<types::RootOwner> root, const types::FieldPath & path,
  std::uint64_t generation, py::handle shape)
{
  return capsule(export_dlpack(std::move(root), path, generation, dimensions(shape)));
}
py::capsule subscriber_capsule(
  std::shared_ptr<types::RootLeaseOwner> root, const types::FieldPath & path,
  std::uint64_t generation, py::handle shape)
{
  return capsule(export_dlpack(std::move(root), path, generation, dimensions(shape)));
}
}  // namespace tensor
}  // namespace rclpy_agnocast
