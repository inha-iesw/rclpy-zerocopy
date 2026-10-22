// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>
#include <string>
#include <utility>
#include "root.hpp"

namespace py = pybind11;
namespace rclpy_agnocast
{
namespace types
{
namespace
{
class BufferOwner
{
public:
  BufferOwner(std::shared_ptr<RootOwner> root, const FieldPath & path, std::uint64_t generation)
  : root_(std::move(root)), span_(root_->acquire_export(path, generation)) {}
  ~BufferOwner() {root_->release_export();}
  py::buffer_info buffer() const
  {
    static std::uint64_t empty_storage = 0;
    return py::buffer_info(
      span_.size ? span_.data : &empty_storage, span_.item_size, span_.format, 1,
      {static_cast<py::ssize_t>(span_.size)}, {static_cast<py::ssize_t>(span_.item_size)});
  }

private:
  std::shared_ptr<RootOwner> root_;
  BufferSpan span_;
};
py::dict field_info(const RootOwner & root, const FieldPath & path, std::uint64_t generation)
{
  const auto info = root.info(path, generation);
  py::dict result;
  const char * kinds[] = {"scalar", "message", "array", "sequence", "string", "wstring"};
  result["kind"] = kinds[static_cast<int>(info.ops->kind)];
  result["size"] = info.size;
  result["generation"] = info.generation;
  py::tuple members(info.ops->member_count);
  for (std::size_t i = 0; i < info.ops->member_count; ++i) {
    members[i] = info.ops->members[i].name;
  }
  result["members"] = std::move(members);
  return result;
}
py::object read(const RootOwner & root, const FieldPath & path, std::uint64_t generation)
{
  const auto info = root.info(path, generation);
  const auto value = root.read(path, generation);
  switch (info.ops->scalar) {
    case ScalarKind::kBool: return py::bool_(value.boolean);
    case ScalarKind::kSigned: return py::int_(value.signed_integer);
    case ScalarKind::kUnsigned: return py::int_(value.unsigned_integer);
    case ScalarKind::kFloat: return py::float_(value.real);
  }
  throw py::type_error("unknown scalar category");
}
void write(RootOwner & root, const FieldPath & path, std::uint64_t generation, py::handle input)
{
  const auto info = root.info(path, generation);
  if (info.ops->kind != FieldKind::kScalar) {
    throw py::type_error("only scalar assignment is supported");
  }
  ScalarValue value{};
  switch (info.ops->scalar) {
    case ScalarKind::kBool:
      if (!PyBool_Check(input.ptr())) {throw py::type_error("boolean field requires bool");}
      value.boolean = input.ptr() == Py_True;
      break;
    case ScalarKind::kSigned:
    case ScalarKind::kUnsigned:
      if (!PyLong_Check(input.ptr()) || PyBool_Check(input.ptr())) {
        throw py::type_error("integer field requires int");
      }
      if (info.ops->scalar == ScalarKind::kSigned) {
        value.signed_integer = PyLong_AsLongLong(input.ptr());
      } else {
        value.unsigned_integer = PyLong_AsUnsignedLongLong(input.ptr());
      }
      if (PyErr_Occurred()) {
        PyErr_Clear();
        throw py::value_error("integer exceeds native range");
      }
      break;
    case ScalarKind::kFloat:
      if ((!PyFloat_Check(input.ptr()) && !PyLong_Check(input.ptr())) ||
        PyBool_Check(input.ptr()))
      {
        throw py::type_error("floating point field requires a number");
      }
      value.real = PyFloat_AsDouble(input.ptr());
      if (PyErr_Occurred()) {
        PyErr_Clear();
        throw py::value_error("floating point value exceeds native range");
      }
      break;
  }
  root.write(path, generation, value);
}
}  // namespace

void bind_root(py::module_ & module)
{
  py::register_exception<BufferBusy>(module, "_TypedBufferBusy", PyExc_BufferError);
  py::register_exception<UnsupportedOperation>(
    module, "_UnsupportedFieldOperation",
    PyExc_NotImplementedError);
  py::class_<BufferOwner, std::shared_ptr<BufferOwner>>(
    module, "_TypedBuffer",
    py::buffer_protocol())
  .def_buffer(&BufferOwner::buffer);
  py::class_<RootOwner, std::shared_ptr<RootOwner>>(module, "_RootOwner")
  .def("info", &field_info)
  .def("read", &read)
  .def(
    "text_bytes", [](const RootOwner & root, const FieldPath & path, std::uint64_t generation) {
      return py::bytes(root.text_bytes(path, generation));
    })
  .def("write", &write)
  .def(
    "resize", [](RootOwner & root, const FieldPath & path, std::uint64_t generation,
    std::size_t size) {
      py::gil_scoped_release release;
      root.resize(path, generation, size);
    })
  .def(
    "buffer", [](std::shared_ptr<RootOwner> root, const FieldPath & path,
    std::uint64_t generation) {
      auto exporter = py::cast(std::make_shared<BufferOwner>(std::move(root), path, generation));
      auto * view = PyMemoryView_FromObject(exporter.ptr());
      if (!view) {throw py::error_already_set();}
      return py::reinterpret_steal<py::object>(view);
    })
  .def(
    "publish", [](RootOwner & root, std::uintptr_t id) {
      py::gil_scoped_release release;
      root.publish(id);
    })
  .def(
    "cancel", [](RootOwner & root, std::uintptr_t id, bool defer) {
      py::gil_scoped_release release;
      root.cancel(id, defer);
    }, py::arg("publisher_id"), py::arg("defer") = false)
  .def("abandon", &RootOwner::abandon)
  .def_property_readonly("publisher_id", &RootOwner::publisher_id)
  .def_property_readonly(
    "snapshot", [](const RootOwner & root) {
      const auto value = root.snapshot();
      const char * phases[] = {
        "PREPARE", "FILLING", "CANCEL_ONLY", "CANCEL_PENDING", "PUBLISHED", "CANCELLED",
        "PUBLISHING", "CANCELLING", "FAILED"};
      py::dict result;
      result["phase"] = phases[static_cast<int>(value.phase)];
      result["generation"] = value.generation;
      result["accounted_bytes"] = value.accounted_bytes;
      result["exports"] = value.exports;
      return result;
    });
}
}  // namespace types
}  // namespace rclpy_agnocast
