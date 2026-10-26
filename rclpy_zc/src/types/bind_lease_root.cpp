// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>
#include <utility>
#include "lease_root.hpp"
#include "../tensor/bindings.hpp"

namespace py = pybind11;
namespace rclpy_agnocast
{
namespace types
{
namespace
{
class ReadonlyBufferOwner
{
public:
  ReadonlyBufferOwner(
    std::shared_ptr<RootLeaseOwner> root, const FieldPath & path, std::uint64_t generation)
  : root_(std::move(root)), span_(root_->buffer(path, generation)) {}
  py::buffer_info buffer() const
  {
    static std::uint64_t empty_storage = 0;
    return py::buffer_info(
      span_.size ? span_.data : &empty_storage, span_.item_size, span_.format, 1,
      {static_cast<py::ssize_t>(span_.size)}, {static_cast<py::ssize_t>(span_.item_size)}, true);
  }

private:
  const std::shared_ptr<RootLeaseOwner> root_;
  const BufferSpan span_;
};

py::dict field_info(const RootLeaseOwner & root, const FieldPath & path, std::uint64_t generation)
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

py::object read(const RootLeaseOwner & root, const FieldPath & path, std::uint64_t generation)
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
}  // namespace

void bind_lease_root(py::module_ & module)
{
  py::class_<ReadonlyBufferOwner, std::shared_ptr<ReadonlyBufferOwner>>(
    module, "_TypedReadonlyBuffer", py::buffer_protocol())
  .def_buffer(&ReadonlyBufferOwner::buffer);
  auto root = py::class_<RootLeaseOwner, std::shared_ptr<RootLeaseOwner>>(
    module, "_RootLeaseOwner");
  root
  .def("info", &field_info)
  .def("read", &read)
  .def("tensor_info", &tensor::tensor_info<RootLeaseOwner>)
  .def("dlpack", &tensor::subscriber_capsule)
  .def_static("discard_dlpack", &tensor::discard_capsule)
  .def(
    "text_bytes", [](const RootLeaseOwner & root, const FieldPath & path,
    std::uint64_t generation) {
      return py::bytes(root.text_bytes(path, generation));
    })
  .def(
    "write", [](const RootLeaseOwner &, const FieldPath &, std::uint64_t, py::handle) {
      throw py::type_error("received message fields are read-only");
    })
  .def(
    "resize", [](const RootLeaseOwner &, const FieldPath &, std::uint64_t, std::size_t) {
      throw py::type_error("received message fields are read-only");
    })
  .def(
    "buffer", [](std::shared_ptr<RootLeaseOwner> root, const FieldPath & path,
    std::uint64_t generation) {
      auto exporter = py::cast(
        std::make_shared<ReadonlyBufferOwner>(std::move(root), path, generation));
      auto * view = PyMemoryView_FromObject(exporter.ptr());
      if (!view) {throw py::error_already_set();}
      return py::reinterpret_steal<py::object>(view);
    })
  .def_property_readonly(
    "snapshot", [](const RootLeaseOwner & root) {
      py::dict result;
      result["phase"] = "READ_ONLY";
      result["generation"] = 0;
      result["accounted_bytes"] = root.accounted_bytes();
      result["exports"] = 0;
      return result;
    });
#ifdef BUILD_TESTING
  root
  .def_property_readonly(
    "_native_root_address", [](const RootLeaseOwner & root) {
      return reinterpret_cast<std::uintptr_t>(root.native_root());
    })
  .def(
    "_native_data_address", [](const RootLeaseOwner & root, const FieldPath & path,
    std::uint64_t generation) {
      return reinterpret_cast<std::uintptr_t>(root.buffer(path, generation).data);
    }, py::arg("path"), py::arg("generation") = 0);
#endif
}
}  // namespace types
}  // namespace rclpy_agnocast
