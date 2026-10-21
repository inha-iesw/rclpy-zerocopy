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

#include "buffer_owner.hpp"

#include <cstddef>
#include <new>
#include <utility>

namespace rclpy_agnocast
{
namespace
{
struct ReadonlyBufferExportObject
{
  PyObject_HEAD
  std::shared_ptr<LeaseBufferOwner> * owner;
};

// The heap type reference is intentionally retained for the process lifetime.
PyTypeObject * export_type = nullptr;
PyInterpreterState * export_interpreter = nullptr;
std::byte zero_length_storage{};

int readonly_getbuffer(PyObject * object, Py_buffer * view, const int flags) noexcept
{
  auto * token = reinterpret_cast<ReadonlyBufferExportObject *>(object);
  if (!view || !token->owner || !*token->owner) {
    PyErr_SetString(PyExc_BufferError, "read-only buffer owner is not active");
    return -1;
  }
  const auto & buffer = (*token->owner)->buffer();
  void * const data = buffer.size == 0 && buffer.data == nullptr ?
    static_cast<void *>(&zero_length_storage) : const_cast<std::uint8_t *>(buffer.data);
  return PyBuffer_FillInfo(view, object, data, static_cast<Py_ssize_t>(buffer.size), 1, flags);
}

void readonly_dealloc(PyObject * object) noexcept
{
  auto * token = reinterpret_cast<ReadonlyBufferExportObject *>(object);
  auto * type = Py_TYPE(object);
  delete token->owner;
  type->tp_free(object);
  Py_DECREF(type);
}

PyType_Slot slots[] = {
  {Py_tp_dealloc, reinterpret_cast<void *>(readonly_dealloc)},
  {Py_bf_getbuffer, reinterpret_cast<void *>(readonly_getbuffer)},
  {0, nullptr},
};
PyType_Spec spec = {
  "rclpy_agnocast._native._ReadonlyBufferExport",
  static_cast<int>(sizeof(ReadonlyBufferExportObject)),
  0,
  Py_TPFLAGS_DEFAULT | Py_TPFLAGS_DISALLOW_INSTANTIATION | Py_TPFLAGS_IMMUTABLETYPE,
  slots,
};
}  // namespace

int register_readonly_buffer_export_type(PyObject * module) noexcept
{
  if (!module || !PyModule_Check(module)) {
    PyErr_SetString(PyExc_TypeError, "read-only buffer owner requires a Python module");
    return -1;
  }
  auto * const interpreter = PyInterpreterState_Get();
  if (export_type && interpreter != export_interpreter) {
    PyErr_SetString(
      PyExc_RuntimeError, "read-only buffer exports do not support CPython subinterpreters");
    return -1;
  }
  if (export_type) {
    return PyModule_AddObjectRef(
      module, "_ReadonlyBufferExport", reinterpret_cast<PyObject *>(export_type));
  }
  auto * type = PyType_FromSpec(&spec);
  if (!type) {return -1;}
  if (PyModule_AddObjectRef(module, "_ReadonlyBufferExport", type) != 0) {
    Py_DECREF(type);
    return -1;
  }
  export_type = reinterpret_cast<PyTypeObject *>(type);
  export_interpreter = interpreter;
  return 0;
}

PyObject * create_readonly_buffer_export(std::shared_ptr<LeaseBufferOwner> owner) noexcept
{
  if (!owner || !export_type) {
    PyErr_SetString(PyExc_RuntimeError, "read-only buffer requires a registered active owner");
    return nullptr;
  }
  if (PyInterpreterState_Get() != export_interpreter) {
    PyErr_SetString(
      PyExc_RuntimeError, "read-only buffer exports do not support CPython subinterpreters");
    return nullptr;
  }
  if (owner->buffer().size > static_cast<std::size_t>(PY_SSIZE_T_MAX)) {
    PyErr_SetString(PyExc_OverflowError, "read-only buffer exceeds Py_ssize_t");
    return nullptr;
  }
  auto * token = reinterpret_cast<ReadonlyBufferExportObject *>(export_type->tp_alloc(
      export_type,
      0));
  if (!token) {return nullptr;}
  token->owner = new (std::nothrow) std::shared_ptr<LeaseBufferOwner>(std::move(owner));
  if (!token->owner) {
    Py_DECREF(reinterpret_cast<PyObject *>(token));
    PyErr_NoMemory();
    return nullptr;
  }
  return reinterpret_cast<PyObject *>(token);
}
}  // namespace rclpy_agnocast
