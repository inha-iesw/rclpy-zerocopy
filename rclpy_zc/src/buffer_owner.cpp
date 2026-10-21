#include "rclpy_agnocast/buffer_owner.hpp"

#include "rclpy_agnocast/loan.hpp"

#include <cstddef>
#include <new>
#include <utility>

namespace rclpy_agnocast
{
namespace
{

constexpr char kExportTypeName[] = "rclpy_agnocast._native._WritableBufferExport";
constexpr char kExportModuleName[] = "_WritableBufferExport";

struct WritableBufferExportObject
{
  PyObject_HEAD
  std::shared_ptr<LoanState> * state;
  void * data;
  Py_ssize_t size;
  Py_ssize_t buffer_requests;
  bool acquired;
};

// The PyType_FromSpec() new reference is intentionally owned for the process lifetime.
PyTypeObject * export_type = nullptr;
PyInterpreterState * export_interpreter = nullptr;
std::byte zero_length_storage{};

int writable_getbuffer(PyObject * object, Py_buffer * view, const int flags) noexcept
{
  if (view == nullptr) {
    PyErr_SetString(PyExc_BufferError, "writable buffer request has no destination view");
    return -1;
  }
  view->obj = nullptr;

  auto * token = reinterpret_cast<WritableBufferExportObject *>(object);
  if (!token->acquired || token->state == nullptr || !*token->state) {
    PyErr_SetString(PyExc_BufferError, "writable buffer export is not active");
    return -1;
  }
  if (token->buffer_requests == PY_SSIZE_T_MAX) {
    PyErr_SetString(PyExc_BufferError, "too many writable buffer requests");
    return -1;
  }

  void * const data = token->size == 0 && token->data == nullptr ?
    static_cast<void *>(&zero_length_storage) : token->data;
  if (PyBuffer_FillInfo(view, object, data, token->size, 0, flags) != 0) {
    return -1;
  }

  ++token->buffer_requests;
  return 0;
}

void writable_releasebuffer(PyObject * object, Py_buffer *) noexcept
{
  auto * token = reinterpret_cast<WritableBufferExportObject *>(object);
  if (token->buffer_requests > 0) {
    --token->buffer_requests;
  }
}

void writable_dealloc(PyObject * object) noexcept
{
  auto * token = reinterpret_cast<WritableBufferExportObject *>(object);
  PyTypeObject * const type = Py_TYPE(object);
  if (token->acquired && token->state != nullptr && *token->state) {
    token->acquired = false;
    (*token->state)->release_writable_export();
  }

  delete token->state;
  token->state = nullptr;
  type->tp_free(object);
  Py_DECREF(type);
}

PyType_Slot export_type_slots[] = {
  {Py_tp_dealloc, reinterpret_cast<void *>(writable_dealloc)},
  {Py_bf_getbuffer, reinterpret_cast<void *>(writable_getbuffer)},
  {Py_bf_releasebuffer, reinterpret_cast<void *>(writable_releasebuffer)},
  {0, nullptr},
};

PyType_Spec export_type_spec = {
  kExportTypeName,
  static_cast<int>(sizeof(WritableBufferExportObject)),
  0,
  Py_TPFLAGS_DEFAULT | Py_TPFLAGS_DISALLOW_INSTANTIATION | Py_TPFLAGS_IMMUTABLETYPE,
  export_type_slots,
};

void set_acquire_error(const WritableExportAcquireResult result) noexcept
{
  if (result == WritableExportAcquireResult::kWrongThread) {
    PyErr_SetString(
      PyExc_RuntimeError, "writable buffer must be acquired on the Loan creator thread");
    return;
  }
  PyErr_SetString(PyExc_RuntimeError, "Loan is not accepting writable buffer exports");
}

}  // namespace

int register_writable_buffer_export_type(PyObject * module) noexcept
{
  if (module == nullptr || !PyModule_Check(module)) {
    PyErr_SetString(PyExc_TypeError, "writable buffer owner requires a Python module");
    return -1;
  }
  PyInterpreterState * const interpreter = PyInterpreterState_Get();
  if (export_type != nullptr && interpreter != export_interpreter) {
    PyErr_SetString(
      PyExc_RuntimeError, "writable buffer exports do not support CPython subinterpreters");
    return -1;
  }
  if (export_type != nullptr) {
    return PyModule_AddObjectRef(
      module, kExportModuleName, reinterpret_cast<PyObject *>(export_type));
  }

  PyObject * const type_object = PyType_FromSpec(&export_type_spec);
  if (type_object == nullptr) {
    return -1;
  }
  if (PyModule_AddObjectRef(module, kExportModuleName, type_object) != 0) {
    Py_DECREF(type_object);
    return -1;
  }

  export_type = reinterpret_cast<PyTypeObject *>(type_object);
  export_interpreter = interpreter;
  return 0;
}

PyObject * create_writable_buffer_export(std::shared_ptr<LoanState> state) noexcept
{
  if (export_type == nullptr) {
    PyErr_SetString(PyExc_RuntimeError, "writable buffer export type is not registered");
    return nullptr;
  }
  if (PyInterpreterState_Get() != export_interpreter) {
    PyErr_SetString(
      PyExc_RuntimeError, "writable buffer exports do not support CPython subinterpreters");
    return nullptr;
  }
  if (!state) {
    PyErr_SetString(PyExc_RuntimeError, "writable buffer export requires an active Loan");
    return nullptr;
  }

  auto * token = reinterpret_cast<WritableBufferExportObject *>(export_type->tp_alloc(
      export_type, 0));
  if (token == nullptr) {
    return nullptr;
  }
  token->state = nullptr;
  token->data = nullptr;
  token->size = 0;
  token->buffer_requests = 0;
  token->acquired = false;

  token->state = new (std::nothrow) std::shared_ptr<LoanState>(std::move(state));
  if (token->state == nullptr) {
    Py_DECREF(reinterpret_cast<PyObject *>(token));
    PyErr_NoMemory();
    return nullptr;
  }

  WritableBufferSpan span{};
  const auto acquire_result = (*token->state)->acquire_writable_export(span);
  if (acquire_result != WritableExportAcquireResult::kOk) {
    set_acquire_error(acquire_result);
    Py_DECREF(reinterpret_cast<PyObject *>(token));
    return nullptr;
  }
  token->acquired = true;
  if (span.size > static_cast<std::size_t>(PY_SSIZE_T_MAX)) {
    PyErr_SetString(PyExc_OverflowError, "writable buffer size exceeds Py_ssize_t");
    Py_DECREF(reinterpret_cast<PyObject *>(token));
    return nullptr;
  }
  if (span.size != 0U && span.data == nullptr) {
    PyErr_SetString(PyExc_RuntimeError, "active Loan returned a null writable buffer");
    Py_DECREF(reinterpret_cast<PyObject *>(token));
    return nullptr;
  }

  token->data = span.data;
  token->size = static_cast<Py_ssize_t>(span.size);
  return reinterpret_cast<PyObject *>(token);
}

}  // namespace rclpy_agnocast
