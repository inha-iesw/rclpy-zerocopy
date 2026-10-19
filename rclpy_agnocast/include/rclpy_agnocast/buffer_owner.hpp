#pragma once

#include <Python.h>

#include <memory>

namespace rclpy_agnocast
{

class LoanState;

/**
 * @brief Register the private writable-buffer export type on a Python module.
 *
 * The registered type cannot be constructed from Python. Call this while the GIL is held before
 * creating writable export tokens. Repeated registration in the same interpreter is supported.
 *
 * @param[in,out] module Non-null borrowed reference to the owning Python module.
 * @return Zero on success, or `-1` with a Python exception set on failure.
 * @note Registration is process-global for one CPython interpreter. Subinterpreters and
 * finalize/reinitialize cycles are not supported.
 */
int register_writable_buffer_export_type(PyObject * module) noexcept;

/**
 * @brief Create one counted writable-buffer export token for a Loan.
 *
 * Each returned object owns a strong reference to `state`. The corresponding writable export is
 * released exactly once when the token is deallocated, after all compliant buffer consumers and
 * derived aliases have released their references.
 *
 * @param[in] state Non-null shared ownership of the active Loan state.
 * @return A caller-owned new Python reference, or `nullptr` with a Python exception set.
 * @pre The GIL is held and register_writable_buffer_export_type() has succeeded.
 */
[[nodiscard]] PyObject * create_writable_buffer_export(
  std::shared_ptr<LoanState> state) noexcept;

}  // namespace rclpy_agnocast
