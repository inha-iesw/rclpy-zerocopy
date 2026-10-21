#pragma once

namespace pybind11
{
class module_;
}

namespace rclpy_agnocast
{

struct PublisherWorkerResult;

/**
 * @brief Register publisher, Loan, and publisher-test bindings on a native module.
 *
 * The caller retains module ownership and must hold the GIL. Runtime bindings may return the
 * registered publisher and Loan holder types after this function completes.
 *
 * @param[in,out] module Borrowed pybind11 module wrapper kept alive by the caller.
 * @return Nothing; registration failures propagate as Python or C++ exceptions.
 * @pre The writable buffer-export type has already been registered on `module`.
 */
void bind_publisher(pybind11::module_ & module);

namespace binding_detail
{

/**
 * @brief Translate one completed worker command into a Python-compatible C++ exception.
 *
 * Stored backend exceptions are rethrown before deterministic worker errors are mapped.
 *
 * @param[in] operation Non-null stable operation label used in the error message.
 * @param[in] result Completed synchronous worker result; ownership remains with the caller.
 * @return Nothing when `result` is successful.
 * @throws std::exception The stored backend failure, or std::runtime_error for worker errors.
 */
void throw_worker_error(const char * operation, const PublisherWorkerResult & result);

}  // namespace binding_detail

}  // namespace rclpy_agnocast
