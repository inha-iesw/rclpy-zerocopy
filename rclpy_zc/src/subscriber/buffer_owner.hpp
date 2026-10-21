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

#pragma once

#include <Python.h>
#include <memory>

#include "lease.hpp"

namespace rclpy_agnocast
{
/**
 * @brief Register the private, non-constructible read-only exporter. Requires the GIL.
 * Subinterpreters and interpreter finalize/reinitialize cycles are not supported.
 */
int register_readonly_buffer_export_type(PyObject * module) noexcept;
/** @brief Return a new exporter reference retaining owner, or nullptr with a Python error. */
PyObject * create_readonly_buffer_export(std::shared_ptr<LeaseBufferOwner> owner) noexcept;
}  // namespace rclpy_agnocast
