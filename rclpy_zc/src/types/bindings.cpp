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

#include <string>

#include "rclpy_agnocast/types/adapter.hpp"

namespace py = pybind11;

namespace rclpy_agnocast
{
namespace types
{
void bind_root(py::module_ & module);
}

void bind_types(py::module_ & module)
{
  if (::rclpy_agnocast_adapter_abi_version() != types::ADAPTER_ABI_VERSION ||
    std::string(::rclpy_agnocast_adapter_build_abi()) != types::compiled_abi_tag())
  {
    throw py::import_error("runtime adapter ABI mismatch");
  }
  module.def(
    "_adapter_abi", []() {
      py::dict result;
      result["version"] = ::rclpy_agnocast_adapter_abi_version();
      result["build"] = ::rclpy_agnocast_adapter_build_abi();
      return result;
    });
  module.def(
    "_registered_type", [](const std::string & name) {
      const auto descriptor = types::registered_type(name);
      py::dict result;
      result["ros_type"] = descriptor.ros_type;
      result["schema_digest"] = descriptor.schema_digest;
      result["fields_json"] = descriptor.fields_json;
      result["native_size"] = descriptor.native_size;
      result["native_alignment"] = descriptor.native_alignment;
      return result;
    }, py::arg("ros_type"));
  types::bind_root(module);
}

}  // namespace rclpy_agnocast
