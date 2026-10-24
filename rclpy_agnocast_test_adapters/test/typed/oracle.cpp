// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include <pybind11/pybind11.h>
#include <cstdint>
#include <string>
#include "native_fixture.hpp"

namespace
{
template<typename T>
pybind11::dict inspect(std::uintptr_t address, const pybind11::buffer & view)
{
  const auto & value = *reinterpret_cast<const T *>(address);
  typed_fixture::validate(value);
  pybind11::dict result;
  result["accounted_bytes"] = typed_fixture::accounted(value);
  result["native_data_address"] = typed_fixture::data_address(value);
  result["view_data_address"] = reinterpret_cast<std::uintptr_t>(view.request().ptr);
  return result;
}
}  // namespace

PYBIND11_MODULE(_typed_oracle, module)
{
  module.def(
    "inspect", [](const std::string & kind, std::uintptr_t address, const pybind11::buffer & view) {
      if (address == 0) {throw pybind11::value_error("native root address must be nonzero");}
      if (kind == "Image") {return inspect<typed_fixture::Image>(address, view);}
      if (kind == "PointCloud2") {return inspect<typed_fixture::PointCloud2>(address, view);}
      if (kind == "Containers") {return inspect<typed_fixture::Containers>(address, view);}
      throw pybind11::value_error("unknown native fixture type");
    }, "Compare a live root and view using independent native ROS member layout.");
}
