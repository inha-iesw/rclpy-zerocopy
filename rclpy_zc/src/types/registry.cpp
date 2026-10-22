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

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "rclpy_agnocast/types/adapter.hpp"

extern "C" const char * rclpy_agnocast_adapter_build_abi()
{
  static const std::string tag = rclpy_agnocast::types::compiled_abi_tag();
  return tag.c_str();
}

extern "C" std::uint32_t rclpy_agnocast_adapter_abi_version()
{
  return rclpy_agnocast::types::ADAPTER_ABI_VERSION;
}

namespace rclpy_agnocast
{
namespace types
{
namespace
{

struct Registry
{
  std::mutex mutex;
  std::unordered_map<std::string, ProviderDescriptor> providers;
  std::unordered_map<std::string, TypeDescriptor> types;
};

Registry & registry()
{
  static Registry instance;
  return instance;
}

bool identifier(const std::string & value)
{
  const auto letter = [](const char c) {
      return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    };
  return !value.empty() && letter(value.front()) &&
         std::all_of(
    value.begin(), value.end(), [&](const char c) {
      return letter(c) || (c >= '0' && c <= '9');
    });
}

bool module_name(const std::string & value)
{
  std::size_t start = 0;
  while (true) {
    const auto end = value.find('.', start);
    if (!identifier(value.substr(start, end - start))) {
      return false;
    }
    if (end == std::string::npos) {
      return true;
    }
    start = end + 1;
  }
}

bool ros_type_name(const std::string & value)
{
  const auto separator = value.find("/msg/");
  return separator != std::string::npos && identifier(value.substr(0, separator)) &&
         identifier(value.substr(separator + 5));
}

bool digest(const std::string & value)
{
  return value.size() == 64 && std::all_of(
    value.begin(), value.end(), [](const char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}

bool same_type(const TypeDescriptor & left, const TypeDescriptor & right)
{
  return std::tie(
    left.ros_type, left.schema_digest, left.fields_json, left.native_size, left.native_alignment) ==
         std::tie(
    right.ros_type, right.schema_digest, right.fields_json,
    right.native_size, right.native_alignment);
}

bool same_provider(const ProviderDescriptor & left, const ProviderDescriptor & right)
{
  return std::tie(
    left.abi_version, left.provider, left.module, left.build_abi, left.manifest_digest) ==
         std::tie(
    right.abi_version, right.provider, right.module, right.build_abi, right.manifest_digest) &&
         left.types.size() == right.types.size() &&
         std::equal(left.types.begin(), left.types.end(), right.types.begin(), same_type);
}

void validate(const ProviderDescriptor & descriptor, const std::string & expected_manifest_digest)
{
  if (descriptor.abi_version != ADAPTER_ABI_VERSION) {
    throw std::invalid_argument("adapter ABI version mismatch: " + descriptor.provider);
  }
  if (descriptor.build_abi != runtime_abi_tag()) {
    throw std::invalid_argument("adapter build ABI mismatch: " + descriptor.provider);
  }
  if (!digest(descriptor.manifest_digest) ||
    descriptor.manifest_digest != expected_manifest_digest)
  {
    throw std::invalid_argument("adapter manifest digest mismatch: " + descriptor.provider);
  }
  if (!identifier(descriptor.provider) || !module_name(descriptor.module) ||
    descriptor.types.empty())
  {
    throw std::invalid_argument("invalid adapter provider metadata: " + descriptor.provider);
  }
  std::unordered_set<std::string> names;
  for (const auto & type : descriptor.types) {
    if (!ros_type_name(type.ros_type) || !digest(type.schema_digest) || type.fields_json.empty() ||
      type.native_size == 0 || type.native_alignment == 0 ||
      (type.native_alignment & (type.native_alignment - 1)) != 0 ||
      type.native_size % type.native_alignment != 0)
    {
      throw std::invalid_argument("invalid adapter type metadata: " + type.ros_type);
    }
    if (!names.insert(type.ros_type).second) {
      throw std::invalid_argument("duplicate adapter type in provider: " + type.ros_type);
    }
  }
}

}  // namespace

std::string runtime_abi_tag()
{
  return compiled_abi_tag();
}

void register_provider(
  const ProviderDescriptor & descriptor, const std::string & expected_manifest_digest)
{
  validate(descriptor, expected_manifest_digest);
  auto & state = registry();
  std::lock_guard<std::mutex> lock(state.mutex);
  const auto existing = state.providers.find(descriptor.provider);
  if (existing != state.providers.end()) {
    if (same_provider(existing->second, descriptor)) {
      return;
    }
    throw std::invalid_argument("adapter provider collision: " + descriptor.provider);
  }
  for (const auto & provider : state.providers) {
    if (provider.second.module == descriptor.module) {
      throw std::invalid_argument("adapter module collision: " + descriptor.module);
    }
  }
  for (const auto & type : descriptor.types) {
    if (state.types.count(type.ros_type)) {
      throw std::invalid_argument("adapter type collision: " + type.ros_type);
    }
  }
  // Allocation failure during either copy or insertion cannot expose a partial batch.
  auto providers = state.providers;
  auto types = state.types;
  providers.emplace(descriptor.provider, descriptor);
  for (const auto & type : descriptor.types) {
    types.emplace(type.ros_type, type);
  }
  state.providers.swap(providers);
  state.types.swap(types);
}

TypeDescriptor registered_type(const std::string & ros_type)
{
  auto & state = registry();
  std::lock_guard<std::mutex> lock(state.mutex);
  const auto found = state.types.find(ros_type);
  if (found == state.types.end()) {
    throw std::out_of_range("no adapter registered for " + ros_type + "; build an adapter first");
  }
  return found->second;
}

}  // namespace types
}  // namespace rclpy_agnocast
