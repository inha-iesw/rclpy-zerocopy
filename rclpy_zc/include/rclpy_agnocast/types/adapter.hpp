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

#ifndef RCLPY_AGNOCAST__TYPES__ADAPTER_HPP_
#define RCLPY_AGNOCAST__TYPES__ADAPTER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief Expose the registry ABI tag without passing standard library objects across DSOs.
 *
 * Plugins and the runtime extension must compare this tag with their own
 * compiled_abi_tag() before invoking any C++ registry API. The shared registry
 * library remains loaded for the process lifetime.
 *
 * @return Non-null, null-terminated immutable string owned by the shared library;
 *   the caller must not free it and its address remains stable for the process lifetime.
 */
extern "C" const char * rclpy_agnocast_adapter_build_abi();

/**
 * @brief Expose the descriptor version before crossing the C++ registration boundary.
 *
 * @return Contract version compiled into the shared registry library.
 */
extern "C" std::uint32_t rclpy_agnocast_adapter_abi_version();

namespace rclpy_agnocast
{
namespace types
{

/** @brief Version of the shared descriptor and registration contract. */
constexpr std::uint32_t ADAPTER_ABI_VERSION = 1;

/**
 * @brief Compute the ABI fingerprint of the including translation unit.
 *
 * Internal linkage prevents symbol interposition from replacing a plugin's tag with
 * the runtime's tag. The exported CMake target supplies the runtime build's ROS
 * dependency identity; it does not independently identify the caller's ROS headers.
 * The types digest identifies the runtime's published headers. Generated providers and their
 * probes independently hash headers exported by their selected agnocastlib target; ordinary
 * registry consumers inherit the runtime's Agnocast identity. Rebuild the runtime whenever its
 * published headers change, including symlink installs, before rebuilding companion providers.
 *
 * @return Compiler, standard library, architecture, ROS and provider header build identity.
 */
static inline std::string compiled_abi_tag()
{
  std::string tag = std::string("compiler=") + __VERSION__;
#ifdef __GLIBCXX__
  tag += ";libstdc++=" + std::to_string(__GLIBCXX__);
#elif defined(_LIBCPP_VERSION)
  tag += ";libc++=" + std::to_string(_LIBCPP_VERSION);
#else
  tag += ";stdlib=unknown";
#endif
#ifdef _GLIBCXX_USE_CXX11_ABI
  tag += ";cxx11abi=" + std::to_string(_GLIBCXX_USE_CXX11_ABI);
#endif
#ifdef _GLIBCXX_DEBUG
  tag += ";libstdc++debug=1";
#else
  tag += ";libstdc++debug=0";
#endif
  tag += ";cxx=" + std::to_string(__cplusplus);
  tag += ";pointer=" + std::to_string(sizeof(void *) * 8);
#if defined(__x86_64__)
  tag += ";arch=x86_64";
#elif defined(__aarch64__)
  tag += ";arch=aarch64";
#elif defined(__i386__)
  tag += ";arch=i386";
#elif defined(__arm__)
  tag += ";arch=arm";
#else
  tag += ";arch=unknown";
#endif
#ifdef RCLPY_AGNOCAST_ROS_ABI
  tag += ";ros=" RCLPY_AGNOCAST_ROS_ABI;
#else
  tag += ";ros=unspecified";
#endif
  // The selected macro is separate so a provider never races duplicate inherited -D values.
#ifdef RCLPY_AGNOCAST_TYPES_HEADERS_SHA256
  tag += ";types=" RCLPY_AGNOCAST_TYPES_HEADERS_SHA256;
#else
  tag += ";types=unspecified";
#endif
#ifdef RCLPY_AGNOCAST_SELECTED_AGNOCAST_HEADERS_SHA256
  tag += ";agnocast=" RCLPY_AGNOCAST_SELECTED_AGNOCAST_HEADERS_SHA256;
#elif defined(RCLPY_AGNOCAST_RUNTIME_AGNOCAST_HEADERS_SHA256)
  tag += ";agnocast=" RCLPY_AGNOCAST_RUNTIME_AGNOCAST_HEADERS_SHA256;
#else
  tag += ";agnocast=unspecified";
#endif
  return tag;
}

/**
 * @brief Return the ABI fingerprint compiled into the shared registry library.
 *
 * @return The runtime's fingerprint, independent of the calling plugin's build.
 */
std::string runtime_abi_tag();

/**
 * @brief Generated schema metadata for one native ROS message type.
 *
 * Strings and metadata are copied by registration; no plugin-owned pointers survive.
 * fields_json is the generator's canonical JSON field list, not executable operations.
 */
struct TypeDescriptor
{
  std::string ros_type;  ///< Fully qualified package/msg/Type identity.
  std::string schema_digest;  ///< Lowercase SHA-256 of the transitive schema.
  std::string fields_json;  ///< Canonical JSON field metadata from the build generator.
  std::size_t native_size;  ///< sizeof of the generated native C++ message.
  std::size_t native_alignment;  ///< alignof of the generated native C++ message.
};

/**
 * @brief Immutable identity of one generated adapter provider and its complete type batch.
 *
 * A provider name, Python module, and ROS type may each belong to only one registration.
 * The manifest digest binds the installed discovery metadata to the compiled plugin.
 */
struct ProviderDescriptor
{
  std::uint32_t abi_version;  ///< Descriptor contract version.
  std::string provider;  ///< Provider package identity.
  std::string module;  ///< Dotted import path of the generated Python extension.
  std::string build_abi;  ///< compiled_abi_tag() evaluated by the provider.
  std::string manifest_digest;  ///< Lowercase SHA-256 of the canonical manifest payload.
  std::vector<TypeDescriptor> types;  ///< Complete batch registered atomically.
};

/**
 * @brief Validate and atomically register one generated adapter provider.
 *
 * Exact repeated registration is idempotent. Any changed identity, ABI, metadata,
 * manifest mismatch, or provider/module/type collision is rejected before mutation.
 * Registration and lookup share a process-wide mutex in this shared library.
 *
 * @param[in] descriptor Caller-owned generated metadata, copied on success.
 * @param[in] expected_manifest_digest Digest read from the installed manifest.
 * @throws std::invalid_argument If the provider is incompatible, malformed, or conflicts.
 * @throws std::bad_alloc If copying fails; existing registrations remain unchanged.
 */
void register_provider(
  const ProviderDescriptor & descriptor, const std::string & expected_manifest_digest);

/**
 * @brief Look up a previously registered native type without exposing registry storage.
 *
 * @param[in] ros_type Fully qualified package/msg/Type identity.
 * @return Independent metadata copy, valid for the caller's chosen lifetime.
 * @throws std::out_of_range If no generated adapter registered this type.
 */
TypeDescriptor registered_type(const std::string & ros_type);

}  // namespace types
}  // namespace rclpy_agnocast

#endif  // RCLPY_AGNOCAST__TYPES__ADAPTER_HPP_
