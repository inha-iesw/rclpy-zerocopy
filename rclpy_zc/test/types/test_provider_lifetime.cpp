// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0

#include <dlfcn.h>
#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "rclpy_agnocast/types/adapter.hpp"

namespace
{
using rclpy_agnocast::types::check_status;
using rclpy_agnocast::types::ErrorCode;
using rclpy_agnocast::types::NativeEndpoint;
using rclpy_agnocast::types::ProviderDescriptor;
using rclpy_agnocast::types::register_provider;
using rclpy_agnocast::types::registered_type;
using rclpy_agnocast::types::Status;

using Library = std::unique_ptr<void, int (*)(void *)>;

TEST(ProviderLifetime, RegistrationPinsAnOtherwiseUnloadableCallbackAndTranslatesItsError)
{
  {
    Library unregistered(dlopen(TEST_PROVIDER_LIBRARY, RTLD_NOW | RTLD_LOCAL), dlclose);
    ASSERT_NE(nullptr, unregistered.get()) << dlerror();
  }
  Library absent(dlopen(TEST_PROVIDER_LIBRARY, RTLD_NOW | RTLD_NOLOAD), dlclose);
  ASSERT_EQ(
    nullptr,
    absent.get()) << "negative control provider did not unload before registration";

  Library caller(dlopen(TEST_PROVIDER_LIBRARY, RTLD_NOW | RTLD_LOCAL), dlclose);
  ASSERT_NE(nullptr, caller.get()) << dlerror();
  using DescriptorFunction = const ProviderDescriptor * (*)();
  auto descriptor_function = reinterpret_cast<DescriptorFunction>(
    dlsym(caller.get(), "test_provider_descriptor"));
  ASSERT_NE(nullptr, descriptor_function) << dlerror();
  const auto * descriptor = descriptor_function();
  ASSERT_NE(nullptr, descriptor);
  ASSERT_EQ(1U, descriptor->types.size());
  const std::string type_name = descriptor->types.front().ros_type;
  ASSERT_NO_THROW(register_provider(*descriptor, descriptor->manifest_digest));
  ASSERT_EQ(0, dlclose(caller.release())) << dlerror();

  // Check residency before reading callback tables, so a missing pin is an assertion, not a crash.
  Library pinned(dlopen(TEST_PROVIDER_LIBRARY, RTLD_NOW | RTLD_NOLOAD), dlclose);
  ASSERT_NE(nullptr, pinned.get()) << "registration did not preserve the provider after dlclose";
  ASSERT_EQ(0, dlclose(pinned.release())) << dlerror();
  const auto found = registered_type(type_name);
  ASSERT_NE(nullptr, found.ops);
  ASSERT_NE(nullptr, found.ops->create);

  int node_token = 0;
  NativeEndpoint endpoint;
  Status status;
  ASSERT_NO_THROW(status = found.ops->create(&node_token, "/lifetime_test", 1, &endpoint));
  EXPECT_EQ(ErrorCode::kRuntime, status.code);
  EXPECT_STREQ("provider-local factory failure", status.message);
  EXPECT_EQ(nullptr, endpoint.object);
  try {
    check_status(status);
    FAIL() << "runtime accepted the provider failure status";
  } catch (const std::runtime_error & error) {
    EXPECT_EQ(typeid(std::runtime_error), typeid(error));
    EXPECT_STREQ("provider-local factory failure", error.what());
  }
}
}  // namespace
