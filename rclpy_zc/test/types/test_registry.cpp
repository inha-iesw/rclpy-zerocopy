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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "rclpy_agnocast/types/adapter.hpp"
#include "rclpy_agnocast/types/field_ops.hpp"

namespace
{
using rclpy_agnocast::types::ADAPTER_ABI_VERSION;
using rclpy_agnocast::types::compiled_abi_tag;
using rclpy_agnocast::types::ProviderDescriptor;
using rclpy_agnocast::types::register_provider;
using rclpy_agnocast::types::registered_type;
using rclpy_agnocast::types::runtime_abi_tag;
using rclpy_agnocast::types::FieldOps;
using rclpy_agnocast::types::message_ops;
using rclpy_agnocast::types::Status;
using rclpy_agnocast::types::NativeEndpoint;
using rclpy_agnocast::types::error_status;
using rclpy_agnocast::types::ErrorCode;
using rclpy_agnocast::types::TypeOps;

const FieldOps root_ops = message_ops<std::uint64_t[2]>(nullptr, 0);
Status unused_factory(void *, const char *, std::size_t, NativeEndpoint *) noexcept
{return error_status(ErrorCode::kUnsupported, "registry fixture has no endpoint");}
const TypeOps operations{&root_ops, unused_factory};

ProviderDescriptor provider(const std::string & suffix)
{
  return {
    ADAPTER_ABI_VERSION, "provider_" + suffix, "adapter_" + suffix, compiled_abi_tag(),
    std::string(64, 'a'),
    {{"fixture/msg/" + suffix, std::string(64, 'b'), "[]", 16, 8, &operations}}
  };
}

TEST(TypeRegistry, RegisteredMetadataIsCopiedAndLookupCannotMutateTheRegistry)
{
  auto descriptor = provider("Valid");
  register_provider(descriptor, descriptor.manifest_digest);
  descriptor.types.front().native_size = 32;
  auto found = registered_type("fixture/msg/Valid");
  EXPECT_EQ(found.ros_type, "fixture/msg/Valid");
  EXPECT_EQ(found.schema_digest, std::string(64, 'b'));
  EXPECT_EQ(found.fields_json, "[]");
  EXPECT_EQ(found.native_size, 16U);
  EXPECT_EQ(found.native_alignment, 8U);
  found.native_size = 64;
  EXPECT_EQ(registered_type("fixture/msg/Valid").native_size, 16U);
}

TEST(TypeRegistry, CompiledAndRuntimeAbiTagsMatch)
{
  EXPECT_EQ(compiled_abi_tag(), runtime_abi_tag());
  EXPECT_FALSE(runtime_abi_tag().empty());
}

TEST(TypeRegistry, CHandshakeMatchesRuntimeIdentityAndKeepsItsStringStable)
{
  EXPECT_EQ(rclpy_agnocast_adapter_abi_version(), ADAPTER_ABI_VERSION);
  const char * const tag = rclpy_agnocast_adapter_build_abi();
  ASSERT_NE(tag, nullptr);
  EXPECT_STREQ(tag, runtime_abi_tag().c_str());
  EXPECT_EQ(rclpy_agnocast_adapter_build_abi(), tag);
}

TEST(TypeRegistry, RejectsAbiVersionMismatchBeforeRegistration)
{
  auto descriptor = provider("Version");
  ++descriptor.abi_version;
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  EXPECT_THROW(registered_type("fixture/msg/Version"), std::out_of_range);
}

TEST(TypeRegistry, RejectsBuildAbiMismatchBeforeRegistration)
{
  auto descriptor = provider("Build");
  descriptor.build_abi += "-other-compiler";
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  EXPECT_THROW(registered_type("fixture/msg/Build"), std::out_of_range);
}

TEST(TypeRegistry, RejectsManifestDigestMismatchBeforeRegistration)
{
  const auto descriptor = provider("Manifest");
  EXPECT_THROW(register_provider(descriptor, std::string(64, 'c')), std::invalid_argument);
  EXPECT_THROW(registered_type("fixture/msg/Manifest"), std::out_of_range);
}

TEST(TypeRegistry, ExactProviderRegistrationIsIdempotent)
{
  const auto descriptor = provider("Repeat");
  EXPECT_NO_THROW(register_provider(descriptor, descriptor.manifest_digest));
  EXPECT_NO_THROW(register_provider(descriptor, descriptor.manifest_digest));
  EXPECT_EQ(registered_type("fixture/msg/Repeat").native_size, 16U);
}

TEST(TypeRegistry, ChangedSchemaCannotReplaceAnExistingProvider)
{
  auto descriptor = provider("Schema");
  register_provider(descriptor, descriptor.manifest_digest);
  descriptor.types.front().schema_digest = std::string(64, 'd');
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  EXPECT_EQ(registered_type("fixture/msg/Schema").schema_digest, std::string(64, 'b'));
}

TEST(TypeRegistry, AnotherProviderCannotClaimAnExistingModule)
{
  const auto first = provider("ModuleFirst");
  register_provider(first, first.manifest_digest);
  auto second = provider("ModuleSecond");
  second.module = first.module;
  EXPECT_THROW(register_provider(second, second.manifest_digest), std::invalid_argument);
  EXPECT_THROW(registered_type("fixture/msg/ModuleSecond"), std::out_of_range);
}

TEST(TypeRegistry, AnotherProviderCannotClaimAnExistingTypeEvenWithIdenticalSchema)
{
  const auto first = provider("TypeFirst");
  register_provider(first, first.manifest_digest);
  auto second = provider("TypeSecond");
  second.types.front() = first.types.front();
  EXPECT_THROW(register_provider(second, second.manifest_digest), std::invalid_argument);
}

TEST(TypeRegistry, DuplicateTypesInOneBatchLeaveNoPartialRegistration)
{
  auto descriptor = provider("Duplicate");
  descriptor.types.push_back(descriptor.types.front());
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  EXPECT_THROW(registered_type("fixture/msg/Duplicate"), std::out_of_range);
  descriptor.types.pop_back();
  EXPECT_NO_THROW(register_provider(descriptor, descriptor.manifest_digest));
}

TEST(TypeRegistry, LateTypeCollisionDoesNotRegisterEarlierTypesOrReserveProvider)
{
  const auto first = provider("AtomicExisting");
  register_provider(first, first.manifest_digest);
  auto batch = provider("AtomicNew");
  batch.types.push_back(first.types.front());
  EXPECT_THROW(register_provider(batch, batch.manifest_digest), std::invalid_argument);
  EXPECT_THROW(registered_type("fixture/msg/AtomicNew"), std::out_of_range);
  batch.types.pop_back();
  EXPECT_NO_THROW(register_provider(batch, batch.manifest_digest));
}

TEST(TypeRegistry, MalformedMetadataIsRejectedWithoutReservingTheProvider)
{
  const auto valid = provider("Malformed");
  auto descriptor = valid;
  descriptor.provider.clear();
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  descriptor = valid;
  descriptor.module = "bad/module";
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  descriptor = valid;
  descriptor.types.clear();
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  descriptor = valid;
  descriptor.types.front().schema_digest = "not-a-digest";
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  descriptor = valid;
  descriptor.types.front().ros_type = "fixture/Bad";
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  descriptor = valid;
  descriptor.types.front().native_alignment = 3;
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  descriptor = valid;
  descriptor.types.front().native_size = 0;
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  descriptor = valid;
  descriptor.types.front().fields_json.clear();
  EXPECT_THROW(register_provider(descriptor, descriptor.manifest_digest), std::invalid_argument);
  EXPECT_THROW(registered_type("fixture/msg/Malformed"), std::out_of_range);
  EXPECT_NO_THROW(register_provider(valid, valid.manifest_digest));
}

TEST(TypeRegistry, MissingTypeErrorIdentifiesTheTypeAndBuildTimeRemedy)
{
  try {
    registered_type("fixture/msg/NotGenerated");
    FAIL() << "missing adapter was accepted";
  } catch (const std::out_of_range & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("fixture/msg/NotGenerated"), std::string::npos);
    EXPECT_NE(message.find("build an adapter"), std::string::npos);
  }
}

TEST(TypeRegistry, MissingOperationsInLastTypeLeavesNoPartialRegistration)
{
  auto batch = provider("OperationsFirst");
  auto last = provider("OperationsLast").types.front();
  last.ops = nullptr;
  batch.types.push_back(last);
  EXPECT_THROW(register_provider(batch, batch.manifest_digest), std::invalid_argument);
  EXPECT_THROW(registered_type("fixture/msg/OperationsFirst"), std::out_of_range);
  batch.types.back().ops = &operations;
  EXPECT_NO_THROW(register_provider(batch, batch.manifest_digest));
  EXPECT_EQ(registered_type("fixture/msg/OperationsFirst").ops, &operations);
}

TEST(TypeRegistry, ExecutableOperationsCannotBeReplacedUnderIdenticalMetadata)
{
  auto batch = provider("OperationsReplace");
  register_provider(batch, batch.manifest_digest);
  const TypeOps replacement{&root_ops, unused_factory};
  batch.types.front().ops = &replacement;
  EXPECT_THROW(register_provider(batch, batch.manifest_digest), std::invalid_argument);
  EXPECT_EQ(registered_type("fixture/msg/OperationsReplace").ops, &operations);
}
}  // namespace
