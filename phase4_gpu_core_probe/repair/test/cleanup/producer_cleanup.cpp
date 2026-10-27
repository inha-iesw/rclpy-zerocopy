#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <memory>

#include "producer_failure.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

namespace
{
using g3_cleanup::Operation;
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;

struct FailureCase
{
  const char * name;
  Operation operation;
  bool rollback = false;
  unsigned occurrence = 1;
  bool pop_mismatch = false;
};

class RepairedProducerCleanup : public ::testing::TestWithParam<FailureCase> {};

struct OwnedNames
{
  OwnedNames() {g3_cleanup::record_owned_names();}
  ~OwnedNames() {EXPECT_TRUE(g3_cleanup::unlink_owned_names());}
};

TEST_P(RepairedProducerCleanup, FailurePoisonsEpoch)
{
  // A poisoned epoch is deliberately process-wide and cannot be reset by a
  // fixture. CTest must invoke each parameter with its own --gtest_filter.
  ASSERT_EQ(::testing::UnitTest::GetInstance()->test_to_run_count(), 1);
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  OwnedNames owned_names;
  const auto & failure = GetParam();
  auto manager = std::make_unique<Manager>(Manager::Config{1, 64, 0});
  if (!failure.rollback) {
    ASSERT_TRUE(manager->initialise());
  }
  const auto before = g3_cleanup::counts();
  g3_cleanup::arm(
    failure.operation, failure.occurrence, failure.rollback, failure.pop_mismatch);
  if (failure.rollback) {
    EXPECT_FALSE(manager->initialise());
    EXPECT_EQ(g3_cleanup::injections(Operation::MemSetAccess), 1u);
  } else {
    EXPECT_FALSE(manager->try_close());
  }
  EXPECT_TRUE(manager->poisoned());
  EXPECT_EQ(g3_cleanup::injections(failure.operation), 1u);
  const auto after = g3_cleanup::counts();
  EXPECT_EQ(
    g3_cleanup::count(after, failure.operation) - g3_cleanup::count(before, failure.operation),
    failure.occurrence);
  EXPECT_EQ(g3_cleanup::owned_metadata_count(), 1u);
  EXPECT_EQ(g3_cleanup::owned_socket_count(), failure.rollback ? 0u : 1u);

  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    EXPECT_FALSE(manager->try_close()) << attempt;
    EXPECT_TRUE(manager->poisoned()) << attempt;
  }
  EXPECT_EQ(g3_cleanup::counts(), after);
  manager.reset();
  EXPECT_EQ(g3_cleanup::counts(), after);
  for (unsigned attempt = 0; attempt < 20; ++attempt) {
    Manager replacement({1, 64, 0});
    EXPECT_FALSE(replacement.initialise()) << attempt;
    EXPECT_TRUE(replacement.poisoned()) << attempt;
  }
  // Includes allocation and primary retain calls: replacements must not grow
  // resources even when the injected operation would succeed on another call.
  EXPECT_EQ(g3_cleanup::counts(), after);
}

INSTANTIATE_TEST_SUITE_P(
  Cleanup, RepairedProducerCleanup,
  ::testing::Values(
    FailureCase{"EventDestroy", Operation::EventDestroy},
    FailureCase{"MemUnmap", Operation::MemUnmap},
    FailureCase{"MemAddressFree", Operation::MemAddressFree},
    FailureCase{"MemRelease", Operation::MemRelease},
    FailureCase{"EventQuery", Operation::EventQuery},
    FailureCase{"ContextPush", Operation::ContextPush},
    FailureCase{"RollbackMemUnmap", Operation::MemUnmap, true},
    FailureCase{"RollbackMemAddressFree", Operation::MemAddressFree, true},
    FailureCase{"RollbackMemRelease", Operation::MemRelease, true},
    FailureCase{"FinalContextPop", Operation::ContextPop, false, 3},
    FailureCase{"FinalContextPopMismatch", Operation::ContextPop, false, 3, true}),
  [](const ::testing::TestParamInfo<FailureCase> & info) {return info.param.name;});
}  // namespace
