#include <dirent.h>
#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include "../support.hpp"
#include "producer_failure.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

namespace
{
using g3_cleanup::Operation;
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;

int open_fd_count()
{
  DIR * directory = opendir("/proc/self/fd");
  if (!directory) {return -1;}
  int count = 0;
  while (const auto * entry = readdir(directory)) {
    if (entry->d_name[0] != '.') {++count;}
  }
  closedir(directory);
  return count;
}

void expect_current_context(CUcontext expected)
{
  CUcontext current = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&current), CUDA_SUCCESS);
  EXPECT_EQ(current, expected);
}

struct FailureCase
{
  const char * name;
  Operation operation;
  unsigned sockets = 1;
  unsigned occurrence = 2;
};

class InitialisationFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(::testing::UnitTest::GetInstance()->test_to_run_count(), 1);
    ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
    stream_ = std::make_unique<g3_test::Stream>();
    // Warm CUDA and its driver FDs before measuring this pool's rollback.
    ASSERT_TRUE(manager_.initialise());
    ASSERT_TRUE(manager_.try_close());
    g3_cleanup::record_owned_names();
    before_ = g3_cleanup::counts();
    descriptors_ = open_fd_count();
    ASSERT_GT(descriptors_, 0);
    ASSERT_EQ(cuCtxGetCurrent(&previous_context_), CUDA_SUCCESS);
  }

  void TearDown() override {EXPECT_TRUE(g3_cleanup::unlink_owned_names());}

  void expect_failed_start(Operation operation, unsigned metadata, unsigned sockets)
  {
    EXPECT_FALSE(manager_.initialise());
    EXPECT_FALSE(manager_.is_initialised());
    EXPECT_FALSE(manager_.poisoned());
    EXPECT_EQ(g3_cleanup::injections(operation), 1u);
    EXPECT_EQ(g3_cleanup::owned_metadata_count(), metadata);
    EXPECT_EQ(g3_cleanup::owned_socket_count(), sockets);
    expect_baseline();
    after_ = g3_cleanup::counts();
  }

  unsigned delta(Operation operation) const
  {
    return g3_cleanup::count(after_, operation) - g3_cleanup::count(before_, operation);
  }

  void expect_restart(Operation operation)
  {
    // A healthy failed start must release the epoch as well as names/FDs.
    Manager replacement({2, 64, 0});
    ASSERT_TRUE(replacement.initialise());
    auto first = replacement.acquire_for_publish(stream_->driver());
    auto second = replacement.acquire_for_publish(stream_->driver());
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_NE(first->device_ptr(), second->device_ptr());
    ASSERT_TRUE(first->prepare_publish(stream_->driver()));
    ASSERT_TRUE(second->prepare_publish(stream_->driver()));
    ASSERT_EQ(cudaStreamSynchronize(stream_->value), cudaSuccess);
    EXPECT_TRUE(replacement.try_close());
    ASSERT_TRUE(manager_.initialise());
    EXPECT_TRUE(manager_.try_close());
    EXPECT_FALSE(manager_.poisoned());
    EXPECT_EQ(g3_cleanup::injections(operation), 1u);
    expect_baseline();
  }

private:
  void expect_baseline()
  {
    EXPECT_TRUE(g3_cleanup::owned_names_absent());
    EXPECT_EQ(open_fd_count(), descriptors_);
    // This checks top-of-stack identity; push/pop counts detect same-context leaks.
    expect_current_context(previous_context_);
  }

  // The manager must be destroyed while its stream is still valid.
  std::unique_ptr<g3_test::Stream> stream_;
  Manager manager_{{2, 64, 0}};
  g3_cleanup::Counts before_{};
  g3_cleanup::Counts after_{};
  int descriptors_ = 0;
  CUcontext previous_context_ = nullptr;
};

class ProducerInitialisation :
  public InitialisationFixture, public ::testing::WithParamInterface<FailureCase> {};

TEST_P(ProducerInitialisation, SecondBlockFailureRollsBackAndAllowsRetry)
{
  const auto & failure = GetParam();
  const auto operation = failure.operation;
  g3_cleanup::arm(operation, failure.occurrence, false, false);
  expect_failed_start(operation, 2, failure.sockets);
  if (operation == Operation::ContextPush) {
    // Rollback also needs context activation. One ready event existed before
    // the second event's activation failed.
    EXPECT_GE(delta(operation), failure.occurrence);
    EXPECT_EQ(delta(Operation::EventCreate), 1u);
    EXPECT_EQ(delta(Operation::IpcGetEventHandle), 1u);
  } else {
    EXPECT_EQ(delta(operation), failure.occurrence);
  }
  // Successful allocations must each have a matching successful release.
  for (const auto & pair : {
      std::pair{Operation::MemAddressReserve, Operation::MemAddressFree},
      std::pair{Operation::MemCreate, Operation::MemRelease},
      std::pair{Operation::MemMap, Operation::MemUnmap},
      std::pair{Operation::EventCreate, Operation::EventDestroy},
      std::pair{Operation::ContextPush, Operation::ContextPop},
      std::pair{Operation::PrimaryRetain, Operation::PrimaryRelease}})
  {
    EXPECT_EQ(delta(pair.first) - g3_cleanup::injections(pair.first), delta(pair.second))
      << static_cast<unsigned>(pair.first);
  }
  expect_restart(operation);
}

INSTANTIATE_TEST_SUITE_P(
  Allocation, ProducerInitialisation,
  ::testing::Values(
    FailureCase{"MemAddressReserve", Operation::MemAddressReserve},
    FailureCase{"MemCreate", Operation::MemCreate},
    FailureCase{"MemMap", Operation::MemMap},
    FailureCase{"MemSetAccess", Operation::MemSetAccess},
    FailureCase{"MemExport", Operation::MemExport},
    FailureCase{"EventCreate", Operation::EventCreate, 2},
    FailureCase{"IpcGetEventHandle", Operation::IpcGetEventHandle, 2},
    FailureCase{"Socket", Operation::Socket},
    FailureCase{"Bind", Operation::Bind},
    FailureCase{"Listen", Operation::Listen, 2},
    FailureCase{"EventContextPush", Operation::ContextPush, 2, 3}),
  [](const ::testing::TestParamInfo<FailureCase> & info) {return info.param.name;});

struct ContextFailureCase
{
  const char * name;
  Operation operation;
  unsigned occurrence;
  unsigned metadata;
  unsigned releases;
};

class ProducerContextInitialisation :
  public InitialisationFixture, public ::testing::WithParamInterface<ContextFailureCase> {};

TEST_P(ProducerContextInitialisation, EarlyFailureReturnsEpochAndAllowsRetry)
{
  const auto & failure = GetParam();
  const auto operation = failure.operation;
  g3_cleanup::arm(operation, failure.occurrence, false, false);
  expect_failed_start(operation, failure.metadata, 0);
  EXPECT_EQ(delta(Operation::PrimaryRetain), 1u);
  EXPECT_EQ(delta(Operation::MemAddressReserve), 0u);
  EXPECT_EQ(delta(Operation::MemCreate), 0u);
  EXPECT_EQ(delta(Operation::EventCreate), 0u);
  EXPECT_EQ(delta(Operation::PrimaryRelease), failure.releases);
  EXPECT_EQ(delta(Operation::ContextPush) - g3_cleanup::injections(Operation::ContextPush),
    delta(Operation::ContextPop));
  if (operation == Operation::PrimaryRetain) {EXPECT_EQ(delta(Operation::ContextPush), 0u);}

  expect_restart(operation);
}

INSTANTIATE_TEST_SUITE_P(
  Context, ProducerContextInitialisation,
  ::testing::Values(
    ContextFailureCase{"PrimaryRetain", Operation::PrimaryRetain, 1, 0, 0},
    ContextFailureCase{"AllocationContextPush", Operation::ContextPush, 1, 2, 1}),
  [](const ::testing::TestParamInfo<ContextFailureCase> & info) {return info.param.name;});
}  // namespace
