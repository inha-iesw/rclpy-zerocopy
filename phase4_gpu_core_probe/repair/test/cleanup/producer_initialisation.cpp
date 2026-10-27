#include <dirent.h>
#include <gtest/gtest.h>

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

struct FailureCase
{
  const char * name;
  Operation operation;
};

class ProducerInitialisation : public ::testing::TestWithParam<FailureCase> {};

TEST_P(ProducerInitialisation, SecondBlockFailureRollsBackAndAllowsRetry)
{
  ASSERT_EQ(::testing::UnitTest::GetInstance()->test_to_run_count(), 1);
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  g3_test::Stream stream;
  Manager manager({2, 64, 0});
  // Warm CUDA and its driver FDs before measuring this pool's rollback.
  ASSERT_TRUE(manager.initialise());
  ASSERT_TRUE(manager.try_close());
  struct OwnedNames
  {
    OwnedNames() {g3_cleanup::record_owned_names();}
    ~OwnedNames() {EXPECT_TRUE(g3_cleanup::unlink_owned_names());}
  } owned_names;
  const auto before = g3_cleanup::counts();
  const int descriptors = open_fd_count();
  ASSERT_GT(descriptors, 0);
  const auto operation = GetParam().operation;
  g3_cleanup::arm(operation, 2, false, false);
  EXPECT_FALSE(manager.initialise());
  EXPECT_FALSE(manager.is_initialised());
  EXPECT_FALSE(manager.poisoned());
  EXPECT_EQ(g3_cleanup::injections(operation), 1u);
  EXPECT_EQ(g3_cleanup::owned_metadata_count(), 2u);
  EXPECT_EQ(g3_cleanup::owned_socket_count(),
    operation == Operation::EventCreate || operation == Operation::IpcGetEventHandle ? 2u : 1u);
  EXPECT_TRUE(g3_cleanup::owned_names_absent());
  EXPECT_EQ(open_fd_count(), descriptors);
  const auto after = g3_cleanup::counts();
  const auto delta = [&](Operation op) {
      return g3_cleanup::count(after, op) - g3_cleanup::count(before, op);
    };
  EXPECT_EQ(delta(operation), 2u);
  // Successful allocations must each have a matching successful release.
  for (const auto & pair : {
      std::pair{Operation::MemAddressReserve, Operation::MemAddressFree},
      std::pair{Operation::MemCreate, Operation::MemRelease},
      std::pair{Operation::MemMap, Operation::MemUnmap},
      std::pair{Operation::EventCreate, Operation::EventDestroy}})
  {
    EXPECT_EQ(delta(pair.first) - g3_cleanup::injections(pair.first), delta(pair.second))
      << static_cast<unsigned>(pair.first);
  }
  // A healthy failed start must release the process epoch as well as names/FDs.
  Manager replacement({2, 64, 0});
  ASSERT_TRUE(replacement.initialise());
  auto first = replacement.acquire_for_publish(stream.driver());
  auto second = replacement.acquire_for_publish(stream.driver());
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_NE(first->device_ptr(), second->device_ptr());
  ASSERT_TRUE(first->prepare_publish(stream.driver()));
  ASSERT_TRUE(second->prepare_publish(stream.driver()));
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(replacement.try_close());
  ASSERT_TRUE(manager.initialise());
  EXPECT_TRUE(manager.try_close());
  EXPECT_FALSE(manager.poisoned());
  EXPECT_EQ(g3_cleanup::injections(operation), 1u);
  EXPECT_TRUE(g3_cleanup::owned_names_absent());
  EXPECT_EQ(open_fd_count(), descriptors);
}

INSTANTIATE_TEST_SUITE_P(
  Allocation, ProducerInitialisation,
  ::testing::Values(
    FailureCase{"MemAddressReserve", Operation::MemAddressReserve},
    FailureCase{"MemCreate", Operation::MemCreate},
    FailureCase{"MemMap", Operation::MemMap},
    FailureCase{"MemSetAccess", Operation::MemSetAccess},
    FailureCase{"MemExport", Operation::MemExport},
    FailureCase{"EventCreate", Operation::EventCreate},
    FailureCase{"IpcGetEventHandle", Operation::IpcGetEventHandle}),
  [](const ::testing::TestParamInfo<FailureCase> & info) {return info.param.name;});
}  // namespace
