#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

namespace
{
using Ref = ros2_cuda_ipc_core::buffer_metadata::BufferRef;
using Metadata = ros2_cuda_ipc_core::buffer_metadata::BufferMetadata;

class Mapping : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static unsigned counter = 0;
    name = "/g3_probe_" + std::to_string(getpid()) + "_" + std::to_string(counter++);
    mapping = Metadata::create(name);
    ASSERT_TRUE(mapping);
  }

  void TearDown() override
  {
    mapping.reset();
    (void)shm_unlink(name.c_str());
  }

  std::string name;
  std::shared_ptr<Metadata> mapping;
};

TEST_F(Mapping, PausedReservationPrefixAdmitsPreviousGenerationReader)
{
  const auto initial = Ref::reserve_for_publish(mapping);
  ASSERT_TRUE(initial);
  ASSERT_TRUE(Ref::commit_publish(mapping, initial->uid));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  auto * metadata = mapping->metadata();

  // White-box interleaving simulation, not a full concurrent API test:
  // execute the exact reservation CAS in buffer_ref.cpp:126, then pause
  // before its UID store at :138 and call the real reader acquisition.
  uint32_t expected = 0;
  ASSERT_TRUE(metadata->refcount.compare_exchange_strong(
      expected, 1, std::memory_order_acq_rel, std::memory_order_acquire));
  const auto next_uid = initial->uid + 1;
  {
    auto old_reader = Ref::acquire(mapping, initial->uid);
    ASSERT_TRUE(old_reader.valid());
    metadata->uid.store(next_uid, std::memory_order_release);
    metadata->publish_timestamp_us.store(0, std::memory_order_release);

    EXPECT_TRUE(old_reader.valid());
    EXPECT_EQ(Ref::current_refcount(mapping), 2u);
    RecordProperty("method", "white_box_paused_reservation_prefix");
    RecordProperty("hazard", "old_reader_coexists_with_new_writer_generation");
  }
  EXPECT_TRUE(Ref::cancel_publish(mapping, next_uid));
}

TEST_F(Mapping, AbruptConsumerExitLeavesReferenceAndPreventsReuse)
{
  const auto initial = Ref::reserve_for_publish(mapping);
  ASSERT_TRUE(initial);
  ASSERT_TRUE(Ref::commit_publish(mapping, initial->uid));
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    auto attached = Metadata::attach(name);
    auto reader = Ref::acquire(attached, initial->uid);
    // _exit bypasses the same RAII cleanup that an unexpected exit loses.
    _exit(reader.valid() ? 0 : 1);
  }
  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(Ref::current_refcount(mapping), 1u);
  EXPECT_FALSE(Ref::reserve_for_publish(mapping));
  RecordProperty("method", "real_process_exit_metadata_only_no_cuda");
  RecordProperty("hazard", "dead_consumer_reference_is_not_reclaimed");
}
}  // namespace
