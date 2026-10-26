#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <atomic>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

namespace
{
using Metadata = ros2_cuda_ipc_core::buffer_metadata::BufferMetadata;
using Ref = ros2_cuda_ipc_core::buffer_metadata::BufferRef;

class Reservation : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static unsigned counter = 0;
    name = "/g3_repaired_" + std::to_string(getpid()) + "_" + std::to_string(counter++);
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

TEST_F(Reservation, ReaderCannotEnterWhileWriterOwnsNewGeneration)
{
  auto writer = Ref::reserve_for_publish(mapping);
  ASSERT_TRUE(writer);
  EXPECT_EQ(Ref::current_refcount(mapping), std::numeric_limits<uint32_t>::max());
  EXPECT_FALSE(Ref::acquire(mapping, writer->uid).valid());
  ASSERT_TRUE(Ref::commit_publish(mapping, writer->uid));
  {
    auto reader = Ref::acquire(mapping, writer->uid);
    ASSERT_TRUE(reader.valid());
    EXPECT_FALSE(Ref::reserve_for_publish(mapping));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  auto next = Ref::reserve_for_publish(mapping);
  ASSERT_TRUE(next);
  EXPECT_NE(next->uid, writer->uid);
  EXPECT_FALSE(Ref::acquire(mapping, writer->uid).valid());
  EXPECT_TRUE(Ref::cancel_publish(mapping, next->uid));
}

TEST_F(Reservation, PausedWriterClaimRejectsPreviousGenerationReader)
{
  auto first = Ref::reserve_for_publish(mapping);
  ASSERT_TRUE(first);
  ASSERT_TRUE(Ref::commit_publish(mapping, first->uid));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  uint32_t expected = 0;
  ASSERT_TRUE(mapping->metadata()->refcount.compare_exchange_strong(
      expected, std::numeric_limits<uint32_t>::max()));
  EXPECT_FALSE(Ref::acquire(mapping, first->uid).valid());
  mapping->metadata()->uid.store(first->uid + 1);
  EXPECT_FALSE(Ref::acquire(mapping, first->uid).valid());
  EXPECT_TRUE(Ref::cancel_publish(mapping, first->uid + 1));
}

TEST_F(Reservation, UidOverflowFailsWithoutLeavingWriterClaim)
{
  mapping->metadata()->uid.store(std::numeric_limits<uint64_t>::max());
  EXPECT_FALSE(Ref::reserve_for_publish(mapping));
  EXPECT_EQ(Ref::current_refcount(mapping), 0u);
}

TEST_F(Reservation, ReaderCountCannotBecomeWriterSentinel)
{
  auto writer = Ref::reserve_for_publish(mapping);
  ASSERT_TRUE(writer);
  ASSERT_TRUE(Ref::commit_publish(mapping, writer->uid));
  const auto uid = writer->uid;
  mapping->metadata()->refcount.store(std::numeric_limits<uint32_t>::max() - 1);
  EXPECT_FALSE(Ref::acquire(mapping, uid).valid());
  EXPECT_EQ(Ref::current_refcount(mapping), std::numeric_limits<uint32_t>::max() - 1);
}

TEST_F(Reservation, DeadReaderLeavesFixedSlotUnavailable)
{
  auto writer = Ref::reserve_for_publish(mapping);
  ASSERT_TRUE(writer);
  ASSERT_TRUE(Ref::commit_publish(mapping, writer->uid));
  const pid_t pid = fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    auto reader = Ref::acquire(mapping, writer->uid);
    _exit(reader.valid() ? 0 : 1);
  }
  int status = 0;
  ASSERT_EQ(waitpid(pid, &status, 0), pid);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(Ref::current_refcount(mapping), 1u);
  EXPECT_FALSE(Ref::reserve_for_publish(mapping));
}

TEST_F(Reservation, ConcurrentOldReaderAndNextWriterNeverBothEnter)
{
  for (unsigned iteration = 0; iteration < 1000; ++iteration) {
    auto seed = Ref::reserve_for_publish(mapping);
    ASSERT_TRUE(seed);
    ASSERT_TRUE(Ref::commit_publish(mapping, seed->uid));
    // Expire only the fixture's grace timestamp, without delaying the race.
    mapping->metadata()->publish_timestamp_us.store(1);
    std::atomic<unsigned> ready{0};
    std::atomic<bool> go{false};
    std::optional<Ref::PublisherReservation> next;
    std::optional<Ref> reader;
    auto start = [&]() {
        ready.fetch_add(1);
        while (!go.load()) {std::this_thread::yield();}
      };
    std::thread writer_thread([&]() {start(); next = Ref::reserve_for_publish(mapping);});
    std::thread reader_thread([&]() {start(); reader.emplace(Ref::acquire(mapping, seed->uid));});
    while (ready.load() != 2) {std::this_thread::yield();}
    go.store(true);
    writer_thread.join();
    reader_thread.join();
    EXPECT_FALSE(next && reader->valid()) << iteration;
    reader.reset();
    if (next) {ASSERT_TRUE(Ref::cancel_publish(mapping, next->uid));}
    ASSERT_EQ(Ref::current_refcount(mapping), 0u);
  }
}

TEST(RepairedProtocol, MetadataNamespaceRejectsLegacyPeers)
{
  const auto name = ros2_cuda_ipc_core::buffer_metadata::shm_name_for_block(10, 20);
  EXPECT_EQ(name, "/rclpy_zc_gpu_v1_10_20");
}
}  // namespace
