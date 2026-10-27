// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <limits>
#include <string>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"

namespace {
std::atomic<unsigned> connect_calls{0};
}

// Observe the real FD-request path; malformed metadata must never reach it.
extern "C" int connect(int fd, const sockaddr* address, socklen_t length) {
  using Connect = int (*)(int, const sockaddr*, socklen_t);
  static auto real_connect = reinterpret_cast<Connect>(dlsym(RTLD_NEXT, "connect"));
  ++connect_calls;
  return real_connect(fd, address, length);
}

namespace ros2_cuda_ipc_core {
namespace {
using Ref = buffer_metadata::BufferRef;
using Metadata = buffer_metadata::BufferMetadata;

class BufferMapperTest : public ::testing::Test {
 protected:
  void SetUp() override {
    static uint32_t block_id = 1000000;
    msg.publisher_pid = static_cast<uint32_t>(::getpid());
    msg.block_id = block_id++;
    msg.byte_size = 64;
    msg.device_id = 0;
    char path[] = "/tmp/g3_mapper_XXXXXX";
    ASSERT_NE(::mkdtemp(path), nullptr);
    directory = path;
    msg.vmm_socket_path = directory + "/missing.sock";
    name = buffer_metadata::shm_name_for_block(msg.publisher_pid, msg.block_id);
    connect_calls = 0;
  }

  void TearDown() override {
    // Creation success, not the chosen name, establishes unlink ownership.
    if (mapping) {
      mapping.reset();
      EXPECT_EQ(::shm_unlink(name.c_str()), 0);
    }
    if (!directory.empty()) EXPECT_EQ(::rmdir(directory.c_str()), 0);
  }

  void reserve() {
    mapping = Metadata::create(name);
    ASSERT_TRUE(mapping);
    auto reservation = Ref::reserve_for_publish(mapping);
    ASSERT_TRUE(reservation);
    msg.uid = reservation->uid;
  }

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  std::shared_ptr<Metadata> mapping;
  std::string name;
  std::string directory;
};

TEST_F(BufferMapperTest, MissingBlockMetadataReturnsEmptyOptional) {
  msg.uid = 99;
  subscriber::BufferMapper mapper({1, 64});
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));
  EXPECT_EQ(connect_calls, 0u);
  EXPECT_FALSE(mapper.poisoned());
  EXPECT_TRUE(mapper.try_close());
}

TEST_F(BufferMapperTest, StaleUidIsRejectedBeforeGpuImport) {
  ASSERT_NO_FATAL_FAILURE(reserve());
  ASSERT_TRUE(Ref::commit_publish(mapping, msg.uid));
  ASSERT_TRUE(Ref::acquire(mapping, msg.uid).valid());
  ++msg.uid;
  subscriber::BufferMapper mapper({1, 64});
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));
  EXPECT_EQ(connect_calls, 0u);
  EXPECT_EQ(Ref::current_refcount(mapping), 0u);
  EXPECT_FALSE(mapper.poisoned());
  EXPECT_TRUE(mapper.try_close());
}

TEST_F(BufferMapperTest, UnpublishedWriterIsRejectedBeforeGpuImport) {
  ASSERT_NO_FATAL_FAILURE(reserve());
  subscriber::BufferMapper mapper({1, 64});
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));
  EXPECT_EQ(connect_calls, 0u);
  EXPECT_EQ(Ref::current_refcount(mapping), std::numeric_limits<uint32_t>::max());
  ASSERT_TRUE(Ref::cancel_publish(mapping, msg.uid));
  EXPECT_TRUE(mapper.try_close());
}

TEST_F(BufferMapperTest, ImportFailureReleasesAcquiredReferenceAndAdmission) {
  ASSERT_NO_FATAL_FAILURE(reserve());
  ASSERT_TRUE(Ref::commit_publish(mapping, msg.uid));
  subscriber::BufferMapper mapper({1, 64});
  // H=1: a leaked permit would stop the next attempt before connect().
  for (unsigned attempt = 1; attempt <= 3; ++attempt) {
    EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));
    EXPECT_EQ(connect_calls, attempt);
    EXPECT_EQ(Ref::current_refcount(mapping), 0u);
    EXPECT_FALSE(mapper.poisoned());
  }
  EXPECT_TRUE(mapper.try_close());
}
}  // namespace
}  // namespace ros2_cuda_ipc_core
