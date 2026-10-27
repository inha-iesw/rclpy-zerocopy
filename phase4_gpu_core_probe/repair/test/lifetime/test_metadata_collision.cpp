#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <memory>
#include <string>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

namespace
{
namespace metadata = ros2_cuda_ipc_core::buffer_metadata;
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;
using Bytes = std::array<unsigned char, sizeof(metadata::BlockMetadata)>;

class RepairedMetadataCollision : public ::testing::TestWithParam<uint32_t>
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
    // Logging may overwrite errno after shm_open fails. Disable this logger
    // so the old EEXIST replacement branch is exercised deterministically.
    ASSERT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
    ASSERT_EQ(rcutils_logging_set_logger_level(
      "ros2_cuda_ipc_core.buffer_metadata", RCUTILS_LOG_SEVERITY_FATAL), RCUTILS_RET_OK);
    collision_name_ = name(GetParam());
    // create uses O_CREAT|O_EXCL and initializes a valid atomic metadata object.
    // Its locator alone gives a new pool no authority to replace this owner.
    collision_ = metadata::BufferMetadata::create(collision_name_);
    ASSERT_TRUE(collision_);
    collision_->metadata()->uid.store(0x123456789abcdef0ULL);
    collision_->metadata()->refcount.store(1);
    collision_->metadata()->publish_timestamp_us.store(1234567890123456ULL);
    original_fd_ = shm_open(collision_name_.c_str(), O_RDONLY | O_CLOEXEC, 0);
    ASSERT_GE(original_fd_, 0);
    ASSERT_EQ(fstat(original_fd_, &original_stat_), 0);
    ASSERT_EQ(original_stat_.st_size, static_cast<off_t>(original_bytes_.size()));
    ASSERT_EQ(original_stat_.st_nlink, 1u);
    ASSERT_EQ(
      pread(original_fd_, original_bytes_.data(), original_bytes_.size(), 0),
      static_cast<ssize_t>(original_bytes_.size()));
  }

  void TearDown() override
  {
    // Only this fixture's successfully created name is eligible for cleanup.
    if (collision_) {
      const int result = shm_unlink(collision_name_.c_str());
      EXPECT_TRUE(result == 0 || errno == ENOENT);
    }
    if (original_fd_ >= 0) {
      EXPECT_EQ(close(original_fd_), 0);
    }
    collision_.reset();
  }

  std::string name(uint32_t block_id) const
  {
    return metadata::shm_name_for_block(static_cast<uint32_t>(getpid()), block_id);
  }

  void expect_missing(uint32_t block_id) const
  {
    const int fd = shm_open(name(block_id).c_str(), O_RDONLY | O_CLOEXEC, 0);
    const int error = errno;
    if (fd >= 0) {
      EXPECT_EQ(close(fd), 0);
    }
    EXPECT_EQ(fd, -1) << "unexpected block " << block_id;
    if (fd < 0) {
      EXPECT_EQ(error, ENOENT) << block_id;
    }
  }

  void expect_present(uint32_t block_id) const
  {
    const int fd = shm_open(name(block_id).c_str(), O_RDONLY | O_CLOEXEC, 0);
    ASSERT_GE(fd, 0) << "missing block " << block_id;
    EXPECT_EQ(close(fd), 0);
  }

  void expect_collision_unchanged() const
  {
    // Checking the old mapping alone would miss unlink-and-recreate. Reopen
    // the published name and compare its identity as well as its full bytes.
    const int reopened = shm_open(collision_name_.c_str(), O_RDONLY | O_CLOEXEC, 0);
    ASSERT_GE(reopened, 0);
    struct stat current{};
    const int stat_result = fstat(reopened, &current);
    Bytes bytes{};
    const ssize_t size = pread(reopened, bytes.data(), bytes.size(), 0);
    EXPECT_EQ(close(reopened), 0);
    ASSERT_EQ(stat_result, 0);
    EXPECT_EQ(current.st_dev, original_stat_.st_dev);
    EXPECT_EQ(current.st_ino, original_stat_.st_ino);
    EXPECT_EQ(current.st_size, original_stat_.st_size);
    EXPECT_EQ(current.st_nlink, original_stat_.st_nlink);
    EXPECT_EQ(size, static_cast<ssize_t>(bytes.size()));
    EXPECT_EQ(bytes, original_bytes_);

    struct stat held{};
    ASSERT_EQ(fstat(original_fd_, &held), 0);
    EXPECT_EQ(held.st_dev, original_stat_.st_dev);
    EXPECT_EQ(held.st_ino, original_stat_.st_ino);
    EXPECT_EQ(held.st_size, original_stat_.st_size);
    EXPECT_EQ(held.st_nlink, 1u);
    ASSERT_EQ(
      pread(original_fd_, bytes.data(), bytes.size(), 0), static_cast<ssize_t>(bytes.size()));
    EXPECT_EQ(bytes, original_bytes_);
  }

private:
  std::string collision_name_;
  std::shared_ptr<metadata::BufferMetadata> collision_;
  int original_fd_ = -1;
  struct stat original_stat_{};
  Bytes original_bytes_{};
};

TEST_P(RepairedMetadataCollision, ExistingNameSurvivesFailedInitialisation)
{
  const uint32_t collision_index = GetParam();
  {
    Manager manager({3, 64, 0});
    EXPECT_FALSE(manager.initialise());
    EXPECT_FALSE(manager.is_initialised());
    EXPECT_FALSE(manager.poisoned());
    expect_collision_unchanged();
    // Earlier owned objects must be rolled back; later objects must not have
    // been created after the first collision.
    for (uint32_t block_id = 0; block_id < 3; ++block_id) {
      if (block_id != collision_index) {
        expect_missing(block_id);
      }
    }
    EXPECT_TRUE(manager.try_close());
    EXPECT_FALSE(manager.poisoned());
  }
  expect_collision_unchanged();

  // Failed creation consumes its block ID, while rollback releases the pool
  // epoch. A fresh pool can initialise without deleting the collision object.
  Manager replacement({3, 64, 0});
  ASSERT_TRUE(replacement.initialise());
  EXPECT_TRUE(replacement.is_initialised());
  EXPECT_FALSE(replacement.poisoned());
  for (uint32_t block_id = collision_index + 1; block_id < collision_index + 4; ++block_id) {
    expect_present(block_id);
  }
  expect_collision_unchanged();
  EXPECT_TRUE(replacement.try_close());
  EXPECT_FALSE(replacement.is_initialised());
  EXPECT_FALSE(replacement.poisoned());
  for (uint32_t block_id = collision_index + 1; block_id < collision_index + 4; ++block_id) {
    expect_missing(block_id);
  }
  expect_collision_unchanged();
}

INSTANTIATE_TEST_SUITE_P(
  Collision, RepairedMetadataCollision, ::testing::Values(0u, 1u),
  [](const ::testing::TestParamInfo<uint32_t> & info) {
    return info.param == 0 ? "First" : "Second";
  });
}  // namespace
