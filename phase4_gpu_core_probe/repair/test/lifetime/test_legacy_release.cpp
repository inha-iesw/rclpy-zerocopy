#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <memory>
#include <thread>

#include "ros2_cuda_ipc_core/detail/read_handle_factory.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "../support.hpp"

namespace
{
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;
using Ref = ros2_cuda_ipc_core::buffer_metadata::BufferRef;
using Metadata = ros2_cuda_ipc_core::buffer_metadata::BufferMetadata;
using Factory = ros2_cuda_ipc_core::subscriber::detail::ReadHandleFactory;
using Resource = ros2_cuda_ipc_core::backend::ImportedResources;

template<class Owner>
void destroy_on_unused_thread(Owner owner)
{
  bool empty_before = false;
  bool empty_after = false;
  std::thread thread([held = std::move(owner), &empty_before, &empty_after]() mutable {
      CUcontext current = nullptr;
      empty_before = cuCtxGetCurrent(&current) == CUDA_SUCCESS && current == nullptr;
      held.reset();
      empty_after = cuCtxGetCurrent(&current) == CUDA_SUCCESS && current == nullptr;
    });
  thread.join();
  EXPECT_TRUE(empty_before);
  EXPECT_TRUE(empty_after);
}

class LegacyRelease : public ::testing::TestWithParam<uintptr_t>
{
protected:
  CUstream stream() const {return reinterpret_cast<CUstream>(GetParam());}
};

TEST_P(LegacyRelease, CancelOnUnusedThreadReleasesBoundReservation)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  auto block = manager.acquire_for_publish(stream());
  ASSERT_TRUE(block);
  destroy_on_unused_thread(std::move(block));
  ASSERT_EQ(cudaStreamSynchronize(reinterpret_cast<cudaStream_t>(stream())), cudaSuccess);
  auto next = manager.acquire_for_publish(stream());
  ASSERT_TRUE(next);
  next->cancel();
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST_P(LegacyRelease, PrepareOnUnusedThreadUsesBoundContext)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  auto block = manager.acquire_for_publish(stream());
  ASSERT_TRUE(block);
  bool prepared = false;
  bool restored = false;
  std::thread thread([held = std::move(block), &prepared, &restored, this]() mutable {
      prepared = static_cast<bool>(held->prepare_publish(stream()));
      CUcontext current = nullptr;
      restored = cuCtxGetCurrent(&current) == CUDA_SUCCESS && current == nullptr;
    });
  thread.join();
  EXPECT_TRUE(prepared);
  EXPECT_TRUE(restored);
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST_P(LegacyRelease, ReadCompletionOnUnusedThreadReleasesNativeReference)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  auto retained = ros2_cuda_ipc_core::detail::CudaDeviceContext::retain_primary(0);
  ASSERT_TRUE(retained);
  const auto name = "/g3_legacy_read_" + std::to_string(getpid()) + "_" + std::to_string(GetParam());
  struct Unlink {
    std::string name;
    ~Unlink() {shm_unlink(name.c_str());}
  } unlink{name};
  auto metadata = Metadata::create(name);
  ASSERT_TRUE(metadata);
  auto seed = Ref::reserve_for_publish(metadata);
  ASSERT_TRUE(seed);
  ASSERT_TRUE(Ref::commit_publish(metadata, seed->uid));
  // As in upstream test_read_handle: no payload dereference or IPC import is
  // needed to exercise the real bound-context/completion/refcount lifecycle.
  auto resource = std::make_shared<Resource>();
  resource->dev_ptr = reinterpret_cast<void *>(0x1000);
  resource->context = retained.value();
  for (unsigned round = 0; round < 2; ++round) {
    auto ref = std::make_unique<Ref>(Ref::acquire(metadata, seed->uid));
    ASSERT_TRUE(ref->valid());
    auto publication = Factory::make_publication(resource, std::move(ref), 64, 0);
    ASSERT_TRUE(publication);
    EXPECT_FALSE(Factory::make_bound(*publication, CU_STREAM_PER_THREAD));
    auto read = Factory::make_bound(*publication, stream());
    ASSERT_TRUE(read);
    destroy_on_unused_thread(std::move(read));
    ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (Ref::current_refcount(metadata) != 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(Ref::current_refcount(metadata), 0u);
  }
}

INSTANTIATE_TEST_SUITE_P(DefaultAndLegacy, LegacyRelease, ::testing::Values(uintptr_t{0}, uintptr_t{1}));
}  // namespace
