#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "ros2_cuda_ipc_core/detail/read_handle_factory.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "subscriber/admission.hpp"
#include "subscriber_failure.hpp"

namespace
{
namespace failure = subscriber_failure;
namespace detail = ros2_cuda_ipc_core::subscriber::detail;
using Mapper = ros2_cuda_ipc_core::subscriber::BufferMapper;
using Ref = ros2_cuda_ipc_core::buffer_metadata::BufferRef;
using Metadata = ros2_cuda_ipc_core::buffer_metadata::BufferMetadata;
using Resource = ros2_cuda_ipc_core::backend::ImportedResources;
using Context = ros2_cuda_ipc_core::detail::CudaDeviceContext;
using Operation = failure::Operation;

template<class Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout = std::chrono::seconds(2))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return predicate();
}

class SubscriberFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
    mapper = std::make_unique<Mapper>(Mapper::Config{1, 64});
    epoch = detail::read_epoch(1, 64);
    name = "/g3_subscriber_cleanup_" + std::to_string(getpid());
    metadata = Metadata::create(name);
    ASSERT_TRUE(metadata);
    auto seed = Ref::reserve_for_publish(metadata);
    ASSERT_TRUE(seed);
    ASSERT_TRUE(Ref::commit_publish(metadata, seed->uid));
    auto ref = std::make_unique<Ref>(Ref::acquire(metadata, seed->uid));
    ASSERT_TRUE(ref->valid());
    auto permit = detail::admit(epoch);
    ASSERT_TRUE(permit);
    auto retained = Context::retain_primary(0);
    ASSERT_TRUE(retained);
    auto resource = std::make_shared<Resource>();
    // Only completion/context/ref ownership is tested here. This pointer is
    // never dereferenced, and this fixture does not claim payload or IPC coverage.
    resource->dev_ptr = reinterpret_cast<void *>(0x1000);
    resource->context = std::move(retained).value();
    ASSERT_EQ(cuEventCreate(&ready_event, CU_EVENT_DISABLE_TIMING), CUDA_SUCCESS);
    ASSERT_EQ(cuEventRecord(ready_event, nullptr), CUDA_SUCCESS);
    resource->event = ready_event;
    publication = detail::ReadHandleFactory::make_publication(
      std::move(resource), std::move(ref), 64, 0, std::move(permit));
    ASSERT_TRUE(publication);
    ASSERT_EQ(epoch->reads.load(), 1u);
    ASSERT_EQ(Ref::current_refcount(metadata), 1u);
  }

  void TearDown() override
  {
    failure::release_pop();
    failure::disarm();
    publication.reset();
    if (ready_event) {(void)cuEventDestroy(ready_event);}
    if (!name.empty()) {(void)shm_unlink(name.c_str());}
  }

  bool drained(Mapper & value)
  {
#ifdef G3_PREVIOUS_CORE
    // The old core has no close API. Keep the same observable drain oracle so
    // the historical binary can demonstrate premature read/ref release.
    return epoch->reads.load() == 0 && !value.poisoned();
#else
    return value.try_close();
#endif
  }

  void expect_retained_and_poisoned()
  {
    ASSERT_TRUE(wait_until([this] {return mapper->poisoned() && epoch->poisoned.load();}));
    EXPECT_TRUE(epoch->poisoned.load());
    EXPECT_EQ(epoch->reads.load(), 1u);
    EXPECT_EQ(Ref::current_refcount(metadata), 1u);
    EXPECT_FALSE(drained(*mapper));
    mapper.reset();
    for (unsigned attempt = 0; attempt < 20; ++attempt) {
      Mapper replacement({1, 64});
      EXPECT_TRUE(replacement.poisoned()) << attempt;
      EXPECT_FALSE(drained(replacement)) << attempt;
      EXPECT_FALSE(detail::admit(epoch)) << attempt;
      EXPECT_EQ(epoch->reads.load(), 1u) << attempt;
      EXPECT_EQ(Ref::current_refcount(metadata), 1u) << attempt;
    }
  }

  std::unique_ptr<Mapper> mapper;
  std::shared_ptr<detail::ReadEpoch> epoch;
  std::shared_ptr<Metadata> metadata;
  std::unique_ptr<detail::MappedPublication> publication;
  CUevent ready_event = nullptr;
  std::string name;
};

class SubscriberBindFailure : public SubscriberFixture,
  public ::testing::WithParamInterface<Operation> {};

TEST_P(SubscriberBindFailure, KeepsPublicationForRetryWhenCleanupSucceeds)
{
  failure::arm(GetParam());
  EXPECT_FALSE(detail::ReadHandleFactory::make_bound(*publication, nullptr));
  EXPECT_EQ(failure::failures(GetParam()), 1u);
  EXPECT_EQ(failure::calls(GetParam()), 1u);
  EXPECT_EQ(epoch->reads.load(), 1u);
  EXPECT_EQ(Ref::current_refcount(metadata), 1u);
  EXPECT_FALSE(mapper->poisoned());
  EXPECT_FALSE(mapper->closing());
  EXPECT_FALSE(detail::admit(epoch));
  auto read = detail::ReadHandleFactory::make_bound(*publication, nullptr);
  ASSERT_TRUE(read);
  EXPECT_EQ(failure::calls(GetParam()), 2u);
  EXPECT_EQ(failure::failures(GetParam()), 1u);
  publication.reset();
  read.reset();
  ASSERT_TRUE(wait_until([this] {return drained(*mapper);}));
  EXPECT_EQ(epoch->reads.load(), 0u);
  EXPECT_EQ(Ref::current_refcount(metadata), 0u);
  EXPECT_FALSE(mapper->poisoned());
}

INSTANTIATE_TEST_SUITE_P(
  Recoverable, SubscriberBindFailure, ::testing::Values(Operation::Create, Operation::Wait),
  [](const ::testing::TestParamInfo<Operation> & info) {
    return info.param == Operation::Create ? "Create" : "Wait";
  });

enum class CleanupFailure {Record, Query, CompletionDestroy, BindWaitAndDestroy};

class SubscriberCleanupFailure : public SubscriberFixture,
  public ::testing::WithParamInterface<CleanupFailure> {};

TEST_P(SubscriberCleanupFailure, RetainsReadAndRejectsReplacementMappers)
{
  if (GetParam() == CleanupFailure::BindWaitAndDestroy) {
    failure::arm(Operation::Wait);
    failure::arm(Operation::Destroy);
    EXPECT_FALSE(detail::ReadHandleFactory::make_bound(*publication, nullptr));
    // Failed completion cleanup consumes publication ownership into quarantine.
    publication.reset();
  } else {
    auto read = detail::ReadHandleFactory::make_bound(*publication, nullptr);
    ASSERT_TRUE(read);
    publication.reset();
    failure::arm(GetParam() == CleanupFailure::Record ? Operation::Record :
      GetParam() == CleanupFailure::Query ? Operation::Query : Operation::Destroy);
    read.reset();
  }
  expect_retained_and_poisoned();
  const auto operation = GetParam() == CleanupFailure::Record ? Operation::Record :
    GetParam() == CleanupFailure::Query ? Operation::Query : Operation::Destroy;
  EXPECT_EQ(failure::failures(operation), 1u);
  EXPECT_EQ(failure::calls(operation), 1u);
  if (GetParam() == CleanupFailure::BindWaitAndDestroy) {
    EXPECT_EQ(failure::failures(Operation::Wait), 1u);
    EXPECT_EQ(failure::calls(Operation::Wait), 1u);
  }
}

std::string cleanup_failure_name(const ::testing::TestParamInfo<CleanupFailure> & info)
{
  const char * names[] = {"Record", "Query", "CompletionDestroy", "BindWaitAndDestroy"};
  return names[static_cast<unsigned>(info.param)];
}

INSTANTIATE_TEST_SUITE_P(Failures, SubscriberCleanupFailure, ::testing::Values(
    CleanupFailure::Record, CleanupFailure::Query, CleanupFailure::CompletionDestroy,
    CleanupFailure::BindWaitAndDestroy), cleanup_failure_name);

struct PopCase
{
  failure::PopOrigin origin;
  bool fail;
};

class SubscriberContextPop : public SubscriberFixture,
  public ::testing::WithParamInterface<PopCase> {};

TEST_P(SubscriberContextPop, DrainWaitsForContextRestoration)
{
  auto read = detail::ReadHandleFactory::make_bound(*publication, nullptr);
  ASSERT_TRUE(read);
  publication.reset();
  failure::gate_pop(GetParam().origin, GetParam().fail);
  std::atomic<bool> releasing_context_restored{false};
  std::thread releasing([held = std::move(read), &releasing_context_restored]() mutable {
      held.reset();
      CUcontext current = nullptr;
      releasing_context_restored.store(
        cuCtxGetCurrent(&current) == CUDA_SUCCESS && current == nullptr);
    });
  struct ReleaseAndJoin
  {
    std::thread & thread;
    ~ReleaseAndJoin() {failure::release_pop(); if (thread.joinable()) {thread.join();}}
  } release_and_join{releasing};
  ASSERT_TRUE(failure::wait_for_pop());
  // Give a prematurely queued worker a chance to release while the releaser's
  // pop is blocked. The fixed path cannot submit until restoration finishes.
  (void)wait_until([this] {return epoch->reads.load() == 0;}, std::chrono::milliseconds(200));
  EXPECT_EQ(epoch->reads.load(), 1u);
  EXPECT_EQ(Ref::current_refcount(metadata), 1u);
  EXPECT_FALSE(drained(*mapper));
  EXPECT_FALSE(mapper->poisoned());
  failure::release_pop();
  releasing.join();
  if (GetParam().fail) {
    expect_retained_and_poisoned();
    EXPECT_EQ(failure::pop_failures(), 1u);
    EXPECT_EQ(failure::pop_calls(), 1u);
  } else {
    ASSERT_TRUE(wait_until([this] {return drained(*mapper);}));
    EXPECT_EQ(epoch->reads.load(), 0u);
    EXPECT_EQ(Ref::current_refcount(metadata), 0u);
    EXPECT_FALSE(mapper->poisoned());
    EXPECT_EQ(failure::pop_failures(), 0u);
    EXPECT_TRUE(releasing_context_restored.load());
  }
}

std::string context_pop_name(const ::testing::TestParamInfo<PopCase> & info)
{
  return std::string(info.param.origin == failure::PopOrigin::Releaser ? "Releaser" : "Worker") +
         (info.param.fail ? "Failure" : "Success");
}

INSTANTIATE_TEST_SUITE_P(PopOrigins, SubscriberContextPop, ::testing::Values(
    PopCase{failure::PopOrigin::Releaser, false}, PopCase{failure::PopOrigin::Worker, false},
    PopCase{failure::PopOrigin::Releaser, true}, PopCase{failure::PopOrigin::Worker, true}),
  context_pop_name);
}  // namespace
