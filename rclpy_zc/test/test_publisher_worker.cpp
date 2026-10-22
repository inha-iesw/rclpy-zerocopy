#include "rclpy_agnocast/publisher_worker.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
namespace
{

using rclpy_agnocast::BorrowedStorage;
using rclpy_agnocast::PublisherBackend;
using rclpy_agnocast::PublisherBackendFactory;
using rclpy_agnocast::PublisherWorker;
using rclpy_agnocast::PublisherWorkerError;

constexpr std::uintptr_t kPublisherId = 42U;
struct BackendTrace
{
  std::thread::id borrow_thread;
  std::thread::id create_thread;
  std::thread::id factory_destroy_thread;
  std::thread::id publish_thread;
  std::thread::id cancel_thread;
  std::thread::id destroy_thread;
  std::thread::id operation_thread;
  std::uint64_t sequence{0U};
  std::array<std::uint8_t, 64> data{};
  std::size_t size{0U};
  std::uint8_t published_byte{0U};
  bool fail_next_borrow{false};
  bool fail_after_borrow{false};
  bool fail_next_publish{false};
  bool return_invalid_storage{false};
};

class FakeBackend : public PublisherBackend
{
public:
  explicit FakeBackend(std::shared_ptr<BackendTrace> trace) : trace_(std::move(trace)) {}

  ~FakeBackend() override {trace_->destroy_thread = std::this_thread::get_id();}

  BorrowedStorage borrow(const std::size_t size) override
  {
    trace_->borrow_thread = std::this_thread::get_id();
    if (trace_->fail_next_borrow) {
      trace_->fail_next_borrow = false;
      throw std::bad_alloc();
    }
    if (size > trace_->data.size()) {
      throw std::length_error("test payload is too large");
    }
    trace_->size = size;
    if (trace_->fail_after_borrow) {
      trace_->fail_after_borrow = false;
      throw std::bad_alloc();
    }
    if (trace_->return_invalid_storage) {
      trace_->return_invalid_storage = false;
      return BorrowedStorage{nullptr, nullptr, size};
    }
    return BorrowedStorage{&trace_->sequence, trace_->data.data(), size};
  }

  void publish() override
  {
    trace_->publish_thread = std::this_thread::get_id();
    if (trace_->fail_next_publish) {
      trace_->fail_next_publish = false;
      throw std::runtime_error("injected publish failure");
    }
    trace_->published_byte = trace_->size == 0U ? 0U : trace_->data[trace_->size - 1U];
  }

  void cancel() noexcept override {trace_->cancel_thread = std::this_thread::get_id();}

private:
  std::shared_ptr<BackendTrace> trace_;
};

class FakeRootBackend final : public FakeBackend
{
public:
  explicit FakeRootBackend(std::shared_ptr<BackendTrace> trace)
  : FakeBackend(trace), trace_(std::move(trace)) {}

  void * borrow_root() override
  {
    trace_->borrow_thread = std::this_thread::get_id();
    if (trace_->fail_next_borrow) {
      trace_->fail_next_borrow = false;
      throw std::bad_alloc();
    }
    if (trace_->return_invalid_storage) {
      trace_->return_invalid_storage = false;
      return nullptr;
    }
    return &trace_->sequence;
  }

private:
  std::shared_ptr<BackendTrace> trace_;
};

class FakeBackendFactory final : public PublisherBackendFactory
{
public:
  explicit FakeBackendFactory(std::shared_ptr<BackendTrace> trace, const bool root = false)
  : trace_(std::move(trace)), root_(root) {}
  ~FakeBackendFactory() override {trace_->factory_destroy_thread = std::this_thread::get_id();}
  std::unique_ptr<PublisherBackend> create() override
  {
    trace_->create_thread = std::this_thread::get_id();
    if (root_) {return std::make_unique<FakeRootBackend>(trace_);}
    return std::make_unique<FakeBackend>(trace_);
  }
private:
  std::shared_ptr<BackendTrace> trace_;
  bool root_;
};
void add_fake(
  PublisherWorker & worker, const std::shared_ptr<BackendTrace> & trace,
  const bool root = false, const std::uintptr_t id = kPublisherId)
{
  std::unique_ptr<PublisherBackendFactory> factory = std::make_unique<FakeBackendFactory>(trace, root);
  const auto result = worker.add_publisher(id, factory);
  ASSERT_EQ(PublisherWorkerError::kNone, result.error);
  ASSERT_EQ(nullptr, factory);
}
TEST(PublisherWorker, RunsBorrowAndPublishOnOneNativeThread)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace);
  const auto caller_thread = std::this_thread::get_id();

  const auto borrowed = worker.borrow(kPublisherId, 4U);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
  *borrowed.region.sequence = 99U;
  borrowed.region.data[3] = 0xABU;
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.publish(kPublisherId, borrowed.region.generation).error);

  EXPECT_NE(caller_thread, trace->borrow_thread);
  EXPECT_EQ(trace->create_thread, trace->borrow_thread);
  EXPECT_EQ(trace->create_thread, trace->factory_destroy_thread);
  EXPECT_EQ(trace->borrow_thread, trace->publish_thread);
  EXPECT_EQ(99U, trace->sequence);
  EXPECT_EQ(0xABU, trace->published_byte);
}
TEST(PublisherWorker, RunsCancelAndDestructionOnTheWorkerThread)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace);

  const auto borrowed = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, borrowed.region.generation).error);
  ASSERT_EQ(PublisherWorkerError::kNone, worker.remove_publisher(kPublisherId).error);

  EXPECT_EQ(trace->borrow_thread, trace->cancel_thread);
  EXPECT_EQ(trace->borrow_thread, trace->destroy_thread);
}
TEST(PublisherWorker, RejectsSecondBorrowWhileLoanIsActive)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace);

  const auto first = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, first.error);

  EXPECT_EQ(PublisherWorkerError::kBusy, worker.borrow(kPublisherId, 1U).error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, first.region.generation).error);
}
TEST(PublisherWorker, RejectsStaleTerminalGeneration)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace);

  const auto borrowed = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);

  EXPECT_EQ(PublisherWorkerError::kStaleGeneration,
    worker.publish(kPublisherId, borrowed.region.generation + 1U).error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, borrowed.region.generation).error);
}
TEST(PublisherWorker, RejectsStopUntilActiveLoanIsCancelled)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace);

  const auto borrowed = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);

  EXPECT_EQ(PublisherWorkerError::kBusy, worker.stop().error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, borrowed.region.generation).error);
  EXPECT_EQ(PublisherWorkerError::kNone, worker.stop().error);
}
TEST(PublisherWorker, RecoversAfterBackendBorrowFailure)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  trace->fail_next_borrow = true;
  add_fake(worker, trace);

  const auto failed = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kOperationFailed, failed.error);
  ASSERT_NE(nullptr, failed.failure);
  EXPECT_THROW(std::rethrow_exception(failed.failure), std::bad_alloc);

  const auto recovered = worker.borrow(kPublisherId, 1U);
  EXPECT_EQ(PublisherWorkerError::kNone, recovered.error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, recovered.region.generation).error);
}
TEST(PublisherWorker, RetainsRejectedBackendOnTheCallerThread)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace);
  const auto borrowed = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
  auto rejected_trace = std::make_shared<BackendTrace>();
  std::unique_ptr<PublisherBackendFactory> rejected =
    std::make_unique<FakeBackendFactory>(rejected_trace);

  EXPECT_EQ(PublisherWorkerError::kBusy,
    worker.add_publisher(kPublisherId + 1U, rejected).error);
  EXPECT_NE(nullptr, rejected);
  EXPECT_EQ(std::thread::id{}, rejected_trace->create_thread);

  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, borrowed.region.generation).error);
  const auto caller_thread = std::this_thread::get_id();
  rejected.reset();
  EXPECT_EQ(caller_thread, rejected_trace->factory_destroy_thread);
}
TEST(PublisherWorker, TreatsPublishFailureAsTerminal)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  trace->fail_next_publish = true;
  add_fake(worker, trace);
  const auto first = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, first.error);

  const auto failed = worker.publish(kPublisherId, first.region.generation);
  ASSERT_EQ(PublisherWorkerError::kOperationFailed, failed.error);
  ASSERT_NE(nullptr, failed.failure);
  EXPECT_THROW(std::rethrow_exception(failed.failure), std::runtime_error);

  const auto recovered = worker.borrow(kPublisherId, 1U);
  EXPECT_EQ(PublisherWorkerError::kNone, recovered.error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, recovered.region.generation).error);
}
TEST(PublisherWorker, CancelsBackendWhenBorrowFailsAfterAcquisition)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  trace->fail_after_borrow = true;
  add_fake(worker, trace);

  EXPECT_EQ(PublisherWorkerError::kOperationFailed, worker.borrow(kPublisherId, 1U).error);
  EXPECT_EQ(trace->borrow_thread, trace->cancel_thread);

  const auto recovered = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, recovered.error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, recovered.region.generation).error);
}
TEST(PublisherWorker, RejectsAndCancelsInvalidBorrowedStorage)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  trace->return_invalid_storage = true;
  add_fake(worker, trace);

  EXPECT_EQ(PublisherWorkerError::kInvalidStorage, worker.borrow(kPublisherId, 1U).error);
  EXPECT_EQ(trace->borrow_thread, trace->cancel_thread);

  const auto recovered = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, recovered.error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, recovered.region.generation).error);
}
TEST(PublisherWorker, QueuesCleanupCancelWithoutCallerSideTerminalWork)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace);
  const auto caller_thread = std::this_thread::get_id();
  const auto borrowed = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);

  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel_async(kPublisherId, borrowed.region.generation));
  EXPECT_EQ(PublisherWorkerError::kNone, worker.wait_until_idle().error);
  EXPECT_NE(caller_thread, trace->cancel_thread);
  EXPECT_EQ(trace->borrow_thread, trace->cancel_thread);

  const auto recovered = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, recovered.error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, recovered.region.generation).error);
}
TEST(PublisherWorker, DestructorJoinsAfterQueuedCleanupCancel)
{
  auto trace = std::make_shared<BackendTrace>();
  {
    PublisherWorker worker;
    add_fake(worker, trace);
    const auto borrowed = worker.borrow(kPublisherId, 1U);
    ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
    ASSERT_EQ(PublisherWorkerError::kNone,
      worker.cancel_async(kPublisherId, borrowed.region.generation));
  }
  EXPECT_EQ(trace->borrow_thread, trace->cancel_thread);
  EXPECT_EQ(trace->borrow_thread, trace->destroy_thread);
}

void record_operation(void * argument)
{
  auto & trace = *static_cast<BackendTrace *>(argument);
  trace.operation_thread = std::this_thread::get_id();
  ++trace.sequence;
}

TEST(PublisherWorker, RunsRootBorrowOperationPublishAndDestructionOnTheWorker)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace, true);
  const auto borrowed = worker.borrow_root(kPublisherId);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
  ASSERT_EQ(&trace->sequence, borrowed.root);
  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.operate(kPublisherId, borrowed.generation, record_operation, trace.get()).error);
  EXPECT_EQ(1U, trace->sequence);
  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.publish(kPublisherId, borrowed.generation).error);
  ASSERT_EQ(PublisherWorkerError::kNone, worker.remove_publisher(kPublisherId).error);
  EXPECT_NE(std::this_thread::get_id(), trace->borrow_thread);
  EXPECT_EQ(trace->create_thread, trace->borrow_thread);
  EXPECT_EQ(trace->borrow_thread, trace->operation_thread);
  EXPECT_EQ(trace->borrow_thread, trace->publish_thread);
  EXPECT_EQ(trace->borrow_thread, trace->destroy_thread);
}

TEST(PublisherWorker, RootAndSharedBufferBorrowsShareOneActiveLoan)
{
  PublisherWorker worker;
  add_fake(worker, std::make_shared<BackendTrace>());
  add_fake(worker, std::make_shared<BackendTrace>(), true, kPublisherId + 1U);
  const auto legacy = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, legacy.error);
  EXPECT_EQ(PublisherWorkerError::kBusy, worker.borrow_root(kPublisherId + 1U).error);
  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, legacy.region.generation).error);
  const auto root = worker.borrow_root(kPublisherId + 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, root.error);
  EXPECT_GT(root.generation, legacy.region.generation);
  EXPECT_EQ(PublisherWorkerError::kBusy, worker.borrow(kPublisherId, 1U).error);
  EXPECT_EQ(PublisherWorkerError::kBusy, worker.borrow_root(kPublisherId + 1U).error);
  EXPECT_EQ(PublisherWorkerError::kBusy, worker.remove_publisher(kPublisherId).error);
  EXPECT_EQ(PublisherWorkerError::kBusy, worker.stop().error);
  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId + 1U, root.generation).error);
  EXPECT_EQ(PublisherWorkerError::kNone, worker.stop().error);
  EXPECT_EQ(PublisherWorkerError::kStopped, worker.borrow_root(kPublisherId).error);
}

TEST(PublisherWorker, RejectsInvalidOperationIdentityBeforeCallingTheTask)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace, true);
  const auto borrowed = worker.borrow_root(kPublisherId);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
  EXPECT_EQ(PublisherWorkerError::kNotFound,
    worker.operate(kPublisherId + 1U, borrowed.generation, record_operation, trace.get()).error);
  EXPECT_EQ(PublisherWorkerError::kStaleGeneration,
    worker.operate(kPublisherId, borrowed.generation + 1U, record_operation, trace.get()).error);
  EXPECT_EQ(PublisherWorkerError::kOperationFailed,
    worker.operate(kPublisherId, borrowed.generation, nullptr, trace.get()).error);
  EXPECT_EQ(0U, trace->sequence);
  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, borrowed.generation).error);
  EXPECT_EQ(PublisherWorkerError::kNotFound,
    worker.operate(kPublisherId, borrowed.generation, record_operation, trace.get()).error);
}

TEST(PublisherWorker, OperationFailureKeepsTheRootActiveForRecoveryOrCancellation)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace, true);
  const auto borrowed = worker.borrow_root(kPublisherId);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
  const auto failed = worker.operate(kPublisherId, borrowed.generation, [](void *) {
      throw std::length_error("injected structural failure");
    }, nullptr);
  ASSERT_EQ(PublisherWorkerError::kOperationFailed, failed.error);
  ASSERT_NE(nullptr, failed.failure);
  EXPECT_THROW(std::rethrow_exception(failed.failure), std::length_error);
  EXPECT_EQ(PublisherWorkerError::kBusy, worker.borrow_root(kPublisherId).error);
  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.operate(kPublisherId, borrowed.generation, record_operation, trace.get()).error);
  EXPECT_EQ(1U, trace->sequence);
  ASSERT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, borrowed.generation).error);
  EXPECT_EQ(trace->borrow_thread, trace->cancel_thread);
}

TEST(PublisherWorker, CancelsFailedOrInvalidRootBorrowBeforeTheNextBorrow)
{
  PublisherWorker worker;
  auto trace = std::make_shared<BackendTrace>();
  add_fake(worker, trace, true);
  trace->fail_next_borrow = true;
  const auto failed = worker.borrow_root(kPublisherId);
  ASSERT_EQ(PublisherWorkerError::kOperationFailed, failed.error);
  ASSERT_NE(nullptr, failed.failure);
  EXPECT_THROW(std::rethrow_exception(failed.failure), std::bad_alloc);
  EXPECT_EQ(trace->borrow_thread, trace->cancel_thread);
  trace->return_invalid_storage = true;
  EXPECT_EQ(PublisherWorkerError::kInvalidStorage, worker.borrow_root(kPublisherId).error);
  const auto recovered = worker.borrow_root(kPublisherId);
  ASSERT_EQ(PublisherWorkerError::kNone, recovered.error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, recovered.generation).error);
}

TEST(PublisherWorker, LegacyBackendRejectsRootBorrowWithoutBlockingLaterBorrow)
{
  PublisherWorker worker;
  add_fake(worker, std::make_shared<BackendTrace>());
  const auto failed = worker.borrow_root(kPublisherId);
  ASSERT_EQ(PublisherWorkerError::kOperationFailed, failed.error);
  ASSERT_NE(nullptr, failed.failure);
  EXPECT_THROW(std::rethrow_exception(failed.failure), std::logic_error);
  const auto recovered = worker.borrow(kPublisherId, 1U);
  ASSERT_EQ(PublisherWorkerError::kNone, recovered.error);
  EXPECT_EQ(PublisherWorkerError::kNone,
    worker.cancel(kPublisherId, recovered.region.generation).error);
}

TEST(PublisherWorker, RootCleanupCancellationAndDestructionStayOnTheWorker)
{
  auto trace = std::make_shared<BackendTrace>();
  {
    PublisherWorker worker;
    add_fake(worker, trace, true);
    const auto borrowed = worker.borrow_root(kPublisherId);
    ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
    ASSERT_EQ(PublisherWorkerError::kNone,
      worker.cancel_async(kPublisherId, borrowed.generation));
  }
  EXPECT_NE(std::this_thread::get_id(), trace->cancel_thread);
  EXPECT_EQ(trace->borrow_thread, trace->cancel_thread);
  EXPECT_EQ(trace->borrow_thread, trace->destroy_thread);
}
}  // namespace
