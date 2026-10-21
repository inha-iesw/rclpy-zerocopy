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
  std::uint64_t sequence{0U};
  std::array<std::uint8_t, 64> data{};
  std::size_t size{0U};
  std::uint8_t published_byte{0U};
  bool fail_next_borrow{false};
  bool fail_after_borrow{false};
  bool fail_next_publish{false};
  bool return_invalid_storage{false};
};

class FakeBackend final : public PublisherBackend
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

class FakeBackendFactory final : public PublisherBackendFactory
{
public:
  explicit FakeBackendFactory(std::shared_ptr<BackendTrace> trace) : trace_(std::move(trace)) {}
  ~FakeBackendFactory() override {trace_->factory_destroy_thread = std::this_thread::get_id();}
  std::unique_ptr<PublisherBackend> create() override
  {trace_->create_thread = std::this_thread::get_id(); return std::make_unique<FakeBackend>(trace_);}
private:
  std::shared_ptr<BackendTrace> trace_;
};
void add_fake(PublisherWorker & worker, const std::shared_ptr<BackendTrace> & trace)
{
  std::unique_ptr<PublisherBackendFactory> factory = std::make_unique<FakeBackendFactory>(trace);
  const auto result = worker.add_publisher(kPublisherId, factory);
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
}  // namespace
