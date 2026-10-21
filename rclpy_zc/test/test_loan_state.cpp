#include "rclpy_agnocast/loan.hpp"
#include "rclpy_agnocast/publisher_worker.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>
#include <thread>

namespace
{

using rclpy_agnocast::LoanAction;
using rclpy_agnocast::LoanError;
using rclpy_agnocast::LoanIdentity;
using rclpy_agnocast::LoanLifecycle;
using rclpy_agnocast::LoanPhase;
using rclpy_agnocast::LoanState;
using rclpy_agnocast::PublisherBackend;
using rclpy_agnocast::PublisherBackendFactory;
using rclpy_agnocast::PublisherWorker;
using rclpy_agnocast::PublisherWorkerError;
using rclpy_agnocast::WritableBufferSpan;
using rclpy_agnocast::WritableExportAcquireResult;

constexpr std::uintptr_t kPublisherId = 0x1234U;
constexpr std::uint64_t kGeneration = 7U;

LoanIdentity current_identity()
{
  return LoanIdentity{kPublisherId, kGeneration, std::this_thread::get_id()};
}

struct StateTrace
{
  std::uint64_t sequence{0U};
  std::array<std::uint8_t, 8> data{};
  std::thread::id publish_thread;
  std::thread::id cancel_thread;
};

class StateBackend final : public PublisherBackend
{
public:
  explicit StateBackend(std::shared_ptr<StateTrace> trace) : trace_(std::move(trace)) {}
  rclpy_agnocast::BorrowedStorage borrow(std::size_t size) override
  {return {&trace_->sequence, trace_->data.data(), size};}
  void publish() override {trace_->publish_thread = std::this_thread::get_id();}
  void cancel() noexcept override {trace_->cancel_thread = std::this_thread::get_id();}

private:
  std::shared_ptr<StateTrace> trace_;
};

class StateBackendFactory final : public PublisherBackendFactory
{
public:
  explicit StateBackendFactory(std::shared_ptr<StateTrace> trace) : trace_(std::move(trace)) {}
  std::unique_ptr<PublisherBackend> create() override
  {return std::make_unique<StateBackend>(trace_);}

private:
  std::shared_ptr<StateTrace> trace_;
};

TEST(LoanLifecycle, PublishesExactlyOnceWithoutExports)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);

  const auto transition = lifecycle.request_publish(identity);

  EXPECT_EQ(LoanError::kNone, transition.error);
  EXPECT_EQ(LoanAction::kPublish, transition.action);
  EXPECT_EQ(LoanError::kNone, lifecycle.complete_publish());
  EXPECT_EQ(LoanPhase::kPublished, lifecycle.phase());
  EXPECT_EQ(LoanError::kInvalidPhase, lifecycle.request_publish(identity).error);
}

TEST(LoanLifecycle, RejectsPublishWhileWritableExportExists)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);

  EXPECT_EQ(LoanError::kNone, lifecycle.acquire_export().error);
  EXPECT_EQ(LoanError::kActiveExports, lifecycle.request_publish(identity).error);
  EXPECT_EQ(LoanPhase::kFilling, lifecycle.phase());

  EXPECT_EQ(LoanError::kNone, lifecycle.release_export().error);
  EXPECT_EQ(LoanAction::kPublish, lifecycle.request_publish(identity).action);
}

TEST(LoanLifecycle, DefersCancelUntilLastExportIsReleased)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);

  lifecycle.acquire_export();
  lifecycle.acquire_export();

  const auto request = lifecycle.request_cancel(identity, true);
  EXPECT_EQ(LoanError::kNone, request.error);
  EXPECT_EQ(LoanAction::kNone, request.action);
  EXPECT_EQ(LoanPhase::kCancelPending, lifecycle.phase());

  EXPECT_EQ(LoanAction::kNone, lifecycle.release_export().action);
  EXPECT_EQ(LoanAction::kCancel, lifecycle.release_export().action);
  EXPECT_EQ(LoanPhase::kCancelling, lifecycle.phase());
}

TEST(LoanLifecycle, AllowsForeignThreadToReleasePendingCancel)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);
  lifecycle.acquire_export();
  lifecycle.request_cancel(identity, true);

  rclpy_agnocast::LoanTransition transition{LoanError::kInvalidPhase, LoanAction::kNone};
  std::thread releaser([&lifecycle, &transition]() {transition = lifecycle.release_export();});
  releaser.join();

  EXPECT_EQ(LoanError::kNone, transition.error);
  EXPECT_EQ(LoanAction::kCancel, transition.action);
}

TEST(LoanLifecycle, RejectsExplicitCancelWhileWritableExportExists)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);
  lifecycle.acquire_export();

  const auto transition = lifecycle.request_cancel(identity, false);

  EXPECT_EQ(LoanError::kActiveExports, transition.error);
  EXPECT_EQ(LoanPhase::kFilling, lifecycle.phase());
}

TEST(LoanLifecycle, RejectsWrongPublisherBeforeTerminalAction)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);
  auto wrong_identity = identity;
  wrong_identity.publisher_id++;

  EXPECT_EQ(LoanError::kWrongPublisher, lifecycle.request_publish(wrong_identity).error);
  EXPECT_EQ(LoanPhase::kFilling, lifecycle.phase());
}

TEST(LoanLifecycle, RejectsStaleGenerationBeforeTerminalAction)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);
  auto stale_identity = identity;
  stale_identity.generation++;

  EXPECT_EQ(LoanError::kStaleGeneration, lifecycle.request_publish(stale_identity).error);
  EXPECT_EQ(LoanPhase::kFilling, lifecycle.phase());
}

TEST(LoanLifecycle, RejectsWrongCreatorThreadBeforeTerminalAction)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);
  auto wrong_identity = identity;
  wrong_identity.creator_thread = std::thread::id{};

  EXPECT_EQ(LoanError::kWrongThread, lifecycle.request_publish(wrong_identity).error);
  EXPECT_EQ(LoanPhase::kFilling, lifecycle.phase());
}

TEST(LoanLifecycle, RejectsOriginalIdentityUsedFromForeignThread)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);
  rclpy_agnocast::LoanTransition transition{LoanError::kNone, LoanAction::kNone};

  std::thread caller([&]() {transition = lifecycle.request_publish(identity);});
  caller.join();

  EXPECT_EQ(LoanError::kWrongThread, transition.error);
  EXPECT_EQ(LoanPhase::kFilling, lifecycle.phase());
}

TEST(LoanLifecycle, RejectsWritableExportAcquiredFromForeignThread)
{
  LoanLifecycle lifecycle(current_identity());
  rclpy_agnocast::LoanTransition transition{LoanError::kNone, LoanAction::kNone};

  std::thread caller([&]() {transition = lifecycle.acquire_export();});
  caller.join();

  EXPECT_EQ(LoanError::kWrongThread, transition.error);
  EXPECT_EQ(0U, lifecycle.export_count());
}

TEST(LoanLifecycle, AllowsForeignThreadToAbandonForDeferredCleanup)
{
  LoanLifecycle lifecycle(current_identity());
  lifecycle.acquire_export();
  rclpy_agnocast::LoanTransition transition{LoanError::kInvalidPhase, LoanAction::kNone};

  std::thread finalizer([&]() {transition = lifecycle.request_abandon();});
  finalizer.join();

  EXPECT_EQ(LoanError::kNone, transition.error);
  EXPECT_EQ(LoanAction::kNone, transition.action);
  EXPECT_EQ(LoanPhase::kCancelPending, lifecycle.phase());
  EXPECT_EQ(LoanAction::kCancel, lifecycle.release_export().action);
}

TEST(LoanLifecycle, RecordsTerminalPublishFailure)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);
  ASSERT_EQ(LoanAction::kPublish, lifecycle.request_publish(identity).action);

  EXPECT_EQ(LoanError::kNone, lifecycle.fail_publish());
  EXPECT_EQ(LoanPhase::kFailed, lifecycle.phase());
  EXPECT_EQ(LoanError::kInvalidPhase, lifecycle.request_publish(identity).error);
}

TEST(LoanLifecycle, RejectsNewExportAfterTerminalTransition)
{
  const auto identity = current_identity();
  LoanLifecycle lifecycle(identity);
  ASSERT_EQ(LoanAction::kCancel, lifecycle.request_cancel(identity, false).action);

  EXPECT_EQ(LoanError::kInvalidPhase, lifecycle.acquire_export().error);
  EXPECT_EQ(LoanError::kNone, lifecycle.complete_cancel());
  EXPECT_EQ(LoanError::kInvalidPhase, lifecycle.complete_cancel());
}

TEST(LoanLifecycle, RejectsReleaseWithoutAnActiveExport)
{
  LoanLifecycle lifecycle(current_identity());

  EXPECT_EQ(LoanError::kNoActiveExport, lifecycle.release_export().error);
  EXPECT_EQ(LoanPhase::kFilling, lifecycle.phase());
}

TEST(LoanState, PublishesFilledRegionThroughItsWorker)
{
  auto worker = std::make_shared<PublisherWorker>();
  auto trace = std::make_shared<StateTrace>();
  std::unique_ptr<PublisherBackendFactory> factory = std::make_unique<StateBackendFactory>(trace);
  ASSERT_EQ(PublisherWorkerError::kNone, worker->add_publisher(kPublisherId, factory).error);
  const auto borrowed = worker->borrow(kPublisherId, 4U);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
  LoanState state(worker,
    {kPublisherId, borrowed.region.generation, std::this_thread::get_id()}, borrowed.region);

  EXPECT_EQ(LoanError::kNone, state.set_sequence(55U));
  WritableBufferSpan span{};
  ASSERT_EQ(WritableExportAcquireResult::kOk, state.acquire_writable_export(span));
  ASSERT_EQ(4U, span.size);
  span.data[3] = 0xA5U;
  state.release_writable_export();
  const auto result = state.publish(kPublisherId);

  EXPECT_EQ(LoanError::kNone, result.loan_error);
  EXPECT_EQ(PublisherWorkerError::kNone, result.worker_error);
  EXPECT_EQ(55U, trace->sequence);
  EXPECT_NE(std::this_thread::get_id(), trace->publish_thread);
}

TEST(LoanState, LastForeignExportReleaseQueuesDeferredCancel)
{
  auto worker = std::make_shared<PublisherWorker>();
  auto trace = std::make_shared<StateTrace>();
  std::unique_ptr<PublisherBackendFactory> factory = std::make_unique<StateBackendFactory>(trace);
  ASSERT_EQ(PublisherWorkerError::kNone, worker->add_publisher(kPublisherId, factory).error);
  const auto borrowed = worker->borrow(kPublisherId, 1U);
  auto state = std::make_shared<LoanState>(worker,
    LoanIdentity{kPublisherId, borrowed.region.generation, std::this_thread::get_id()},
    borrowed.region);
  WritableBufferSpan span{};
  ASSERT_EQ(WritableExportAcquireResult::kOk, state->acquire_writable_export(span));
  state->abandon();

  std::thread releaser([state]() {state->release_writable_export();});
  releaser.join();
  EXPECT_EQ(PublisherWorkerError::kNone, worker->wait_until_idle().error);
  EXPECT_NE(std::thread::id{}, trace->cancel_thread);
}

}  // namespace
