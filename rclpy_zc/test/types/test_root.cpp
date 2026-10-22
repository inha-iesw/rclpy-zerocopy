// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclpy_agnocast/types/field_ops.hpp"
#include "rclpy_agnocast/loan.hpp"
#include "types/root.hpp"

namespace
{
using rclpy_agnocast::BorrowedStorage;
using rclpy_agnocast::LoanError;
using rclpy_agnocast::LoanIdentity;
using rclpy_agnocast::LoanLifecycle;
using rclpy_agnocast::LoanPhase;
using rclpy_agnocast::PublisherBackend;
using rclpy_agnocast::PublisherBackendFactory;
using rclpy_agnocast::PublisherWorker;
using rclpy_agnocast::PublisherWorkerError;
using rclpy_agnocast::types::BufferBusy;
using rclpy_agnocast::types::FieldMember;
using rclpy_agnocast::types::message_ops;
using rclpy_agnocast::types::RootOwner;
using rclpy_agnocast::types::RootPhase;
using rclpy_agnocast::types::scalar_ops;
using rclpy_agnocast::types::ScalarValue;
using rclpy_agnocast::types::sequence_ops;
using rclpy_agnocast::types::string_ops;
using rclpy_agnocast::types::TypeDescriptor;
using rclpy_agnocast::types::TypeOps;

constexpr std::uintptr_t kPublisher = 73;
struct Message;
struct Trace
{
  Message * live{nullptr};
  std::thread::id worker_thread;
  std::thread::id allocation_thread;
  std::thread::id deallocation_thread;
  std::thread::id root_destroy_thread;
  std::thread::id backend_destroy_thread;
  std::size_t allocations{0};
  std::size_t live_blocks{0};
  std::size_t fail_allocation{0};
  std::size_t extra_capacity{0};
  std::size_t initial_values{0};
  std::size_t cancels{0};
  std::size_t publishes{0};
  bool fail_publish{false};
  std::size_t destroyed_roots{0};
  std::int32_t published_first{0};
};
thread_local Trace * allocator_trace = nullptr;

template<typename T>
struct Allocator
{
  using value_type = T;
  Trace * trace{allocator_trace};
  Allocator() = default;
  template<typename U>
  Allocator(const Allocator<U> & other)
  : trace(other.trace) {}
  T * allocate(std::size_t count)
  {
    trace->allocation_thread = std::this_thread::get_id();
    if (++trace->allocations == trace->fail_allocation) {throw std::bad_alloc();}
    auto result = std::allocator<T>{}.allocate(count);
    ++trace->live_blocks;
    return result;
  }
  void deallocate(T * data, std::size_t count)
  {
    trace->deallocation_thread = std::this_thread::get_id();
    --trace->live_blocks;
    std::allocator<T>{}.deallocate(data, count);
  }
  template<typename U>
  bool operator==(const Allocator<U> & other) const {return trace == other.trace;}
  template<typename U>
  bool operator!=(const Allocator<U> & other) const {return !(*this == other);}
};

class Values : public std::vector<std::int32_t, Allocator<std::int32_t>>
{
public:
  void resize(std::size_t count)
  {
    const auto extra = get_allocator().trace->extra_capacity;
    if (extra) {reserve(count + extra);}
    std::vector<std::int32_t, Allocator<std::int32_t>>::resize(count);
  }
};
using Bytes = std::vector<std::uint8_t, Allocator<std::uint8_t>>;
struct Child
{
  Child() {bytes.resize(2);}
  Bytes bytes;
};
using Children = std::vector<Child, Allocator<Child>>;
using Flags = std::vector<bool, Allocator<bool>>;
using Texts = std::vector<std::string, Allocator<std::string>>;
using Text = std::basic_string<char, std::char_traits<char>, Allocator<char>>;
using WideText = std::basic_string<char16_t, std::char_traits<char16_t>, Allocator<char16_t>>;
struct Message
{
  explicit Message(Trace * owner)
  : trace(owner) {values.resize(trace->initial_values);}
  ~Message()
  {
    trace->root_destroy_thread = std::this_thread::get_id();
    ++trace->destroyed_roots;
  }
  Trace * trace;
  std::int32_t scalar{0};
  Values values;
  Children children;
  Flags flags;
  Texts texts;
  Text text;
  WideText wide_text;
};

const TypeDescriptor & descriptor()
{
  static const auto bytes = sequence_ops<Bytes>(scalar_ops<std::uint8_t>(), 16);
  static const FieldMember child_members[] = {
    {"bytes", &bytes, [] (void * root) noexcept->void * {
        return std::addressof(static_cast<Child *>(root)->bytes);
      }}
  };
  static const auto child = message_ops<Child>(child_members, 1);
  static const auto values = sequence_ops<Values>(scalar_ops<std::int32_t>(), 8);
  static const auto children = sequence_ops<Children>(&child, 4);
  static const auto flags = sequence_ops<Flags>(scalar_ops<bool>());
  static const auto text = string_ops<std::string>();
  static const auto texts = sequence_ops<Texts>(&text);
  static const auto tracked_text = string_ops<Text>();
  static const auto tracked_wide_text = string_ops<WideText>();
  static const FieldMember members[] = {
    {"scalar", scalar_ops<std::int32_t>(), [] (void * root) noexcept->void * {
        return std::addressof(static_cast<Message *>(root)->scalar);
      }},
    {"values", &values, [] (void * root) noexcept->void * {
        return std::addressof(static_cast<Message *>(root)->values);
      }},
    {"children", &children, [] (void * root) noexcept->void * {
        return std::addressof(static_cast<Message *>(root)->children);
      }},
    {"flags", &flags, [] (void * root) noexcept->void * {
        return std::addressof(static_cast<Message *>(root)->flags);
      }},
    {"texts", &texts, [] (void * root) noexcept->void * {
        return std::addressof(static_cast<Message *>(root)->texts);
      }},
    {"text", &tracked_text, [] (void * root) noexcept->void * {
        return std::addressof(static_cast<Message *>(root)->text);
      }},
    {"wide_text", &tracked_wide_text, [] (void * root) noexcept->void * {
        return std::addressof(static_cast<Message *>(root)->wide_text);
      }}
  };
  static const auto root = message_ops<Message>(members, 7);
  static const TypeOps operations{&root, nullptr};
  static const TypeDescriptor value{
    "root_test/msg/Message", std::string(64, 'a'), "[]", sizeof(Message), alignof(Message),
    &operations};
  return value;
}

class Backend final : public PublisherBackend
{
public:
  explicit Backend(std::shared_ptr<Trace> trace)
  : trace_(std::move(trace)) {}
  ~Backend() override {trace_->backend_destroy_thread = std::this_thread::get_id();}
  BorrowedStorage borrow(std::size_t) override {throw std::logic_error("root only");}
  void * borrow_root() override
  {
    allocator_trace = trace_.get();
    root_ = std::make_unique<Message>(trace_.get());
    trace_->live = root_.get();
    return root_.get();
  }
  void publish() override
  {
    ++trace_->publishes;
    if (!root_->values.empty()) {trace_->published_first = root_->values.front();}
    reset();
    if (trace_->fail_publish) {throw std::runtime_error("injected typed publish failure");}
  }
  void cancel() noexcept override
  {
    if (root_) {++trace_->cancels;}
    reset();
  }

private:
  void reset()
  {
    root_.reset();
    trace_->live = nullptr;
    allocator_trace = nullptr;
  }
  std::shared_ptr<Trace> trace_;
  std::unique_ptr<Message> root_;
};
class Factory final : public PublisherBackendFactory
{
public:
  explicit Factory(std::shared_ptr<Trace> trace)
  : trace_(std::move(trace)) {}
  std::unique_ptr<PublisherBackend> create() override
  {
    trace_->worker_thread = std::this_thread::get_id();
    return std::make_unique<Backend>(trace_);
  }

private:
  std::shared_ptr<Trace> trace_;
};

class TypedRoot : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::unique_ptr<PublisherBackendFactory> factory = std::make_unique<Factory>(trace);
    ASSERT_EQ(PublisherWorkerError::kNone, worker->add_publisher(kPublisher, factory).error);
  }
  void TearDown() override
  {
    owner.reset();
    EXPECT_EQ(PublisherWorkerError::kNone, worker->wait_until_idle().error);
    EXPECT_EQ(PublisherWorkerError::kNone, worker->remove_publisher(kPublisher).error);
    EXPECT_EQ(trace->worker_thread, trace->backend_destroy_thread);
    EXPECT_EQ(0U, trace->live_blocks);
  }
  std::shared_ptr<RootOwner> borrow(std::size_t maximum = 1024 * 1024)
  {
    const auto result = worker->borrow_root(kPublisher);
    if (result.error != PublisherWorkerError::kNone) {throw std::runtime_error("borrow failed");}
    return RootOwner::create(
      worker, kPublisher, result.generation, result.root, descriptor(), maximum);
  }
  std::uint64_t generation() const {return owner->snapshot().generation;}
  std::shared_ptr<Trace> trace{std::make_shared<Trace>()};
  std::shared_ptr<PublisherWorker> worker{std::make_shared<PublisherWorker>()};
  std::shared_ptr<RootOwner> owner;
};

TEST_F(TypedRoot, AncestorAndSiblingResizeInvalidateChildrenButRootCanResolveAgain)
{
  owner = borrow();
  owner->resize({2}, generation(), 1);
  const auto child_generation = owner->info({2, 0}, generation()).generation;
  owner->resize({1}, generation(), 2);
  EXPECT_THROW(owner->info({2, 0}, child_generation), std::runtime_error);
  const auto refreshed = owner->info({}, child_generation).generation;
  EXPECT_GT(refreshed, child_generation);
  EXPECT_EQ(2U, owner->info({2, 0, 0}, refreshed).size);
  owner->resize({2}, refreshed, 2);
  EXPECT_THROW(owner->info({2, 0}, refreshed), std::runtime_error);
  EXPECT_EQ(2U, owner->info({2}, generation()).size);
  const auto & native = *trace->live;
  std::size_t expected = sizeof(Message) + native.values.capacity() * sizeof(std::int32_t) +
    native.children.capacity() * sizeof(Child);
  for (const auto & child : native.children) {
    expected += child.bytes.capacity();
  }
  EXPECT_EQ(expected, owner->snapshot().accounted_bytes);
  EXPECT_EQ(trace->worker_thread, trace->allocation_thread);
}

TEST_F(TypedRoot, FirstScalarWriteRejectsResizeBeforeChangingPointerLengthOrGeneration)
{
  owner = borrow();
  owner->resize({1}, generation(), 2);
  const auto before = owner->snapshot();
  const auto pointer = trace->live->values.data();
  ScalarValue value{};
  value.signed_integer = 33;
  owner->write({0}, generation(), value);
  EXPECT_THROW(owner->resize({1}, generation(), 3), BufferBusy);
  EXPECT_EQ(pointer, trace->live->values.data());
  EXPECT_EQ(2U, trace->live->values.size());
  EXPECT_EQ(before.generation, generation());
  EXPECT_EQ(before.accounted_bytes, owner->snapshot().accounted_bytes);
  EXPECT_EQ(RootPhase::kFilling, owner->snapshot().phase);
  EXPECT_EQ(33, trace->live->scalar);
  EXPECT_EQ(33, owner->read({0}, generation()).signed_integer);
  owner->publish(kPublisher);
  EXPECT_EQ(1U, trace->publishes);
  EXPECT_THROW(owner->info({}, generation()), std::runtime_error);
  EXPECT_THROW(owner->publish(kPublisher), std::runtime_error);
}

TEST_F(TypedRoot, BufferExportBlocksPublishAndPermanentlyClosesTheResizePhase)
{
  owner = borrow();
  owner->resize({1}, generation(), 2);
  const auto before = owner->snapshot();
  const auto pointer = trace->live->values.data();
  const auto span = owner->acquire_export({1}, generation());
  ASSERT_EQ(pointer, span.data);
  ASSERT_EQ(sizeof(std::int32_t), span.item_size);
  ASSERT_EQ(2U, span.size);
  static_cast<std::int32_t *>(span.data)[0] = 24;
  EXPECT_THROW(owner->resize({1}, generation(), 3), BufferBusy);
  EXPECT_THROW(owner->publish(kPublisher), BufferBusy);
  EXPECT_EQ(0U, trace->publishes);
  EXPECT_EQ(pointer, trace->live->values.data());
  EXPECT_EQ(2U, trace->live->values.size());
  EXPECT_EQ(before.generation, generation());
  EXPECT_EQ(before.accounted_bytes, owner->snapshot().accounted_bytes);
  owner->release_export();
  EXPECT_EQ(0U, owner->snapshot().exports);
  EXPECT_THROW(owner->resize({1}, generation(), 1), BufferBusy);
  owner->publish(kPublisher);
  EXPECT_EQ(24, trace->published_first);
  EXPECT_EQ(trace->worker_thread, trace->root_destroy_thread);
}

TEST_F(TypedRoot, InvalidBoundAndScalarRangePreservePrepareAndPermitValidResize)
{
  owner = borrow();
  const auto before = owner->snapshot();
  const auto allocations = trace->allocations;
  EXPECT_THROW(owner->resize({1}, generation(), 9), std::invalid_argument);
  EXPECT_THROW(owner->resize({1}, generation(), SIZE_MAX), std::invalid_argument);
  ScalarValue value{};
  value.signed_integer = std::numeric_limits<std::int64_t>::max();
  EXPECT_THROW(owner->write({0}, generation(), value), std::invalid_argument);
  EXPECT_EQ(0, trace->live->scalar);
  EXPECT_EQ(allocations, trace->allocations);
  EXPECT_EQ(before.generation, generation());
  EXPECT_EQ(before.accounted_bytes, owner->snapshot().accounted_bytes);
  EXPECT_EQ(RootPhase::kPrepare, owner->snapshot().phase);
  owner->resize({1}, generation(), 2);
  EXPECT_EQ(2U, owner->info({1}, generation()).size);
  EXPECT_THROW(owner->read({1, 2}, generation()), std::out_of_range);
}

TEST_F(TypedRoot, ActualCapacityOvershootBecomesCancelOnlyAndTheNextBorrowSucceeds)
{
  const auto maximum = sizeof(Message) + 4 * sizeof(std::int32_t);
  owner = borrow(maximum);
  trace->extra_capacity = 64;
  EXPECT_THROW(owner->resize({1}, generation(), 4), std::invalid_argument);
  ASSERT_NE(nullptr, trace->live);
  EXPECT_GE(trace->live->values.capacity(), 68U);
  EXPECT_EQ(4U, trace->live->values.size());
  EXPECT_EQ(
    sizeof(Message) + trace->live->values.capacity() * sizeof(std::int32_t),
    owner->snapshot().accounted_bytes);
  EXPECT_EQ(RootPhase::kCancelOnly, owner->snapshot().phase);
  EXPECT_THROW(owner->publish(kPublisher), std::runtime_error);
  EXPECT_THROW(owner->acquire_export({1}, generation()), std::runtime_error);
  EXPECT_EQ(0U, trace->publishes);
  owner->cancel(kPublisher, false);
  EXPECT_EQ(1U, trace->cancels);
  EXPECT_EQ(trace->worker_thread, trace->deallocation_thread);
  trace->extra_capacity = 0;
  owner = borrow(maximum);
  EXPECT_EQ(RootPhase::kPrepare, owner->snapshot().phase);
}

TEST_F(TypedRoot, DefaultRootOverBudgetCancelsOnTheWorkerBeforeAValidNextBorrow)
{
  trace->initial_values = 2;
  EXPECT_THROW(borrow(sizeof(Message)), std::invalid_argument);
  EXPECT_EQ(nullptr, trace->live);
  EXPECT_EQ(1U, trace->cancels);
  EXPECT_EQ(1U, trace->destroyed_roots);
  EXPECT_EQ(trace->worker_thread, trace->root_destroy_thread);
  EXPECT_EQ(0U, trace->live_blocks);
  trace->initial_values = 0;
  owner = borrow(sizeof(Message));
  EXPECT_EQ(sizeof(Message), owner->snapshot().accounted_bytes);
}

TEST_F(TypedRoot, NestedAllocationFailureCancelsWithoutPublishingAPartialMessage)
{
  owner = borrow();
  // Allocate the children array, then one child's bytes, and fail the second child's bytes.
  trace->fail_allocation = trace->allocations + 3;
  EXPECT_THROW(owner->resize({2}, generation(), 2), std::bad_alloc);
  EXPECT_EQ(trace->fail_allocation, trace->allocations);
  EXPECT_EQ(trace->worker_thread, trace->allocation_thread);
  EXPECT_EQ(RootPhase::kCancelOnly, owner->snapshot().phase);
  EXPECT_THROW(owner->publish(kPublisher), std::runtime_error);
  EXPECT_EQ(0U, trace->publishes);
  owner->cancel(kPublisher, false);
  EXPECT_EQ(0U, trace->live_blocks);
  EXPECT_EQ(trace->worker_thread, trace->root_destroy_thread);
  owner = borrow();
  owner->resize({2}, generation(), 1);
  EXPECT_EQ(1U, owner->info({2}, generation()).size);
}

TEST_F(TypedRoot, ForeignPublisherAndThreadCannotModifyOrEndTheLoan)
{
  owner = borrow();
  EXPECT_THROW(owner->publish(kPublisher + 1), std::invalid_argument);
  EXPECT_THROW(owner->cancel(kPublisher + 1, false), std::invalid_argument);
  std::size_t rejected = 0;
  std::thread foreign([&] {
      const auto reject = [&](auto operation) {
        try {
          operation();
        } catch (const std::runtime_error &) {
          ++rejected;
        }
      };
      reject([&] {owner->info({}, 0);});
      reject([&] {owner->read({0}, 0);});
      reject([&] {owner->write({0}, 0, ScalarValue{});});
      reject([&] {owner->resize({1}, 0, 1);});
      reject([&] {owner->acquire_export({1}, 0);});
      reject([&] {owner->publish(kPublisher);});
      reject([&] {owner->cancel(kPublisher, false);});
    });
  foreign.join();
  EXPECT_EQ(7U, rejected);
  EXPECT_EQ(RootPhase::kPrepare, owner->snapshot().phase);
  EXPECT_EQ(0U, generation());
  EXPECT_EQ(0U, trace->publishes);
  EXPECT_EQ(0U, trace->cancels);
  owner->resize({1}, generation(), 1);
}

TEST_F(TypedRoot, LastExportReleaseOnAnotherThreadQueuesWorkerOnlyCancellation)
{
  owner = borrow();
  owner->resize({1}, generation(), 2);
  const auto first = owner->acquire_export({1}, generation());
  const auto second = owner->acquire_export({1}, generation());
  EXPECT_EQ(first.data, second.data);
  EXPECT_THROW(owner->cancel(kPublisher, false), BufferBusy);
  owner->cancel(kPublisher, true);
  EXPECT_EQ(RootPhase::kCancelPending, owner->snapshot().phase);
  EXPECT_EQ(2U, owner->snapshot().exports);
  owner->release_export();
  EXPECT_EQ(1U, owner->snapshot().exports);
  EXPECT_EQ(0U, trace->cancels);
  static_cast<std::int32_t *>(second.data)[1] = 88;
  EXPECT_EQ(88, trace->live->values[1]);
  std::thread::id release_thread;
  std::thread foreign([&] {
      release_thread = std::this_thread::get_id();
      owner->release_export();
    });
  foreign.join();
  ASSERT_EQ(PublisherWorkerError::kNone, worker->wait_until_idle().error);
  EXPECT_EQ(RootPhase::kCancelling, owner->snapshot().phase);
  EXPECT_EQ(1U, trace->cancels);
  EXPECT_EQ(1U, trace->destroyed_roots);
  EXPECT_EQ(trace->worker_thread, trace->root_destroy_thread);
  EXPECT_NE(release_thread, trace->root_destroy_thread);
  owner->release_export();
  EXPECT_THROW(owner->cancel(kPublisher, false), std::runtime_error);
  EXPECT_EQ(1U, trace->cancels);
  owner = borrow();
}

TEST_F(TypedRoot, AbandonedRootOnAnotherThreadIsDestroyedByTheWorker)
{
  owner = borrow();
  std::thread foreign([last = std::move(owner)]() mutable {last.reset();});
  foreign.join();
  ASSERT_EQ(PublisherWorkerError::kNone, worker->wait_until_idle().error);
  EXPECT_EQ(1U, trace->cancels);
  EXPECT_EQ(trace->worker_thread, trace->root_destroy_thread);
}

TEST_F(TypedRoot, FailedPublishIsNotReportedAsPublishedAndTheNextBorrowSucceeds)
{
  const LoanIdentity identity{kPublisher, 1, std::this_thread::get_id()};
  LoanLifecycle reference(identity);
  ASSERT_EQ(LoanError::kNone, reference.request_publish(identity).error);
  ASSERT_EQ(LoanError::kNone, reference.fail_publish());
  ASSERT_EQ(LoanPhase::kFailed, reference.phase());
  owner = borrow();
  trace->fail_publish = true;
  EXPECT_THROW(owner->publish(kPublisher), std::runtime_error);
  EXPECT_EQ(RootPhase::kFailed, owner->snapshot().phase);
  EXPECT_THROW(owner->info({}, 0), std::runtime_error);
  EXPECT_EQ(1U, trace->destroyed_roots);
  trace->fail_publish = false;
  owner = borrow();
  owner->publish(kPublisher);
  EXPECT_EQ(RootPhase::kPublished, owner->snapshot().phase);
}

TEST_F(TypedRoot, TerminalPublisherMismatchIsRejectedBeforePhase)
{
  const LoanIdentity identity{kPublisher, 1, std::this_thread::get_id()};
  LoanLifecycle reference(identity);
  ASSERT_EQ(LoanError::kNone, reference.request_publish(identity).error);
  ASSERT_EQ(LoanError::kNone, reference.complete_publish());
  EXPECT_EQ(
    LoanError::kWrongPublisher,
    reference.request_publish({kPublisher + 1, 1, identity.creator_thread}).error);
  owner = borrow();
  owner->publish(kPublisher);
  EXPECT_THROW(owner->publish(kPublisher + 1), std::invalid_argument);
  EXPECT_THROW(owner->cancel(kPublisher + 1, false), std::invalid_argument);
  EXPECT_EQ(1U, trace->publishes);
}

TEST_F(TypedRoot, QueuedCancellationDoesNotClaimWorkerCompletion)
{
  const LoanIdentity identity{kPublisher, 1, std::this_thread::get_id()};
  LoanLifecycle reference(identity);
  ASSERT_EQ(LoanError::kNone, reference.acquire_export().error);
  ASSERT_EQ(LoanError::kNone, reference.request_cancel(identity, true).error);
  ASSERT_EQ(LoanError::kNone, reference.release_export().error);
  ASSERT_EQ(LoanPhase::kCancelling, reference.phase());
  owner = borrow();
  owner->acquire_export({1}, 0);
  owner->cancel(kPublisher, true);
  owner->release_export();
  EXPECT_EQ(RootPhase::kCancelling, owner->snapshot().phase);
  ASSERT_EQ(PublisherWorkerError::kNone, worker->wait_until_idle().error);
  EXPECT_EQ(1U, trace->cancels);
  EXPECT_EQ(1U, trace->destroyed_roots);
}

TEST_F(TypedRoot, RejectedCancellationClosesAccessAndDoesNotRetry)
{
  const LoanIdentity identity{kPublisher, 1, std::this_thread::get_id()};
  LoanLifecycle reference(identity);
  ASSERT_EQ(LoanError::kNone, reference.request_cancel(identity, false).error);
  ASSERT_EQ(LoanPhase::kCancelling, reference.phase());
  EXPECT_EQ(LoanError::kInvalidPhase, reference.request_cancel(identity, false).error);
  const auto borrowed = worker->borrow_root(kPublisher);
  ASSERT_EQ(PublisherWorkerError::kNone, borrowed.error);
  owner = RootOwner::create(
    worker, kPublisher, borrowed.generation, borrowed.root, descriptor(), 1024 * 1024);
  // A stale worker token simulates cancellation rejection without throwing from noexcept cancel.
  ASSERT_EQ(PublisherWorkerError::kNone, worker->cancel(kPublisher, borrowed.generation).error);
  EXPECT_THROW(owner->cancel(kPublisher, false), std::runtime_error);
  EXPECT_EQ(RootPhase::kCancelling, owner->snapshot().phase);
  EXPECT_THROW(owner->info({}, 0), std::runtime_error);
  EXPECT_THROW(owner->cancel(kPublisher, false), std::runtime_error);
  EXPECT_EQ(1U, trace->cancels);
}

TEST_F(TypedRoot, PackedBooleanResizeFitsTheByteBudgetWithoutChargingOneBytePerBit)
{
  constexpr std::size_t bits = 1024;
  owner = borrow(sizeof(Message) + bits / 8);
  ASSERT_GT(bits, sizeof(Message) + bits / 8);
  ASSERT_NO_THROW(owner->resize({3}, generation(), bits));
  ASSERT_NE(nullptr, trace->live);
  EXPECT_EQ(bits, trace->live->flags.size());
  const auto capacity = trace->live->flags.capacity();
  EXPECT_EQ(
    sizeof(Message) + capacity / 8 + (capacity % 8 != 0),
    owner->snapshot().accounted_bytes);
  EXPECT_EQ(trace->worker_thread, trace->allocation_thread);
}

TEST_F(TypedRoot, ObviousStringSequenceBudgetOverflowRejectsBeforeNativeAllocation)
{
  owner = borrow(sizeof(Message) + 64);
  const auto allocations = trace->allocations;
  EXPECT_THROW(owner->resize({4}, generation(), 1000), std::invalid_argument);
  EXPECT_EQ(allocations, trace->allocations);
  EXPECT_TRUE(trace->live->texts.empty());
  EXPECT_EQ(RootPhase::kPrepare, owner->snapshot().phase);
  EXPECT_EQ(0U, generation());
  owner->resize({4}, generation(), 1);
  EXPECT_EQ(1U, trace->live->texts.size());
}

TEST_F(TypedRoot, NarrowAndWideInlineStringsResizeWithinTheExactRootBudget)
{
  owner = borrow(sizeof(Message));
  auto & native = *trace->live;
  const auto text_pointer = native.text.data();
  const auto wide_pointer = native.wide_text.data();
  const auto text_capacity = native.text.capacity();
  const auto wide_capacity = native.wide_text.capacity();
  ASSERT_GT(text_capacity, 0U);
  ASSERT_GT(wide_capacity, 0U);
  ASSERT_GE(
    reinterpret_cast<std::uintptr_t>(text_pointer),
    reinterpret_cast<std::uintptr_t>(&native.text));
  ASSERT_LT(
    reinterpret_cast<std::uintptr_t>(text_pointer),
    reinterpret_cast<std::uintptr_t>(&native.text) + sizeof(Text));
  ASSERT_GE(
    reinterpret_cast<std::uintptr_t>(wide_pointer),
    reinterpret_cast<std::uintptr_t>(&native.wide_text));
  ASSERT_LT(
    reinterpret_cast<std::uintptr_t>(wide_pointer),
    reinterpret_cast<std::uintptr_t>(&native.wide_text) + sizeof(WideText));
  const auto allocations = trace->allocations;

  ASSERT_NO_THROW(owner->resize({5}, generation(), text_capacity));
  ASSERT_NO_THROW(owner->resize({6}, generation(), wide_capacity));
  EXPECT_EQ(text_capacity, native.text.size());
  EXPECT_EQ(wide_capacity, native.wide_text.size());
  EXPECT_EQ(text_pointer, native.text.data());
  EXPECT_EQ(wide_pointer, native.wide_text.data());
  EXPECT_EQ(allocations, trace->allocations);
  EXPECT_EQ(sizeof(Message), owner->snapshot().accounted_bytes);
  EXPECT_EQ(RootPhase::kPrepare, owner->snapshot().phase);
}

TEST_F(TypedRoot, StringGrowthBeyondInlineCapacityRejectsBeforeAllocationAndRemainsPrepare)
{
  owner = borrow(sizeof(Message));
  const auto text_pointer = trace->live->text.data();
  const auto wide_pointer = trace->live->wide_text.data();
  const auto allocations = trace->allocations;
  EXPECT_THROW(
    owner->resize({5}, generation(), trace->live->text.capacity() + 1),
    std::invalid_argument);
  EXPECT_THROW(
    owner->resize({6}, generation(), trace->live->wide_text.capacity() + 1),
    std::invalid_argument);
  EXPECT_EQ(allocations, trace->allocations);
  EXPECT_TRUE(trace->live->text.empty());
  EXPECT_TRUE(trace->live->wide_text.empty());
  EXPECT_EQ(text_pointer, trace->live->text.data());
  EXPECT_EQ(wide_pointer, trace->live->wide_text.data());
  EXPECT_EQ(RootPhase::kPrepare, owner->snapshot().phase);
  EXPECT_EQ(0U, generation());
  EXPECT_EQ(sizeof(Message), owner->snapshot().accounted_bytes);
  EXPECT_NO_THROW(owner->resize({5}, generation(), 1));
  EXPECT_NO_THROW(owner->resize({6}, generation(), 1));
}
}  // namespace
