// Copyright 2026 rclpy zerocopy maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "subscriber/queue.hpp"

namespace
{
using rclpy_agnocast::SubscriberQueue;
const std::uint8_t payload[]{1, 2, 3, 4, 5, 6, 7};

TEST(SubscriberQueue, WakeCoalescesUntilDrainFinishesAndRearmsAfterEmpty)
{
  SubscriberQueue queue(4, 28);
  auto storage = std::make_shared<int>(0);
  ASSERT_TRUE(queue.push({1, payload, 7, storage}));
  const auto first = queue.wait_for_wake(0);
  ASSERT_NE(first, 0U);
  ASSERT_TRUE(queue.push({2, payload, 7, storage}));
  EXPECT_EQ(queue.wait_for_wake(0), first);
  queue.take()->release();
  queue.take()->release();
  EXPECT_EQ(queue.take(), nullptr);
  queue.finish_dispatch();
  ASSERT_TRUE(queue.push({3, payload, 7, storage}));
  EXPECT_GT(queue.wait_for_wake(first), first);
}

TEST(SubscriberQueue, EnqueueBetweenEmptyTakeAndDrainCompletionCannotLoseWake)
{
  SubscriberQueue queue(2, 14);
  auto storage = std::make_shared<int>(0);
  ASSERT_TRUE(queue.push({1, payload, 7, storage}));
  auto generation = queue.wait_for_wake(0);
  for (unsigned index = 0; index < 100; ++index) {
    queue.take()->release();
    EXPECT_EQ(queue.take(), nullptr);
    std::thread producer([&]() {queue.push({index + 2, payload, 7, storage});});
    producer.join();
    // Still armed: this enqueue was coalesced with the dispatch just ending.
    EXPECT_EQ(queue.wait_for_wake(0), generation);
    queue.finish_dispatch();
    const auto next = queue.wait_for_wake(generation);
    EXPECT_GT(next, generation);
    generation = next;
  }
}

TEST(SubscriberQueue, StopWakesAnIdleWaiterWithoutADataNotification)
{
  SubscriberQueue queue(1, 7);
  auto waiter = std::async(std::launch::async, [&]() {return queue.wait_for_wake(0);});
  EXPECT_EQ(waiter.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
  queue.stop_admission();
  ASSERT_EQ(waiter.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(waiter.get(), 0U);
}

TEST(SubscriberQueue, NativeFailureWakesWaiterAndExplicitCloseRemainsPossible)
{
  SubscriberQueue queue(1, 7);
  auto waiter = std::async(std::launch::async, [&]() {return queue.wait_for_wake(0);});
  queue.fail(std::make_exception_ptr(std::runtime_error("receive failed")));
  ASSERT_EQ(waiter.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_THROW(waiter.get(), std::runtime_error);
  queue.stop_admission();
  EXPECT_EQ(queue.wait_for_wake(0), 0U);
}

TEST(SubscriberQueue, FailureCheckDoesNotTreatExplicitStopAsFailure)
{
  SubscriberQueue queue(1, 7);
  EXPECT_NO_THROW(queue.rethrow_failure());
  queue.stop_admission();
  EXPECT_NO_THROW(queue.rethrow_failure());
}

TEST(SubscriberQueue, FirstFailureSurvivesLaterFailureAndStorageCleanup)
{
  for (const bool receive_first : {false, true}) {
    SubscriberQueue queue(1, 7);
    ASSERT_TRUE(queue.push({1, payload, 7, std::make_shared<int>(0)}));
    const auto receive = std::make_exception_ptr(std::logic_error("receive failure"));
    const auto executor = std::make_exception_ptr(std::runtime_error("executor failure"));
    queue.fail(receive_first ? receive : executor);
    queue.fail(receive_first ? executor : receive);
    if (receive_first) {
      EXPECT_THROW(queue.wait_for_wake(0), std::logic_error);
      EXPECT_THROW(queue.rethrow_failure(), std::logic_error);
    } else {
      EXPECT_THROW(queue.wait_for_wake(0), std::runtime_error);
      EXPECT_THROW(queue.rethrow_failure(), std::runtime_error);
    }
    EXPECT_TRUE(queue.drain_for_close());
    EXPECT_EQ(queue.stats().outstanding_leases, 0U);
    EXPECT_EQ(queue.wait_for_wake(0), 0U);
    if (receive_first) {
      EXPECT_THROW(queue.rethrow_failure(), std::logic_error);
    } else {
      EXPECT_THROW(queue.rethrow_failure(), std::runtime_error);
    }
  }
}

TEST(SubscriberQueue, NativeStorageReleasePrecedesAdmissionBudgetReturn)
{
  SubscriberQueue queue(1, 7);
  std::size_t released = 0;
  auto storage = std::shared_ptr<int>(
    new int(0), [&](int * pointer) {
      EXPECT_EQ(queue.stats().outstanding_leases, 1U);
      EXPECT_EQ(queue.stats().outstanding_payload_bytes, 7U);
      ++released;
      delete pointer;
    });
  ASSERT_TRUE(queue.push({1, payload, 7, std::move(storage)}));
  std::size_t keepalive_released = 0;
  auto keepalive = std::shared_ptr<int>(
    new int(0), [&](int * pointer) {
      EXPECT_EQ(released, 1U);
      EXPECT_EQ(queue.stats().outstanding_leases, 0U);
      EXPECT_EQ(queue.stats().outstanding_payload_bytes, 0U);
      ++keepalive_released;
      delete pointer;
    });
  auto lease = queue.take(std::move(keepalive));
  lease->release();
  lease->release();
  EXPECT_EQ(released, 1U);
  EXPECT_EQ(keepalive_released, 1U);
  EXPECT_EQ(queue.stats().outstanding_leases, 0U);
}

TEST(SubscriberQueue, CloseDrainRejectsAdmissionAndPreservesDeliveredStorageUntilRetry)
{
  SubscriberQueue queue(2, 14);
  auto storage = std::make_shared<int>(0);
  ASSERT_TRUE(queue.push({1, payload, 7, storage}));
  auto held = queue.take();
  ASSERT_TRUE(queue.push({2, payload, 7, storage}));
  EXPECT_FALSE(queue.drain_for_close());
  EXPECT_EQ(queue.stats().queued_leases, 0U);
  EXPECT_EQ(queue.stats().outstanding_leases, 1U);
  EXPECT_EQ(queue.stats().outstanding_payload_bytes, 7U);
  EXPECT_FALSE(queue.push({3, payload, 7, storage}));
  held->release();
  EXPECT_TRUE(queue.drain_for_close());
  EXPECT_TRUE(queue.drain_for_close());
  EXPECT_EQ(queue.stats().outstanding_payload_bytes, 0U);
}

TEST(SubscriberQueue, ByteAdmissionCannotOverflowAtSizeMaximum)
{
  const auto maximum = std::numeric_limits<std::size_t>::max();
  SubscriberQueue queue(2, maximum);
  auto storage = std::make_shared<int>(0);
  // Only accounting is exercised: the synthetic size is never exported or dereferenced.
  ASSERT_TRUE(queue.push({1, payload, maximum, storage}));
  ASSERT_FALSE(queue.push({2, payload, 1, storage}));
  EXPECT_EQ(queue.stats().outstanding_payload_bytes, maximum);
  EXPECT_EQ(queue.stats().dropped_outstanding, 1U);
  queue.clear();
  EXPECT_EQ(queue.stats().outstanding_payload_bytes, 0U);
  ASSERT_TRUE(queue.push({3, payload, 1, storage}));
}

TEST(SubscriberQueue, ConcurrentProducersAndConsumerRespectBoundsAndReleaseEveryStorage)
{
  SubscriberQueue queue(4, 28);
  std::atomic<unsigned> producers_done{0};
  std::atomic<unsigned> released{0};
  std::atomic<bool> bounds_valid{true};
  std::thread consumer([&]() {
      while (producers_done != 4 || queue.stats().queued_leases != 0) {
        if (auto lease = queue.take()) {lease->release();}
        const auto stats = queue.stats();
        if (stats.outstanding_leases > 4 || stats.outstanding_payload_bytes > 28) {
          bounds_valid = false;
        }
        std::this_thread::yield();
      }
    });
  std::vector<std::thread> producers;
  for (unsigned producer = 0; producer < 4; ++producer) {
    producers.emplace_back(
      [&, producer]() {
        for (unsigned index = 0; index < 200; ++index) {
          auto storage = std::shared_ptr<int>(
            new int(0), [&](int * pointer) {
              delete pointer;
              ++released;
            });
          queue.push({producer * 200 + index, payload, 7, std::move(storage)});
        }
        ++producers_done;
      });
  }
  for (auto & producer : producers) {
    producer.join();
  }
  consumer.join();
  const auto stats = queue.stats();
  EXPECT_TRUE(bounds_valid);
  EXPECT_EQ(stats.received_total, 800U);
  EXPECT_EQ(stats.delivered_total + stats.dropped_outstanding, 800U);
  EXPECT_EQ(stats.outstanding_leases, 0U);
  EXPECT_EQ(stats.outstanding_payload_bytes, 0U);
  EXPECT_EQ(released, 800U);
}
}  // namespace
