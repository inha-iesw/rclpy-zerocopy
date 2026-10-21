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

#include <stdexcept>
#include <thread>

#include "publisher/observation.hpp"

namespace obs = rclpy_agnocast::observation;

class PublisherObservationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    obs::reset();
    obs::record(obs::Event::kCreate);
  }
  void TearDown() override {obs::disable();}
};

TEST_F(PublisherObservationTest, FailureTargetsOnlyNthSharedAllocationInSelectedPhase)
{
  obs::arm_failure(obs::Stage::kResize, 1);
  obs::record(obs::Event::kBorrow);
  EXPECT_FALSE(obs::allocator_event(0, nullptr, 32, true));
  obs::record(obs::Event::kResize);
  EXPECT_FALSE(obs::allocator_event(0, nullptr, 32, false));
  EXPECT_FALSE(obs::allocator_event(0, nullptr, 32, true));
  EXPECT_TRUE(obs::allocator_event(0, nullptr, 32, true));
  EXPECT_FALSE(obs::allocator_event(0, nullptr, 32, true));
  EXPECT_EQ(obs::snapshot().injected_failures, 1U);
}

TEST_F(PublisherObservationTest, FailureAndWorkerCountsExcludeOtherThreads)
{
  obs::arm_failure(obs::Stage::kBorrow, 0);
  obs::record(obs::Event::kBorrow);
  std::thread caller([] {EXPECT_FALSE(obs::allocator_event(0, nullptr, 32, true));});
  caller.join();
  EXPECT_EQ(obs::snapshot().worker_allocations, 0U);
  EXPECT_TRUE(obs::allocator_event(0, nullptr, 32, true));
}

TEST_F(PublisherObservationTest, FillingCountsBothHeapKindsAndSnapshotDoesNotChangeCounters)
{
  obs::filling(true);
  EXPECT_FALSE(obs::allocator_event(0, nullptr, 32, false));
  EXPECT_FALSE(obs::allocator_event(0, nullptr, 32, true));
  EXPECT_EQ(obs::snapshot().filling_allocations, 2U);
  EXPECT_EQ(obs::snapshot().filling_allocations, 2U);
  obs::filling(false);
  EXPECT_FALSE(obs::allocator_event(0, nullptr, 32, true));
  EXPECT_EQ(obs::snapshot().filling_allocations, 2U);
  EXPECT_EQ(obs::snapshot().worker_allocations, 3U);
}

TEST_F(PublisherObservationTest, UnpublishedDestructionUsesActualMatchingFree)
{
  int message;
  int other;
  obs::record(obs::Event::kResize, reinterpret_cast<std::uintptr_t>(&message));
  obs::allocator_event(2, &other, 0, true);
  EXPECT_EQ(obs::snapshot().unpublished_destroy, 0U);
  obs::record(obs::Event::kCancel);
  obs::allocator_event(2, &message, 0, true);
  EXPECT_EQ(obs::snapshot().unpublished_destroy, 1U);
  EXPECT_EQ(obs::snapshot().destroy_tid, obs::snapshot().worker_tid);
}

TEST_F(PublisherObservationTest, InjectionCannotBeArmedWhileFillingOrAlreadyArmed)
{
  obs::filling(true);
  EXPECT_THROW(obs::arm_failure(obs::Stage::kBorrow, 0), std::runtime_error);
  obs::filling(false);
  obs::arm_failure(obs::Stage::kBorrow, 0);
  EXPECT_THROW(obs::arm_failure(obs::Stage::kResize, 0), std::runtime_error);
  EXPECT_THROW(obs::arm_failure(obs::Stage::kIdle, 0), std::invalid_argument);
}

TEST_F(PublisherObservationTest, FailureTracksPriorStorageUntilFreedOnAnyThread)
{
  int message, control, exception;
  obs::arm_failure(obs::Stage::kResize, 0);
  obs::record(obs::Event::kBorrow);
  obs::allocator_event(1, &message, 32, true);
  obs::allocator_event(1, &control, 32, true);
  EXPECT_EQ(obs::snapshot().failure_live_allocations, 2U);
  obs::record(obs::Event::kResize);
  EXPECT_TRUE(obs::allocator_event(0, nullptr, 128, true));
  // Exception storage after the fault is owned by exception_ptr, not the failed loan.
  obs::allocator_event(1, &exception, 32, true);
  EXPECT_EQ(obs::snapshot().failure_tracked_allocations, 2U);
  obs::record(obs::Event::kResizeFailed);
  EXPECT_THROW(obs::arm_failure(obs::Stage::kBorrow, 0), std::runtime_error);
  obs::allocator_event(2, &message, 0, true);
  std::thread cleanup([&control] {obs::allocator_event(2, &control, 0, true);});
  cleanup.join();
  EXPECT_EQ(obs::snapshot().failure_live_allocations, 0U);
  EXPECT_EQ(obs::snapshot().failure_tracking_overflow, 0U);
}
