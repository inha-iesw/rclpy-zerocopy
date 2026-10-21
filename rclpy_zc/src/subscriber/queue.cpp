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

#include "queue.hpp"

#include <stdexcept>
#include <utility>

namespace rclpy_agnocast
{
SubscriberQueue::SubscriberQueue(const std::size_t max_count, const std::size_t max_bytes)
: max_bytes_(max_bytes), budget_(std::make_shared<SubscriberBudget>()), ring_(max_count)
{
  if (max_count == 0 || max_bytes == 0) {
    throw std::invalid_argument("subscriber count and byte limits must be positive");
  }
}

SubscriberQueue::~SubscriberQueue() {clear();}

bool SubscriberQueue::push(ReceivedBuffer buffer)
{
  auto owner = std::make_shared<LeaseBufferOwner>(std::move(buffer), budget_);
  std::lock_guard<std::mutex> lock(budget_->mutex);
  auto & stats = budget_->stats;
  ++stats.received_total;
  if (!accepting_) {return false;}
  const auto size = owner->buffer().size;
  if (size > max_bytes_) {
    ++stats.dropped_oversize;
    return false;
  }
  // Subtraction avoids overflow even for a limit of SIZE_MAX.
  if (
    stats.outstanding_leases == ring_.size() ||
    size > max_bytes_ - stats.outstanding_payload_bytes)
  {
    ++stats.dropped_outstanding;
    return false;
  }
  owner->reserved_ = true;
  ring_[(head_ + stats.queued_leases) % ring_.size()] = std::move(owner);
  ++stats.queued_leases;
  ++stats.outstanding_leases;
  stats.outstanding_payload_bytes += size;
  if (!armed_) {
    armed_ = true;
    ++generation_;
    wake_.notify_one();
  }
  return true;
}

std::shared_ptr<LeaseState> SubscriberQueue::take(std::shared_ptr<void> keepalive)
{
  std::lock_guard<std::mutex> lock(budget_->mutex);
  auto & stats = budget_->stats;
  if (stats.queued_leases == 0) {return nullptr;}
  // Allocate before changing the ring so allocation failure leaves the queued owner intact.
  auto lease = std::make_shared<LeaseState>(ring_[head_]);
  ring_[head_]->keepalive_ = std::move(keepalive);
  ring_[head_].reset();
  head_ = (head_ + 1) % ring_.size();
  --stats.queued_leases;
  ++stats.delivered_total;
  return lease;
}

std::uint64_t SubscriberQueue::wait_for_wake(const std::uint64_t generation)
{
  std::unique_lock<std::mutex> lock(budget_->mutex);
  wake_.wait(
    lock, [&]() {
      return notifications_stopped_ || failure_ || generation_ != generation;
    });
  if (notifications_stopped_) {return 0;}
  if (failure_) {std::rethrow_exception(failure_);}
  return generation_;
}

void SubscriberQueue::finish_dispatch() noexcept
{
  std::lock_guard<std::mutex> lock(budget_->mutex);
  armed_ = budget_->stats.queued_leases != 0;
  if (armed_ && accepting_) {
    ++generation_;
    wake_.notify_one();
  }
}

void SubscriberQueue::fail(std::exception_ptr failure) noexcept
{
  std::lock_guard<std::mutex> lock(budget_->mutex);
  accepting_ = false;
  if (!failure_) {failure_ = std::move(failure);}
  wake_.notify_all();
}

void SubscriberQueue::stop_admission() noexcept
{
  std::lock_guard<std::mutex> lock(budget_->mutex);
  accepting_ = false;
  notifications_stopped_ = true;
  wake_.notify_all();
}

void SubscriberQueue::rethrow_failure() const
{
  std::lock_guard<std::mutex> lock(budget_->mutex);
  if (failure_) {std::rethrow_exception(failure_);}
}

void SubscriberQueue::clear() noexcept
{
  for (;; ) {
    std::shared_ptr<LeaseBufferOwner> released;
    {
      std::lock_guard<std::mutex> lock(budget_->mutex);
      if (budget_->stats.queued_leases == 0) {return;}
      released = std::move(ring_[head_]);
      head_ = (head_ + 1) % ring_.size();
      --budget_->stats.queued_leases;
    }
  }
}

SubscriberStats SubscriberQueue::stats() const noexcept
{
  std::lock_guard<std::mutex> lock(budget_->mutex);
  return budget_->stats;
}

bool SubscriberQueue::drain_for_close() noexcept
{
  stop_admission();
  clear();
  return stats().outstanding_leases == 0;
}
}  // namespace rclpy_agnocast
