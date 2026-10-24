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

#include "lease.hpp"

#include <stdexcept>
#include <utility>

#include "queue.hpp"

namespace rclpy_agnocast
{
LeaseBufferOwner::LeaseBufferOwner(
  ReceivedBuffer buffer, std::shared_ptr<SubscriberBudget> budget)
: buffer_(std::move(buffer)), budget_(std::move(budget)) {}

LeaseBufferOwner::~LeaseBufferOwner()
{
  // Native RELEASE_SUB_REF must finish before count zero permits endpoint destruction.
  buffer_.storage.reset();
  if (reserved_) {
    std::lock_guard<std::mutex> lock(budget_->mutex);
    --budget_->stats.outstanding_leases;
    budget_->retained_bytes -= buffer_.size;
    if (buffer_.descriptor) {budget_->stats.outstanding_accounted_bytes -= buffer_.size;} else {
      budget_->stats.outstanding_payload_bytes -= buffer_.size;
    }
  }
  // keepalive_ is destroyed after native storage and the reservation have been returned.
}

LeaseState::LeaseState(std::shared_ptr<LeaseBufferOwner> owner)
: owner_(std::move(owner)) {}

std::shared_ptr<LeaseBufferOwner> LeaseState::owner() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!owner_) {throw std::runtime_error("Lease has been released");}
  return owner_;
}

void LeaseState::release() noexcept
{
  std::shared_ptr<LeaseBufferOwner> released;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released = std::move(owner_);
  }
}
}  // namespace rclpy_agnocast
