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

#pragma once

#ifdef BUILD_TESTING

#include <cstddef>
#include <cstdint>

namespace rclpy_agnocast::observation
{

enum class Stage {kIdle, kBorrow, kResize, kFilling, kPublish, kCancel};
enum class Event {kCreate, kBorrow, kResize, kBorrowed, kPublish, kPublished, kCancel, kCancelled,
  kBorrowFailed, kResizeFailed};

// Fixed storage: recording and allocator callbacks must not allocate or log.
struct Snapshot
{
  std::uint64_t worker_tid{}, borrow_tid{}, resize_tid{}, publish_tid{}, cancel_tid{},
    destroy_tid{};
  std::uint64_t borrow_entries{}, borrow_complete{}, resize_entries{}, publish_complete{};
  std::uint64_t cancel_complete{}, unpublished_destroy{}, terminal_enqueues{},
    async_cancel_enqueues{};
  std::uint64_t last_enqueue_tid{}, terminal_tls{}, borrow_tls{}, data_address{}, message_address{};
  std::uint64_t backend_epoch{}, filling_allocations{}, worker_allocations{}, shared_allocations{};
  std::uint64_t injected_failures{}, caller_tid{}, caller_tls{};
  std::uint64_t failure_live_allocations{}, failure_tracked_allocations{},
    failure_tracking_overflow{};
};

// Called only before creating endpoints, with no active loan.
void enable();
void disable();
Snapshot snapshot();
void arm_failure(Stage stage, std::size_t ordinal);
bool is_shared(std::uintptr_t address);
void record(Event event, std::uintptr_t message = 0, std::uintptr_t data = 0) noexcept;
void filling(bool active) noexcept;
void enqueued(bool asynchronous) noexcept;
void completed(bool published) noexcept;

// Internal test seam uses the same callback installed in the real heaphook.
void reset() noexcept;
bool allocator_event(
  std::uint32_t event, void * pointer, std::size_t size, bool shared) noexcept;

}  // namespace rclpy_agnocast::observation

#endif
