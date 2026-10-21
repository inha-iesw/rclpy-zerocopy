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

#include "observation.hpp"

#ifdef BUILD_TESTING

#include <dlfcn.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <array>
#include <mutex>
#include <stdexcept>

namespace rclpy_agnocast::observation
{
namespace
{
using Observer = bool (*)(std::uint32_t, void *, std::size_t, bool);
using SetObserver = void (*)(Observer);
using IsShared = bool (*)(const void *);
using ReadTls = std::uint32_t (*)();

std::mutex mutex;
Snapshot counters;
bool enabled{false};
Stage stage{Stage::kIdle};
Stage failure_stage{Stage::kIdle};
std::size_t remaining{0};
SetObserver set_observer{nullptr};
IsShared shared_address{nullptr};
ReadTls read_tls{nullptr};
// Track only storage acquired before the injected fault, excluding exception_ptr storage.
std::array<void *, 16> failure_storage{};

std::uint64_t tid() noexcept {return static_cast<std::uint64_t>(syscall(SYS_gettid));}
std::uint64_t tls() noexcept {return read_tls ? read_tls() : 0;}
}  // namespace

void reset() noexcept
{
  std::lock_guard<std::mutex> lock(mutex);
  counters = {};
  failure_storage.fill(nullptr);
  stage = failure_stage = Stage::kIdle;
  enabled = true;
}

void enable()
{
  set_observer = reinterpret_cast<SetObserver>(
    dlsym(RTLD_DEFAULT, "agnocast_heaphook_test_set_observer"));
  shared_address = reinterpret_cast<IsShared>(
    dlsym(RTLD_DEFAULT, "agnocast_heaphook_test_is_shared"));
  read_tls = reinterpret_cast<ReadTls>(
    dlsym(RTLD_DEFAULT, "agnocast_get_borrowed_publisher_num"));
  if (!set_observer || !shared_address || !read_tls) {
    throw std::runtime_error(
            "production observations require the test-observation heaphook feature");
  }
  reset();
  set_observer(allocator_event);
}

void disable()
{
  if (set_observer) {set_observer(nullptr);}
  std::lock_guard<std::mutex> lock(mutex);
  enabled = false;
  failure_stage = Stage::kIdle;
}

Snapshot snapshot()
{
  Snapshot result;
  {
    std::lock_guard<std::mutex> lock(mutex);
    result = counters;
  }
  result.caller_tid = tid();
  result.caller_tls = tls();
  return result;
}

void arm_failure(const Stage target, const std::size_t ordinal)
{
  if (target != Stage::kBorrow && target != Stage::kResize) {
    throw std::invalid_argument("allocation failure stage must be borrow or resize");
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (enabled && stage == Stage::kIdle && failure_stage == Stage::kIdle &&
      counters.failure_live_allocations == 0)
    {
      failure_stage = target;
      remaining = ordinal;
      counters.failure_tracked_allocations = counters.failure_tracking_overflow = 0;
      return;
    }
  }
  // Constructing exceptions may re-enter the allocator observer.
  throw std::runtime_error("allocation injection requires an idle observed publisher");
}

bool is_shared(const std::uintptr_t address)
{
  if (!shared_address) {throw std::runtime_error("heaphook observation is not enabled");}
  return shared_address(reinterpret_cast<const void *>(address));
}

void record(const Event event, const std::uintptr_t message, const std::uintptr_t data) noexcept
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!enabled) {return;}
  ++counters.backend_epoch;
  const auto current_tid = tid();
  switch (event) {
    case Event::kCreate:
      counters.worker_tid = current_tid;
      break;
    case Event::kBorrow:
      stage = Stage::kBorrow;
      ++counters.borrow_entries;
      counters.borrow_tid = current_tid;
      break;
    case Event::kResize:
      stage = Stage::kResize;
      counters.message_address = message;
      ++counters.resize_entries;
      counters.resize_tid = current_tid;
      break;
    case Event::kBorrowed:
      counters.data_address = data;
      counters.message_address = message;
      counters.borrow_tls = tls();
      ++counters.borrow_complete;
      break;
    case Event::kPublish:
      stage = Stage::kPublish;
      counters.publish_tid = current_tid;
      break;
    case Event::kPublished:
      counters.terminal_tls = tls();
      stage = Stage::kIdle;
      break;
    case Event::kCancel:
      stage = Stage::kCancel;
      counters.cancel_tid = current_tid;
      break;
    case Event::kCancelled:
      counters.terminal_tls = tls();
      stage = Stage::kIdle;
      break;
    case Event::kBorrowFailed:
    case Event::kResizeFailed:
      counters.terminal_tls = tls();
      stage = Stage::kIdle;
      break;
  }
}

void filling(const bool active) noexcept
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!enabled) {return;}
  stage = active ? Stage::kFilling : Stage::kIdle;
}

void enqueued(const bool asynchronous) noexcept
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!enabled) {return;}
  ++counters.terminal_enqueues;
  if (asynchronous) {++counters.async_cancel_enqueues;}
  counters.last_enqueue_tid = tid();
}

void completed(const bool published) noexcept
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!enabled) {return;}
  if (published) {++counters.publish_complete;} else {++counters.cancel_complete;}
}

bool allocator_event(
  const std::uint32_t event, void * pointer, std::size_t, const bool shared) noexcept
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!enabled) {return false;}
  const auto current_tid = tid();
  if (event == 2 && shared) {
    for (auto & allocation : failure_storage) {
      if (allocation == pointer && allocation != nullptr) {
        allocation = nullptr;
        --counters.failure_live_allocations;
        break;
      }
    }
  }
  if (event == 1 && shared && current_tid == counters.worker_tid &&
    failure_stage != Stage::kIdle && (stage == Stage::kBorrow || stage == Stage::kResize))
  {
    for (auto & allocation : failure_storage) {
      if (allocation == nullptr) {
        allocation = pointer;
        ++counters.failure_tracked_allocations;
        ++counters.failure_live_allocations;
        return false;
      }
    }
    ++counters.failure_tracking_overflow;
    return false;
  }
  if (
    event == 2 && shared &&
    reinterpret_cast<std::uintptr_t>(pointer) == counters.message_address &&
    (stage == Stage::kCancel || stage == Stage::kResize))
  {
    ++counters.unpublished_destroy;
    counters.destroy_tid = current_tid;
  }
  if (event != 0 || current_tid != counters.worker_tid) {return false;}
  ++counters.worker_allocations;
  if (shared) {++counters.shared_allocations;}
  if (stage == Stage::kFilling) {++counters.filling_allocations;}
  if (!shared || failure_stage == Stage::kIdle || stage != failure_stage) {return false;}
  if (remaining != 0) {
    --remaining;
    return false;
  }
  failure_stage = Stage::kIdle;
  ++counters.injected_failures;
  return true;
}

}  // namespace rclpy_agnocast::observation

#endif
