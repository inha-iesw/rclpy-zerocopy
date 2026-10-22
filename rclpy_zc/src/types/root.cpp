// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include "root.hpp"
#include <limits>
#include <utility>
#include <string>
#include "../publisher/observation.hpp"

namespace rclpy_agnocast
{
namespace types
{
namespace
{
void check_worker(const PublisherWorkerResult & result)
{
  if (result.failure) {std::rethrow_exception(result.failure);}
  if (result.error != PublisherWorkerError::kNone) {
    throw std::runtime_error("typed publisher worker rejected command");
  }
}
void check_lifecycle(LoanError error)
{
  switch (error) {
    case LoanError::kNone: return;
    case LoanError::kWrongPublisher:
      throw std::invalid_argument("typed loan belongs to another publisher");
    case LoanError::kStaleGeneration:
      throw std::invalid_argument("typed loan has a stale worker generation");
    case LoanError::kWrongThread:
      throw std::runtime_error("typed loan must be used on its creator thread");
    case LoanError::kActiveExports:
      throw BufferBusy("writable buffer aliases are still active");
    case LoanError::kInvalidPhase:
    case LoanError::kNoActiveExport:
      throw std::runtime_error("typed loan is terminal or cancellation is pending");
  }
}
}  // namespace
RootOwner::RootOwner(
  std::shared_ptr<PublisherWorker> worker, std::uintptr_t publisher_id,
  std::uint64_t loan_generation, void * root, const TypeDescriptor & descriptor,
  std::size_t maximum)
: worker_(std::move(worker)), publisher_id_(publisher_id), loan_generation_(loan_generation),
  creator_(std::this_thread::get_id()), lifecycle_({publisher_id, loan_generation, creator_}),
  root_(root), descriptor_(descriptor), maximum_(maximum) {}

std::shared_ptr<RootOwner> RootOwner::create(
  std::shared_ptr<PublisherWorker> worker, std::uintptr_t publisher_id,
  std::uint64_t loan_generation, void * root, const TypeDescriptor & descriptor,
  std::size_t maximum)
{
  std::shared_ptr<RootOwner> result;
  try {
    result.reset(new RootOwner(worker, publisher_id, loan_generation, root, descriptor, maximum));
    result->accounted_ = result->account();
    if (result->accounted_ > maximum) {
      throw std::invalid_argument("default root exceeds max_storage_bytes");
    }
    return result;
  } catch (...) {
    if (result) {
      try {
        result->cancel(publisher_id, false);
      } catch (...) {
      }
    } else {
      (void)worker->cancel(publisher_id, loan_generation);
    }
    throw;
  }
}

RootOwner::~RootOwner() {abandon();}
void RootOwner::check_thread() const
{
  if (creator_ != std::this_thread::get_id()) {
    throw std::runtime_error("typed loan must be used on its creator thread");
  }
}
FieldRef RootOwner::resolve(const FieldPath & path, std::uint64_t generation) const
{
  check_thread();
  if (lifecycle_.phase() != LoanPhase::kFilling ||
    structure_phase_ == RootPhase::kCancelOnly)
  {
    throw std::runtime_error("typed loan is terminal or cancel-only");
  }
  if (!path.empty() && generation != generation_) {
    throw std::runtime_error("field proxy has a stale structural generation");
  }
  FieldRef ref{root_, descriptor_.ops->root};
  for (auto index : path) {
    if (!ref.ops->child) {throw std::invalid_argument("scalar has no child fields");}
    FieldRef next;
    check_status(ref.ops->child(ref, index, &next));
    ref = next;
  }
  return ref;
}
FieldInfo RootOwner::info(const FieldPath & path, std::uint64_t generation) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ref = resolve(path, generation);
  std::size_t size = ref.ops->member_count;
  if (ref.ops->size) {check_status(ref.ops->size(ref, &size));}
  return {ref.ops, size, generation_};
}
ScalarValue RootOwner::read(const FieldPath & path, std::uint64_t generation) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ref = resolve(path, generation);
  if (!ref.ops->read) {throw std::invalid_argument("field is not a scalar");}
  ScalarValue value{};
  check_status(ref.ops->read(ref, &value));
  return value;
}
std::string RootOwner::text_bytes(const FieldPath & path, std::uint64_t generation) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ref = resolve(path, generation);
  if (ref.ops->kind != FieldKind::kString && ref.ops->kind != FieldKind::kWString) {
    throw std::invalid_argument("to_text requires a string field");
  }
  BufferSpan span;
  check_status(ref.ops->buffer(ref, &span));
  if (!span.size) {return std::string();}
  return std::string(static_cast<const char *>(span.data), span.size * span.item_size);
}
void RootOwner::write(const FieldPath & path, std::uint64_t generation, ScalarValue value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ref = resolve(path, generation);
  if (!ref.ops->write) {throw std::invalid_argument("field is not a scalar");}
  check_status(ref.ops->write(ref, value));
  structure_phase_ = RootPhase::kFilling;
}
std::size_t RootOwner::account() const
{
  std::size_t dynamic = 0;
  check_status(descriptor_.ops->root->dynamic_bytes({root_, descriptor_.ops->root}, &dynamic));
  if (dynamic > std::numeric_limits<std::size_t>::max() - descriptor_.native_size) {
    throw std::invalid_argument("accounted storage overflow");
  }
  return descriptor_.native_size + dynamic;
}
void RootOwner::resize(const FieldPath & path, std::uint64_t generation, std::size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ref = resolve(path, generation);
  if (!ref.ops->resize) {throw std::invalid_argument("field has fixed structure");}
  if (structure_phase_ != RootPhase::kPrepare || lifecycle_.export_count()) {
    throw BufferBusy("resize requires PREPARE before the first write or buffer export");
  }
  if (size > ref.ops->bound) {throw std::invalid_argument("container bound exceeded");}
  const auto available = maximum_ - descriptor_.native_size;
  std::size_t minimum = 0;
  check_status(ref.ops->minimum_resize_bytes(ref, size, &minimum));
  if (minimum > available) {
    throw std::invalid_argument("requested size exceeds max_storage_bytes");
  }
  if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::runtime_error("structural generation exhausted");
  }
  struct Task {FieldRef ref; std::size_t size;};
  Task task{ref, size};
  const auto result = worker_->operate(
    publisher_id_, loan_generation_, [](void * raw) {
      auto & task = *static_cast<Task *>(raw);
#ifdef BUILD_TESTING
      observation::record(observation::Event::kResize);
#endif
      check_status(task.ref.ops->resize(task.ref, task.size));
    }, &task);
  try {
    check_worker(result);
    ++generation_;
    accounted_ = account();
    if (accounted_ > maximum_) {
      throw std::invalid_argument("actual capacity exceeds max_storage_bytes");
    }
  } catch (...) {
    // Provider errors may follow partial nested construction.
    // No partially changed root can publish.
    structure_phase_ = RootPhase::kCancelOnly;
    throw;
  }
}
BufferSpan RootOwner::acquire_export(const FieldPath & path, std::uint64_t generation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ref = resolve(path, generation);
  if (!ref.ops->buffer) {throw std::invalid_argument("field does not support a buffer");}
  BufferSpan span;
  check_status(ref.ops->buffer(ref, &span));
  if (lifecycle_.export_count() == std::numeric_limits<std::size_t>::max()) {
    throw BufferBusy("too many exports");
  }
  if (!span.item_size || !span.format || (span.size && !span.data) ||
    span.size > static_cast<std::size_t>(PTRDIFF_MAX) / span.item_size)
  {
    throw BufferBusy("invalid or oversized native buffer");
  }
  check_lifecycle(lifecycle_.acquire_export().error);
  structure_phase_ = RootPhase::kFilling;
  return span;
}
void RootOwner::release_export() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_cancel(lifecycle_.release_export().action);
}
void RootOwner::publish(std::uintptr_t publisher_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (publisher_id != publisher_id_) {
    throw std::invalid_argument("typed loan belongs to another publisher");
  }
  (void)resolve({}, generation_);
  if (lifecycle_.export_count()) {throw BufferBusy("writable buffer aliases are still active");}
  try {
    accounted_ = account();
    if (accounted_ > maximum_) {
      throw std::invalid_argument("actual capacity exceeds max_storage_bytes");
    }
  } catch (...) {
    structure_phase_ = RootPhase::kCancelOnly;
    throw;
  }
  check_lifecycle(lifecycle_.request_publish({publisher_id, loan_generation_, creator_}).error);
  const auto result = worker_->publish(publisher_id_, loan_generation_);
  root_ = nullptr;
  if (result.error == PublisherWorkerError::kNone) {
    (void)lifecycle_.complete_publish();
  } else {
    (void)lifecycle_.fail_publish();
  }
  check_worker(result);
}
void RootOwner::cancel(std::uintptr_t publisher_id, bool defer)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (publisher_id != publisher_id_) {
    throw std::invalid_argument("typed loan belongs to another publisher");
  }
  check_thread();
  const auto phase = lifecycle_.phase();
  // Context cleanup is idempotent once the worker has consumed this root, including failure.
  if (phase == LoanPhase::kPublished || phase == LoanPhase::kCancelled ||
    phase == LoanPhase::kFailed) {return;}
  const auto transition = lifecycle_.request_cancel(
    {publisher_id, loan_generation_, creator_},
    defer);
  check_lifecycle(transition.error);
  if (transition.action == LoanAction::kNone) {return;}
  check_worker(worker_->cancel(publisher_id_, loan_generation_));
  root_ = nullptr;
  (void)lifecycle_.complete_cancel();
}
void RootOwner::queue_cancel(LoanAction action) noexcept
{
  if (action != LoanAction::kCancel) {return;}
  root_ = nullptr;
  // As in LoanState, enqueue has no completion acknowledgement. Keep CANCELLING.
  (void)worker_->cancel_async(publisher_id_, loan_generation_);
}
void RootOwner::abandon() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_cancel(lifecycle_.request_abandon().action);
}
RootPhase RootOwner::phase_locked() const
{
  switch (lifecycle_.phase()) {
    case LoanPhase::kFilling: return structure_phase_;
    case LoanPhase::kCancelPending: return RootPhase::kCancelPending;
    case LoanPhase::kPublishing: return RootPhase::kPublishing;
    case LoanPhase::kCancelling: return RootPhase::kCancelling;
    case LoanPhase::kPublished: return RootPhase::kPublished;
    case LoanPhase::kCancelled: return RootPhase::kCancelled;
    case LoanPhase::kFailed: return RootPhase::kFailed;
  }
  throw std::logic_error("unknown loan lifecycle phase");
}
RootSnapshot RootOwner::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {phase_locked(), generation_, accounted_, lifecycle_.export_count()};
}
}  // namespace types
}  // namespace rclpy_agnocast
