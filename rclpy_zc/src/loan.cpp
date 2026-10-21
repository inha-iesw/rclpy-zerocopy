#include "rclpy_agnocast/loan.hpp"

#include <stdexcept>
#include <utility>

namespace rclpy_agnocast
{

LoanLifecycle::LoanLifecycle(LoanIdentity identity) : identity_(identity) {}

LoanTransition LoanLifecycle::acquire_export()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (std::this_thread::get_id() != identity_.creator_thread) {
    return {LoanError::kWrongThread, LoanAction::kNone};
  }
  if (phase_ != LoanPhase::kFilling) {
    return {LoanError::kInvalidPhase, LoanAction::kNone};
  }

  ++export_count_;
  return {LoanError::kNone, LoanAction::kNone};
}

LoanTransition LoanLifecycle::release_export()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (export_count_ == 0U) {
    return {LoanError::kNoActiveExport, LoanAction::kNone};
  }

  --export_count_;
  if (export_count_ == 0U && phase_ == LoanPhase::kCancelPending) {
    phase_ = LoanPhase::kCancelling;
    return {LoanError::kNone, LoanAction::kCancel};
  }
  return {LoanError::kNone, LoanAction::kNone};
}

LoanTransition LoanLifecycle::request_publish(const LoanIdentity & identity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto identity_error = validate_identity(identity);
  if (identity_error != LoanError::kNone) {
    return {identity_error, LoanAction::kNone};
  }
  if (phase_ != LoanPhase::kFilling) {
    return {LoanError::kInvalidPhase, LoanAction::kNone};
  }
  if (export_count_ != 0U) {
    return {LoanError::kActiveExports, LoanAction::kNone};
  }

  phase_ = LoanPhase::kPublishing;
  return {LoanError::kNone, LoanAction::kPublish};
}

LoanTransition LoanLifecycle::request_cancel(
  const LoanIdentity & identity, const bool defer_if_exported)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto identity_error = validate_identity(identity);
  if (identity_error != LoanError::kNone) {
    return {identity_error, LoanAction::kNone};
  }
  if (phase_ != LoanPhase::kFilling) {
    return {LoanError::kInvalidPhase, LoanAction::kNone};
  }
  if (export_count_ != 0U) {
    if (!defer_if_exported) {
      return {LoanError::kActiveExports, LoanAction::kNone};
    }
    phase_ = LoanPhase::kCancelPending;
    return {LoanError::kNone, LoanAction::kNone};
  }

  phase_ = LoanPhase::kCancelling;
  return {LoanError::kNone, LoanAction::kCancel};
}

LoanTransition LoanLifecycle::request_abandon()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != LoanPhase::kFilling) {
    return {LoanError::kInvalidPhase, LoanAction::kNone};
  }
  if (export_count_ != 0U) {
    phase_ = LoanPhase::kCancelPending;
    return {LoanError::kNone, LoanAction::kNone};
  }

  phase_ = LoanPhase::kCancelling;
  return {LoanError::kNone, LoanAction::kCancel};
}

LoanError LoanLifecycle::complete_publish()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != LoanPhase::kPublishing) {
    return LoanError::kInvalidPhase;
  }
  phase_ = LoanPhase::kPublished;
  return LoanError::kNone;
}

LoanError LoanLifecycle::fail_publish()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != LoanPhase::kPublishing) {
    return LoanError::kInvalidPhase;
  }
  phase_ = LoanPhase::kFailed;
  return LoanError::kNone;
}

LoanError LoanLifecycle::complete_cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != LoanPhase::kCancelling) {
    return LoanError::kInvalidPhase;
  }
  phase_ = LoanPhase::kCancelled;
  return LoanError::kNone;
}

LoanPhase LoanLifecycle::phase() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return phase_;
}

std::size_t LoanLifecycle::export_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return export_count_;
}

LoanError LoanLifecycle::validate_identity(const LoanIdentity & identity) const
{
  if (identity.publisher_id != identity_.publisher_id) {
    return LoanError::kWrongPublisher;
  }
  if (identity.generation != identity_.generation) {
    return LoanError::kStaleGeneration;
  }
  if (
    identity.creator_thread != identity_.creator_thread ||
    std::this_thread::get_id() != identity_.creator_thread)
  {
    return LoanError::kWrongThread;
  }
  return LoanError::kNone;
}

LoanState::LoanState(
  std::shared_ptr<PublisherWorker> worker, LoanIdentity identity, const BorrowedRegion region)
: worker_(std::move(worker)),
  identity_(identity),
  lifecycle_(identity),
  sequence_(region.sequence),
  data_(region.data),
  size_(region.size)
{
  if (
    !worker_ || region.generation != identity_.generation || sequence_ == nullptr ||
    (size_ != 0U && data_ == nullptr))
  {
    throw std::invalid_argument("invalid publisher loan region");
  }
}

LoanState::~LoanState() {abandon();}

WritableExportAcquireResult LoanState::acquire_writable_export(WritableBufferSpan & span) noexcept
{
  const auto transition = lifecycle_.acquire_export();
  if (transition.error == LoanError::kWrongThread) {
    return WritableExportAcquireResult::kWrongThread;
  }
  if (transition.error != LoanError::kNone) {
    return WritableExportAcquireResult::kNotFilling;
  }
  std::lock_guard<std::mutex> lock(region_mutex_);
  span = {data_, size_};
  return WritableExportAcquireResult::kOk;
}

void LoanState::release_writable_export() noexcept
{
  try {
    queue_cancel(lifecycle_.release_export().action);
  } catch (...) {
  }
}

LoanOperationResult LoanState::publish(const std::uintptr_t publisher_id)
{
  const auto transition = lifecycle_.request_publish(caller_identity(publisher_id));
  if (transition.error != LoanError::kNone) {
    return {transition.error, PublisherWorkerError::kNone, nullptr};
  }
  const auto result = worker_->publish(identity_.publisher_id, identity_.generation);
  clear_region();
  if (result.error == PublisherWorkerError::kNone) {
    (void)lifecycle_.complete_publish();
  } else {
    (void)lifecycle_.fail_publish();
  }
  return {LoanError::kNone, result.error, result.failure};
}

LoanOperationResult LoanState::cancel(
  const std::uintptr_t publisher_id, const bool defer_if_exported)
{
  const auto transition =
    lifecycle_.request_cancel(caller_identity(publisher_id), defer_if_exported);
  if (transition.error != LoanError::kNone) {
    return {transition.error, PublisherWorkerError::kNone, nullptr};
  }
  if (transition.action == LoanAction::kNone) {
    return {LoanError::kNone, PublisherWorkerError::kNone, nullptr};
  }
  const auto result = worker_->cancel(identity_.publisher_id, identity_.generation);
  if (result.error == PublisherWorkerError::kNone) {
    clear_region();
    (void)lifecycle_.complete_cancel();
  }
  return {LoanError::kNone, result.error, result.failure};
}

void LoanState::abandon() noexcept
{
  try {
    queue_cancel(lifecycle_.request_abandon().action);
  } catch (...) {
  }
}

LoanError LoanState::set_sequence(const std::uint64_t value) noexcept
{
  if (std::this_thread::get_id() != identity_.creator_thread) return LoanError::kWrongThread;
  if (lifecycle_.phase() != LoanPhase::kFilling) return LoanError::kInvalidPhase;
  std::lock_guard<std::mutex> lock(region_mutex_);
  if (sequence_ == nullptr) return LoanError::kInvalidPhase;
  *sequence_ = value;
  return LoanError::kNone;
}

LoanError LoanState::sequence(std::uint64_t & value) const noexcept
{
  if (std::this_thread::get_id() != identity_.creator_thread) return LoanError::kWrongThread;
  if (lifecycle_.phase() != LoanPhase::kFilling) return LoanError::kInvalidPhase;
  std::lock_guard<std::mutex> lock(region_mutex_);
  if (sequence_ == nullptr) return LoanError::kInvalidPhase;
  value = *sequence_;
  return LoanError::kNone;
}

std::size_t LoanState::active_writable_exports() const {return lifecycle_.export_count();}

LoanPhase LoanState::phase() const {return lifecycle_.phase();}

#ifdef BUILD_TESTING
PublisherWorkerResult LoanState::test_replay_publish()
{
  const auto current = phase();
  if (current != LoanPhase::kPublished && current != LoanPhase::kCancelled) {
    throw std::logic_error("only a terminal Loan token may be replayed");
  }
  return worker_->publish(identity_.publisher_id, identity_.generation);
}
#endif

LoanIdentity LoanState::caller_identity(const std::uintptr_t publisher_id) const
{
  return {publisher_id, identity_.generation, identity_.creator_thread};
}

void LoanState::clear_region() noexcept
{
  std::lock_guard<std::mutex> lock(region_mutex_);
  sequence_ = nullptr;
  data_ = nullptr;
  size_ = 0U;
}

void LoanState::queue_cancel(const LoanAction action) noexcept
{
  if (action != LoanAction::kCancel) return;
  clear_region();
  (void)worker_->cancel_async(identity_.publisher_id, identity_.generation);
}

}  // namespace rclpy_agnocast
