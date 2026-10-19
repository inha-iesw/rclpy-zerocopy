#pragma once

#include "rclpy_agnocast/publisher_worker.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

namespace rclpy_agnocast
{

/** @brief Stable identity of the publisher and loan generation. */
struct LoanIdentity
{
  std::uintptr_t publisher_id;
  std::uint64_t generation;
  std::thread::id creator_thread;
};

/** @brief Observable phase of one publisher loan. */
enum class LoanPhase
{
  kFilling,
  kCancelPending,
  kPublishing,
  kCancelling,
  kPublished,
  kCancelled,
  kFailed,
};

/** @brief Validation error returned before entering a native Agnocast terminal API. */
enum class LoanError
{
  kNone,
  kWrongPublisher,
  kStaleGeneration,
  kWrongThread,
  kInvalidPhase,
  kActiveExports,
  kNoActiveExport,
};

/** @brief Native worker action selected by a validated state transition. */
enum class LoanAction
{
  kNone,
  kPublish,
  kCancel,
};

/** @brief Result of a Loan lifecycle operation. */
struct LoanTransition
{
  LoanError error;
  LoanAction action;
};

/**
 * @brief Serialize one Loan's buffer exports and terminal transition.
 *
 * The lifecycle never owns the shared payload. It validates public operations before the
 * publisher worker touches Agnocast and permits a foreign thread to release the final Python
 * buffer export safely.
 */
class LoanLifecycle
{
public:
  /**
   * @brief Create a filling Loan lifecycle.
   *
   * @param[in] identity Publisher, generation, and creator-thread identity.
   */
  explicit LoanLifecycle(LoanIdentity identity);

  /**
   * @brief Register one writable Python buffer export.
   *
   * @return Validation result; this operation never requests a worker action.
   */
  LoanTransition acquire_export();

  /**
   * @brief Release one writable Python buffer export from any Python thread.
   *
   * @return `kCancel` when this was the last export of a cancel-pending Loan.
   */
  LoanTransition release_export();

  /**
   * @brief Validate and begin a one-shot publish transition.
   *
   * @param[in] identity Identity supplied by the calling Publisher and Python thread.
   * @return `kPublish` on success or a validation error with no worker action.
   */
  LoanTransition request_publish(const LoanIdentity & identity);

  /**
   * @brief Validate and begin or defer a cancel transition.
   *
   * @param[in] identity Identity supplied by the calling Publisher and Python thread.
   * @param[in] defer_if_exported Preserve an active alias and cancel after its last release.
   * @return `kCancel` when cancellation may run immediately, otherwise no worker action.
   */
  LoanTransition request_cancel(const LoanIdentity & identity, bool defer_if_exported);

  /**
   * @brief Begin cleanup from a finalizer running on any Python thread.
   *
   * @return `kCancel` when cleanup may run immediately, otherwise a deferred transition.
   */
  LoanTransition request_abandon();

  /**
   * @brief Mark the worker publish action complete.
   *
   * @return `kNone` on success or `kInvalidPhase` if publish was not requested.
   */
  LoanError complete_publish();

  /**
   * @brief Record that the worker consumed the loan but failed to publish it.
   *
   * @return `kNone` on success or `kInvalidPhase` if publish was not requested.
   */
  LoanError fail_publish();

  /**
   * @brief Mark the worker cancel action complete.
   *
   * @return `kNone` on success or `kInvalidPhase` if cancel was not requested.
   */
  LoanError complete_cancel();

  /**
   * @brief Return the current phase under the lifecycle lock.
   *
   * @return Current Loan phase.
   */
  LoanPhase phase() const;

  /**
   * @brief Return the number of active Python buffer exports.
   *
   * @return Active export count.
   */
  std::size_t export_count() const;

private:
  LoanError validate_identity(const LoanIdentity & identity) const;

  const LoanIdentity identity_;
  mutable std::mutex mutex_;
  LoanPhase phase_{LoanPhase::kFilling};
  std::size_t export_count_{0};
};

/** @brief Result of acquiring one Python-visible writable export token. */
enum class WritableExportAcquireResult : std::uint8_t
{
  kOk,
  kWrongThread,
  kNotFilling,
};

/** @brief Stable byte span captured while one writable export token is active. */
struct WritableBufferSpan
{
  std::uint8_t * data;
  std::size_t size;
};

/** @brief Combined validation and native-worker result for a terminal Loan operation. */
struct LoanOperationResult
{
  LoanError loan_error;
  PublisherWorkerError worker_error;
  std::exception_ptr failure;
};

/**
 * @brief Own one borrowed region's Python-facing state and worker lifetime.
 *
 * The native backend remains owned by PublisherWorker. This state only exposes the borrowed
 * scalar/byte span while filling and queues cleanup cancellation from arbitrary release threads.
 */
class LoanState
{
public:
  /**
   * @brief Adopt a worker-borrowed region.
   * @param[in] worker Strong worker ownership retained through all export tokens.
   * @param[in] identity Publisher, generation, and creator-thread identity.
   * @param[in] region Borrowed pointers valid until a terminal worker action.
   * @throws std::invalid_argument if ownership or region invariants are invalid.
   */
  LoanState(
    std::shared_ptr<PublisherWorker> worker, LoanIdentity identity, BorrowedRegion region);

  /** @brief Queue cancellation for an abandoned non-terminal loan. */
  ~LoanState();

  LoanState(const LoanState &) = delete;
  LoanState & operator=(const LoanState &) = delete;

  /**
   * @brief Snapshot the writable payload and register one export token.
   * @param[out] span Borrowed bytes kept valid by the registered token.
   * @return Thread/phase validation result.
   */
  WritableExportAcquireResult acquire_writable_export(WritableBufferSpan & span) noexcept;

  /** @brief Release one token and queue pending cancellation without waiting for ACK. */
  void release_writable_export() noexcept;

  /** @brief Publish after validating publisher identity, creator thread, and zero exports. */
  LoanOperationResult publish(std::uintptr_t publisher_id);

  /** @brief Cancel now, or defer cleanup while exports remain when requested. */
  LoanOperationResult cancel(std::uintptr_t publisher_id, bool defer_if_exported);

  /** @brief Request finalizer cleanup from any thread without throwing or waiting for ACK. */
  void abandon() noexcept;

  /** @brief Set the scalar sequence while filling on the creator thread. */
  LoanError set_sequence(std::uint64_t value) noexcept;

  /** @brief Read the scalar sequence while filling on the creator thread. */
  LoanError sequence(std::uint64_t & value) const noexcept;

  /** @brief Return active export-token count. */
  std::size_t active_writable_exports() const;

  /** @brief Return current lifecycle phase. */
  LoanPhase phase() const;

private:
  LoanIdentity caller_identity(std::uintptr_t publisher_id) const;
  void clear_region() noexcept;
  void queue_cancel(LoanAction action) noexcept;

  std::shared_ptr<PublisherWorker> worker_;
  const LoanIdentity identity_;
  LoanLifecycle lifecycle_;
  mutable std::mutex region_mutex_;
  std::uint64_t * sequence_;
  std::uint8_t * data_;
  std::size_t size_;
};

}  // namespace rclpy_agnocast
