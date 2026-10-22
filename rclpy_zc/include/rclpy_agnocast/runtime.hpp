#pragma once

#include "rclpy_agnocast/loan.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace rclpy_agnocast
{
namespace types
{
class RootOwner;
struct TypeDescriptor;
}  // namespace types

/** @brief Observable process-runtime lifecycle. */
enum class RuntimeState
{
  kOpen,
  kQuiescing,
  kClosed,
};

class PublisherHandle;
class SubscriptionHandle;

/**
 * @brief Process-wide owner of the Agnocast node and publisher/subscriber workers.
 *
 * Use create() rather than constructing this class directly. A second runtime is rejected until
 * the first runtime and every Loan/Lease alias have released their native resources.
 */
class RuntimeCore : public std::enable_shared_from_this<RuntimeCore>
{
public:
  /**
   * @brief Initialize Agnocast and its adapter node.
   *
   * @param[in] node_name Non-empty native node name.
   * @return The sole active process runtime.
   * @throws std::invalid_argument if `node_name` is empty.
   * @throws std::runtime_error if another native runtime session is still alive.
   * @throws Any exception raised by Agnocast initialization or node construction.
   */
  static std::shared_ptr<RuntimeCore> create(const std::string & node_name);

  /** @brief Attempt ordered native cleanup without throwing. */
  ~RuntimeCore();

  RuntimeCore(const RuntimeCore &) = delete;
  RuntimeCore & operator=(const RuntimeCore &) = delete;

  /**
   * @brief Create a typed SharedBuffer publisher on the native worker thread.
   *
   * @param[in] topic Non-empty ROS topic name.
   * @param[in] depth Positive KeepLast history depth.
   * @param[in] max_payload_bytes Positive per-message admission limit.
   * @return A publisher handle retaining this runtime.
   * @throws std::invalid_argument for an empty topic or zero limit/depth.
   * @throws std::runtime_error if not open or the worker reports failure without an exception.
   * @throws Any exception reported by native endpoint construction.
   */
  std::shared_ptr<PublisherHandle> create_publisher(
    const std::string & topic, std::size_t depth, std::size_t max_payload_bytes);

  /**
   * @brief Create a registered message publisher on the existing native worker.
   * @param[in] ros_type Fully qualified registered ROS message identity.
   * @param[in] topic Non-empty resolved ROS topic name.
   * @param[in] depth Positive KeepLast history depth.
   * @param[in] max_storage_bytes Positive accounted-storage limit per borrowed root.
   * @return Publisher retaining this runtime and its immutable type descriptor.
   * @throws std::invalid_argument for an empty topic or zero depth/limit.
   * @throws std::out_of_range when no adapter registered the requested type.
   * @throws std::runtime_error when closing, busy, or native creation fails.
   */
  std::shared_ptr<PublisherHandle> create_typed_publisher(
    const std::string & ros_type, const std::string & topic,
    std::size_t depth, std::size_t max_storage_bytes);

  /**
   * @brief Create a typed subscriber with bounded retained count and payload bytes.
   *
   * @param[in] topic Non-empty ROS topic name.
   * @param[in] depth Positive native KeepLast history depth.
   * @param[in] max_outstanding_leases Positive count limit, including queued and exported storage.
   * @param[in] max_payload_bytes Positive aggregate retained payload limit.
   * @return A native handle; rclpy callback dispatch is integrated separately.
   * @throws std::invalid_argument for an empty topic or zero limit/depth.
   * @throws std::runtime_error if the runtime is not open.
   * @throws Any exception raised by native endpoint creation.
   */
  std::shared_ptr<SubscriptionHandle> create_subscription(
    const std::string & topic, std::size_t depth, std::size_t max_outstanding_leases,
    std::size_t max_payload_bytes);

  /**
   * @brief Stop native workers while retaining subscriber queues, endpoints and session.
   *
   * @return Retryable `kBusy` while a Loan/export is active, otherwise worker result.
   * @post On success, Python may join notifiers, drain queues and remove guards before close().
   */
  PublisherWorkerResult prepare_close() noexcept;

  /**
   * @brief Quiesce endpoints and join native workers before destroying subscriber resources.
   *
   * @return `kBusy` without destructive cleanup while a Loan remains active; otherwise the first
   * worker failure. Live subscriber Leases/views also return a retryable busy result after
   * stopping the executor and draining its queue. Success and repeated close return `kNone`.
   */
  PublisherWorkerResult close() noexcept;

  /**
   * @brief Return the current lifecycle under the runtime lock.
   * @return Open, quiescing, or closed.
   */
  RuntimeState state() const noexcept;

private:
  class Impl;

  explicit RuntimeCore(std::unique_ptr<Impl> impl);
  std::shared_ptr<LoanState> borrow(std::uintptr_t publisher_id, std::size_t size);
  std::shared_ptr<types::RootOwner> borrow_message(
    std::uintptr_t publisher_id, std::shared_ptr<const types::TypeDescriptor> descriptor,
    std::size_t max_storage_bytes);
  PublisherWorkerResult remove_publisher(std::uintptr_t publisher_id) noexcept;

  std::unique_ptr<Impl> impl_;

  friend class PublisherHandle;
};

/** @brief One SharedBuffer or generated message endpoint owned by RuntimeCore's worker. */
class PublisherHandle
{
public:
  /**
   * @brief Attempt best-effort endpoint cleanup without throwing.
   *
   * Call close() explicitly for endpoint-level cleanup. If another publisher is active, this
   * destructor leaves the endpoint in RuntimeCore's registry for final runtime cleanup.
   */
  ~PublisherHandle();

  PublisherHandle(const PublisherHandle &) = delete;
  PublisherHandle & operator=(const PublisherHandle &) = delete;

  /**
   * @brief Borrow exactly `size` writable payload bytes.
   *
   * @param[in] size Requested bytes, including zero.
   * @return Shared Loan state retaining worker, node, and Agnocast session resources.
   * @throws std::invalid_argument when `size` exceeds the configured positive limit.
   * @throws std::runtime_error when closed, busy, or the worker has no stored native exception.
   * @throws Any exception reported by native borrow or Loan-state allocation.
   */
  std::shared_ptr<LoanState> borrow(std::size_t size);

  /**
   * @brief Borrow one default-constructed generated root on the native worker.
   * @return Root owner retaining the worker/session through all field and buffer aliases.
   * @throws std::logic_error when this is a legacy SharedBuffer publisher.
   * @throws std::runtime_error when the endpoint is closing or another loan is active.
   * @throws Any exception from native borrow or root accounting; the loan is rolled back.
   */
  std::shared_ptr<types::RootOwner> borrow_message();

  /**
   * @brief Publish a Loan using this endpoint identity.
   * @param[in] loan Non-null Loan state.
   * @return Loan validation and worker completion. Wrong-publisher rejection occurs in LoanState.
   * @throws std::invalid_argument when `loan` is null.
   */
  LoanOperationResult publish(const std::shared_ptr<LoanState> & loan)
  {
    if (!loan) throw std::invalid_argument("loan must not be null");
    return loan->publish(publisher_id_);
  }

  /**
   * @brief Cancel a Loan using this endpoint identity.
   * @param[in] loan Non-null Loan state.
   * @param[in] defer_if_exported Queue cleanup after the last export instead of rejecting it.
   * @return Loan validation and worker completion. Wrong-publisher rejection occurs in LoanState.
   * @throws std::invalid_argument when `loan` is null.
   */
  LoanOperationResult cancel(
    const std::shared_ptr<LoanState> & loan, const bool defer_if_exported)
  {
    if (!loan) throw std::invalid_argument("loan must not be null");
    return loan->cancel(publisher_id_, defer_if_exported);
  }

  /**
   * @brief Stop new borrows and destroy an idle endpoint.
   * @return `kBusy` while any runtime publisher Loan is active; the call may then be retried.
   */
  PublisherWorkerResult close() noexcept;

  /** @brief Return this endpoint's non-zero process-local identity. @return Endpoint identity. */
  std::uintptr_t publisher_id() const noexcept {return publisher_id_;}

  /** @brief Return the configured byte limit. @return Positive per-message payload limit. */
  std::size_t max_payload_bytes() const noexcept {return max_payload_bytes_;}

  /** @brief Return whether endpoint destruction completed. @return True after successful close. */
  bool is_closed() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

private:
  PublisherHandle(
    std::shared_ptr<RuntimeCore> runtime, std::uintptr_t publisher_id,
    std::size_t max_payload_bytes,
    std::shared_ptr<const types::TypeDescriptor> typed_descriptor = nullptr)
  : runtime_(std::move(runtime)),
    publisher_id_(publisher_id),
    max_payload_bytes_(max_payload_bytes),
    typed_descriptor_(std::move(typed_descriptor)) {}

  mutable std::mutex mutex_;
  std::shared_ptr<RuntimeCore> runtime_;
  const std::uintptr_t publisher_id_;
  const std::size_t max_payload_bytes_;
  std::shared_ptr<const types::TypeDescriptor> typed_descriptor_;
  bool accepting_{true};
  bool closed_{false};

  friend class RuntimeCore;
};

}  // namespace rclpy_agnocast
