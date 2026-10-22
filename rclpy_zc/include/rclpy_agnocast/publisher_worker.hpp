#pragma once

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>

namespace rclpy_agnocast
{

/** @brief Mutable SharedBuffer storage borrowed by a native publisher backend. */
struct BorrowedStorage
{
  std::uint64_t * sequence;
  std::uint8_t * data;
  std::size_t size;
};

/** @brief Borrowed storage plus the worker-assigned one-shot generation. */
struct BorrowedRegion
{
  std::uint64_t * sequence;
  std::uint8_t * data;
  std::size_t size;
  std::uint64_t generation;
};

/** @brief Expected command validation failures for the publisher worker. */
enum class PublisherWorkerError
{
  kNone,
  kBusy,
  kAlreadyExists,
  kNotFound,
  kStaleGeneration,
  kStopped,
  kInvalidStorage,
  kOperationFailed,
};

/** @brief Completion of a worker command without a borrowed region. */
struct PublisherWorkerResult
{
  PublisherWorkerError error;
  std::exception_ptr failure;
};

/** @brief Completion of a worker borrow command. */
struct PublisherBorrowResult
{
  PublisherWorkerError error;
  BorrowedRegion region;
  std::exception_ptr failure;
};

/** @brief Borrowed native root and its one-shot worker identity, or a backend failure. */
struct PublisherRootResult
{
  PublisherWorkerError error;
  void * root;
  std::uint64_t generation;
  std::exception_ptr failure;
};

/**
 * @brief Typed-publisher operations that must run on the publisher worker.
 *
 * Implementations own at most one unpublished native message. All methods are invoked by the
 * same worker OS thread.
 */
class PublisherBackend
{
public:
  /** @brief Destroy the backend after its active loan has reached a terminal state. */
  virtual ~PublisherBackend() = default;

  /**
   * @brief Borrow and resize one native message.
   *
   * @param[in] size Requested payload bytes.
   * @return Mutable scalar and payload storage owned by the backend.
   */
  virtual BorrowedStorage borrow(std::size_t size) = 0;

  /**
   * @brief Borrow a default-constructed root on the publisher worker.
   * @return Non-null root retained by the backend until publish or cancel.
   * @throws std::logic_error when this backend supports only SharedBuffer borrows.
   */
  virtual void * borrow_root() {throw std::logic_error("backend does not support root borrow");}

  /**
   * @brief Publish the currently borrowed native message.
   *
   * The backend must treat this as terminal even when it throws.
   */
  virtual void publish() = 0;

  /** @brief Destroy the currently borrowed unpublished message on the worker thread. */
  virtual void cancel() noexcept = 0;
};

/** @brief Worker-owned constructor for one typed native publisher backend. */
class PublisherBackendFactory
{
public:
  /** @brief Destroy unused configuration without native endpoint side effects. */
  virtual ~PublisherBackendFactory() = default;

  /**
   * @brief Create one backend on the publisher worker thread.
   * @return Non-null backend whose lifetime remains on that worker.
   */
  virtual std::unique_ptr<PublisherBackend> create() = 0;
};

/**
 * @brief Process-global fixed-slot executor for typed publisher loan operations.
 *
 * Only one backend may hold an active loan. Control and second-borrow requests fail fast while
 * the worker waits for the active loan's publish or cancel command.
 */
class PublisherWorker
{
public:
  /** @brief Start the native worker thread. */
  PublisherWorker();

  /** @brief Cancel any abandoned loan and stop the worker as a final safety net. */
  ~PublisherWorker();

  PublisherWorker(const PublisherWorker &) = delete;
  PublisherWorker & operator=(const PublisherWorker &) = delete;

  /**
   * @brief Transfer a publisher backend to worker-thread ownership.
   *
   * @param[in] publisher_id Non-zero endpoint identity unique within this worker.
   * @param[in,out] factory Non-null endpoint factory. Ownership transfers only after validation;
   * a fail-fast rejection leaves the caller's pointer intact.
   * @return Synchronous worker completion.
   */
  PublisherWorkerResult add_publisher(
    std::uintptr_t publisher_id, std::unique_ptr<PublisherBackendFactory> & factory);

  /**
   * @brief Destroy an idle publisher backend on the worker thread.
   *
   * @param[in] publisher_id Endpoint identity.
   * @return Synchronous worker completion.
   */
  PublisherWorkerResult remove_publisher(std::uintptr_t publisher_id);

  /**
   * @brief Borrow and resize shared storage on the worker thread.
   *
   * @param[in] publisher_id Endpoint identity.
   * @param[in] size Requested payload bytes.
   * @return Region and generation or a validation/backend failure.
   */
  PublisherBorrowResult borrow(std::uintptr_t publisher_id, std::size_t size);

  /**
   * @brief Borrow a generated native message using the shared single-loan slot.
   * @param[in] publisher_id Endpoint identity.
   * @return Root and generation, or a validation/backend failure with no active loan.
   */
  PublisherRootResult borrow_root(std::uintptr_t publisher_id);

  /**
   * @brief Run a synchronous structural operation without ending the active loan.
   *
   * The caller prepares the task before submission. An operation failure leaves the loan
   * active so its owner can select recovery or cancellation. No task allocation occurs here.
   *
   * @param[in] publisher_id Active endpoint identity.
   * @param[in] generation Active borrow generation.
   * @param[in] operation Non-null callback executed on the publisher worker.
   * @param[in,out] argument Caller-owned task valid through completion; may be null.
   * @return Validation or callback result, preserving any thrown exception.
   */
  PublisherWorkerResult operate(
    std::uintptr_t publisher_id, std::uint64_t generation,
    void (*operation)(void *), void * argument);

  /**
   * @brief Publish an active generation on the worker thread.
   *
   * @param[in] publisher_id Endpoint identity.
   * @param[in] generation Active borrow generation.
   * @return Synchronous worker completion.
   */
  PublisherWorkerResult publish(std::uintptr_t publisher_id, std::uint64_t generation);

  /**
   * @brief Cancel an active generation on the worker thread.
   *
   * @param[in] publisher_id Endpoint identity.
   * @param[in] generation Active borrow generation.
   * @return Synchronous worker completion.
   */
  PublisherWorkerResult cancel(std::uintptr_t publisher_id, std::uint64_t generation);

  /**
   * @brief Queue cleanup cancellation without waiting for its worker acknowledgement.
   *
   * @param[in] publisher_id Endpoint identity.
   * @param[in] generation Active borrow generation.
   * @return Validation or queueing result. This function never runs backend terminal work.
   */
  PublisherWorkerError cancel_async(
    std::uintptr_t publisher_id, std::uint64_t generation) noexcept;

  /**
   * @brief Wait until an asynchronous terminal command clears the active loan.
   *
   * @return Success once idle, or `kStopped` if the worker is already stopped.
   */
  PublisherWorkerResult wait_until_idle();

  /**
   * @brief Stop and join an idle worker.
   *
   * @return `kBusy` while a loan is active; otherwise success or `kStopped`.
   */
  PublisherWorkerResult stop();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rclpy_agnocast
