// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef TYPES__ROOT_HPP_
#define TYPES__ROOT_HPP_

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "rclpy_agnocast/loan.hpp"
#include "rclpy_agnocast/types/adapter.hpp"

namespace rclpy_agnocast
{
namespace types
{
enum class RootPhase
{
  kPrepare, kFilling, kCancelOnly, kCancelPending, kPublished, kCancelled,
  kPublishing, kCancelling, kFailed
};
class BufferBusy : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
using FieldPath = std::vector<std::size_t>;
struct FieldInfo
{
  const FieldOps * ops;
  std::size_t size;
  std::uint64_t generation;
};
struct RootSnapshot
{
  RootPhase phase;
  std::uint64_t generation;
  std::size_t accounted_bytes;
  std::size_t exports;
};

// Native owner only: never contains a Python object or a plugin-owned exception.
class RootOwner : public std::enable_shared_from_this<RootOwner>
{
public:
  static std::shared_ptr<RootOwner> create(
    std::shared_ptr<PublisherWorker> worker, std::uintptr_t publisher_id,
    std::uint64_t loan_generation, void * root, const TypeDescriptor & descriptor,
    std::size_t maximum);
  ~RootOwner();
  FieldInfo info(const FieldPath & path, std::uint64_t generation) const;
  ScalarValue read(const FieldPath & path, std::uint64_t generation) const;
  std::string text_bytes(const FieldPath & path, std::uint64_t generation) const;
  void write(const FieldPath & path, std::uint64_t generation, ScalarValue value);
  void resize(const FieldPath & path, std::uint64_t generation, std::size_t size);
  BufferSpan acquire_export(const FieldPath & path, std::uint64_t generation);
  void release_export() noexcept;
  void publish(std::uintptr_t publisher_id);
  void cancel(std::uintptr_t publisher_id, bool defer);
  void abandon() noexcept;
  RootSnapshot snapshot() const;
  std::uintptr_t publisher_id() const {return publisher_id_;}

private:
  RootOwner(
    std::shared_ptr<PublisherWorker>, std::uintptr_t, std::uint64_t,
    void *, const TypeDescriptor &, std::size_t);
  FieldRef resolve(const FieldPath & path, std::uint64_t generation) const;
  void check_thread() const;
  void queue_cancel(LoanAction action) noexcept;
  RootPhase phase_locked() const;
  std::size_t account() const;
  mutable std::mutex mutex_;
  std::shared_ptr<PublisherWorker> worker_;
  const std::uintptr_t publisher_id_;
  const std::uint64_t loan_generation_;
  const std::thread::id creator_;
  // Lock order is owner mutex -> lifecycle mutex; lifecycle never calls the owner.
  LoanLifecycle lifecycle_;
  void * root_;
  const TypeDescriptor descriptor_;
  const std::size_t maximum_;
  RootPhase structure_phase_{RootPhase::kPrepare};
  std::uint64_t generation_{0};
  std::size_t accounted_{0};
};
}  // namespace types
}  // namespace rclpy_agnocast
#endif  // TYPES__ROOT_HPP_
