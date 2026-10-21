#include "rclpy_agnocast/publisher_worker.hpp"
#include "publisher/observation.hpp"

#include <condition_variable>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
namespace rclpy_agnocast
{
namespace
{
enum class Command {kAdd, kRemove, kBorrow, kPublish, kCancel, kStop};
struct CommandSlot
{
  Command command{Command::kStop};
  std::uintptr_t publisher_id{0U};
  std::size_t size{0U};
  std::uint64_t generation{0U};
  std::unique_ptr<PublisherBackendFactory> factory;
};
struct Completion
{
  PublisherWorkerError error{PublisherWorkerError::kNone};
  BorrowedRegion region{nullptr, nullptr, 0U, 0U};
  std::exception_ptr failure;
};
}  // namespace
class PublisherWorker::Impl
{
public:
  Impl() : worker_([this]() {run();}) {}

  ~Impl()
  {
    bool active = false;
    std::uintptr_t publisher_id = 0U;
    std::uint64_t generation = 0U;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active = active_;
      publisher_id = active_publisher_id_;
      generation = active_generation_;
    }
    if (active) {
      const auto result = cancel(publisher_id, generation);
      if (result.error == PublisherWorkerError::kBusy) (void)wait_until_idle();
    }
    if (stop().error == PublisherWorkerError::kBusy) {
      (void)wait_until_idle();
      (void)stop();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  PublisherWorkerResult add_publisher(
    const std::uintptr_t publisher_id, std::unique_ptr<PublisherBackendFactory> & factory)
  {
    std::lock_guard<std::mutex> api_lock(api_mutex_);
    std::unique_lock<std::mutex> state_lock(state_mutex_);
    if (stopped_) return error_result(PublisherWorkerError::kStopped);
    if (active_) return error_result(PublisherWorkerError::kBusy);
    CommandSlot slot{Command::kAdd, publisher_id, 0U, 0U, std::move(factory)};
    return command_result(submit(state_lock, std::move(slot)));
  }
  PublisherWorkerResult remove_publisher(const std::uintptr_t publisher_id)
  {
    std::lock_guard<std::mutex> api_lock(api_mutex_);
    std::unique_lock<std::mutex> state_lock(state_mutex_);
    if (stopped_) return error_result(PublisherWorkerError::kStopped);
    if (active_) return error_result(PublisherWorkerError::kBusy);
    CommandSlot slot{Command::kRemove, publisher_id, 0U, 0U, nullptr};
    return command_result(submit(state_lock, std::move(slot)));
  }
  PublisherBorrowResult borrow(const std::uintptr_t publisher_id, const std::size_t size)
  {
    std::lock_guard<std::mutex> api_lock(api_mutex_);
    std::unique_lock<std::mutex> state_lock(state_mutex_);
    if (stopped_) return borrow_error(PublisherWorkerError::kStopped);
    if (active_) return borrow_error(PublisherWorkerError::kBusy);
    CommandSlot slot{Command::kBorrow, publisher_id, size, 0U, nullptr};
    const auto completion = submit(state_lock, std::move(slot));
    return {completion.error, completion.region, completion.failure};
  }
  PublisherWorkerResult publish(const std::uintptr_t id, const std::uint64_t generation)
  {return terminal_command(Command::kPublish, id, generation);}
  PublisherWorkerResult cancel(const std::uintptr_t id, const std::uint64_t generation)
  {return terminal_command(Command::kCancel, id, generation);}
  PublisherWorkerError cancel_async(
    const std::uintptr_t publisher_id, const std::uint64_t generation) noexcept
  {
    try {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (stopped_) return PublisherWorkerError::kStopped;
      if (!active_ || active_publisher_id_ != publisher_id) return PublisherWorkerError::kNotFound;
      if (active_generation_ != generation) return PublisherWorkerError::kStaleGeneration;
      if (command_ready_) return PublisherWorkerError::kBusy;
      command_slot_ = {Command::kCancel, publisher_id, 0U, generation, nullptr};
      completion_ready_ = false;
      command_ready_ = true;
#ifdef BUILD_TESTING
      observation::enqueued(true);
#endif
      command_cv_.notify_one();
      return PublisherWorkerError::kNone;
    } catch (...) {
      return PublisherWorkerError::kOperationFailed;
    }
  }
  PublisherWorkerResult wait_until_idle()
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    completion_cv_.wait(lock, [this]() {return !active_ || stopped_;});
    return error_result(stopped_ ? PublisherWorkerError::kStopped : PublisherWorkerError::kNone);
  }
  PublisherWorkerResult stop()
  {
    std::lock_guard<std::mutex> api_lock(api_mutex_);
    std::unique_lock<std::mutex> state_lock(state_mutex_);
    if (stopped_) return error_result(PublisherWorkerError::kStopped);
    if (active_) return error_result(PublisherWorkerError::kBusy);
    CommandSlot slot{Command::kStop, 0U, 0U, 0U, nullptr};
    const auto result = command_result(submit(state_lock, std::move(slot)));
    state_lock.unlock();
    if (worker_.joinable()) {
      worker_.join();
    }
    return result;
  }
private:
  static PublisherWorkerResult error_result(PublisherWorkerError error) {return {error, nullptr};}
  static PublisherBorrowResult borrow_error(PublisherWorkerError error)
  {return {error, {nullptr, nullptr, 0U, 0U}, nullptr};}
  static PublisherWorkerResult command_result(const Completion & result)
  {return {result.error, result.failure};}

  Completion submit(std::unique_lock<std::mutex> & state_lock, CommandSlot slot)
  {
    command_slot_ = std::move(slot);
    completion_ = Completion{};
    command_ready_ = true;
    completion_ready_ = false;
    command_cv_.notify_one();
    completion_cv_.wait(state_lock, [this]() {return completion_ready_;});
    return completion_;
  }

  PublisherWorkerResult terminal_command(
    const Command command, const std::uintptr_t publisher_id, const std::uint64_t generation)
  {
    std::lock_guard<std::mutex> api_lock(api_mutex_);
    std::unique_lock<std::mutex> state_lock(state_mutex_);
    if (stopped_) return error_result(PublisherWorkerError::kStopped);
    if (!active_ || active_publisher_id_ != publisher_id) {
      return error_result(PublisherWorkerError::kNotFound);
    }
    if (active_generation_ != generation) return error_result(PublisherWorkerError::kStaleGeneration);
    if (command_ready_) return error_result(PublisherWorkerError::kBusy);
    CommandSlot slot{command, publisher_id, 0U, generation, nullptr};
#ifdef BUILD_TESTING
    observation::enqueued(false);
#endif
    return command_result(submit(state_lock, std::move(slot)));
  }

  Completion execute(CommandSlot & slot)
  {
    Completion completion;
    try {
      const auto backend = publishers_.find(slot.publisher_id);
      switch (slot.command) {
        case Command::kAdd:
          if (!slot.factory) return {PublisherWorkerError::kOperationFailed, {}, nullptr};
          if (publishers_.count(slot.publisher_id) != 0U) {
            return {PublisherWorkerError::kAlreadyExists, {}, nullptr};
          }
          {
            auto created = slot.factory->create();
            if (!created) return {PublisherWorkerError::kOperationFailed, {}, nullptr};
            publishers_.emplace(slot.publisher_id, std::move(created));
          }
          break;
        case Command::kRemove:
          if (backend == publishers_.end()) {
            return {PublisherWorkerError::kNotFound, {}, nullptr};
          }
          publishers_.erase(backend);
          break;
        case Command::kBorrow:
          if (backend == publishers_.end()) {
            return {PublisherWorkerError::kNotFound, {}, nullptr};
          }
          {
            BorrowedStorage storage{};
            try {
              storage = backend->second->borrow(slot.size);
            } catch (...) {
              backend->second->cancel();
              throw;
            }
            if (
              storage.sequence == nullptr || storage.size != slot.size ||
              (storage.size != 0U && storage.data == nullptr))
            {
              backend->second->cancel();
              return {PublisherWorkerError::kInvalidStorage, {}, nullptr};
            }
            completion.region = {
              storage.sequence, storage.data, storage.size, next_generation_++};
          }
          break;
        case Command::kPublish:
          backend->second->publish();
          break;
        case Command::kCancel:
          backend->second->cancel();
          break;
        case Command::kStop:
          publishers_.clear();
          break;
      }
    } catch (...) {
      completion.error = PublisherWorkerError::kOperationFailed;
      completion.failure = std::current_exception();
    }
    return completion;
  }

  void run()
  {
    while (true) {
      CommandSlot slot;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        command_cv_.wait(lock, [this]() {return command_ready_;});
        slot = std::move(command_slot_);
      }

#ifdef BUILD_TESTING
      observation::filling(false);
#endif
      auto completion = execute(slot);
#ifdef BUILD_TESTING
      if (
        slot.command == Command::kBorrow &&
        completion.error == PublisherWorkerError::kNone)
      {
        observation::filling(true);
      }
#endif
      const bool should_stop = slot.command == Command::kStop;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (completion.error == PublisherWorkerError::kNone) {
          if (slot.command == Command::kBorrow) {
            active_ = true;
            active_publisher_id_ = slot.publisher_id;
            active_generation_ = completion.region.generation;
          } else if (slot.command == Command::kCancel) {
            active_ = false;
          } else if (slot.command == Command::kStop) {
            stopped_ = true;
          }
        }
        if (slot.command == Command::kPublish) {
          active_ = false;
        }
#ifdef BUILD_TESTING
        if (
          completion.error == PublisherWorkerError::kNone &&
          (slot.command == Command::kPublish || slot.command == Command::kCancel))
        {
          observation::completed(slot.command == Command::kPublish);
        }
#endif
        completion_ = std::move(completion);
        command_ready_ = false;
        completion_ready_ = true;
      }
      completion_cv_.notify_all();
      if (should_stop) return;
    }
  }

  std::mutex api_mutex_;
  std::mutex state_mutex_;
  std::condition_variable command_cv_;
  std::condition_variable completion_cv_;
  CommandSlot command_slot_;
  Completion completion_;
  bool command_ready_{false};
  bool completion_ready_{false};
  bool active_{false};
  bool stopped_{false};
  std::uintptr_t active_publisher_id_{0U};
  std::uint64_t active_generation_{0U};
  std::uint64_t next_generation_{1U};
  std::map<std::uintptr_t, std::unique_ptr<PublisherBackend>> publishers_;
  std::thread worker_;
};
PublisherWorker::PublisherWorker() : impl_(std::make_unique<Impl>()) {}
PublisherWorker::~PublisherWorker() = default;

PublisherWorkerResult PublisherWorker::add_publisher(
  const std::uintptr_t id, std::unique_ptr<PublisherBackendFactory> & factory)
{return impl_->add_publisher(id, factory);}
PublisherWorkerResult PublisherWorker::remove_publisher(const std::uintptr_t publisher_id)
{return impl_->remove_publisher(publisher_id);}
PublisherBorrowResult PublisherWorker::borrow(
  const std::uintptr_t publisher_id, const std::size_t size)
{return impl_->borrow(publisher_id, size);}
PublisherWorkerResult PublisherWorker::publish(
  const std::uintptr_t publisher_id, const std::uint64_t generation)
{return impl_->publish(publisher_id, generation);}
PublisherWorkerResult PublisherWorker::cancel(
  const std::uintptr_t publisher_id, const std::uint64_t generation)
{return impl_->cancel(publisher_id, generation);}
PublisherWorkerError PublisherWorker::cancel_async(
  const std::uintptr_t publisher_id, const std::uint64_t generation) noexcept
{return impl_->cancel_async(publisher_id, generation);}
PublisherWorkerResult PublisherWorker::wait_until_idle()
{return impl_->wait_until_idle();}
PublisherWorkerResult PublisherWorker::stop() {return impl_->stop();}
}  // namespace rclpy_agnocast
