#include "rclpy_agnocast/runtime.hpp"

#include "agnocast/agnocast.hpp"
#include "rclpy_agnocast_msgs/msg/shared_buffer.hpp"
#include "publisher/observation.hpp"
#include "subscriber/native.hpp"

#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
namespace rclpy_agnocast
{
namespace
{
using Message = rclpy_agnocast_msgs::msg::SharedBuffer;
using NativeLoan = agnocast::ipc_shared_ptr<Message>;
using NativePublisher = agnocast::Publisher<Message>;
std::mutex session_mutex;
bool session_claimed{false};

class SessionClaim
{
public:
  SessionClaim()
  {
    std::lock_guard<std::mutex> lock(session_mutex);
    if (session_claimed) throw std::runtime_error("an Agnocast runtime session is already active");
    session_claimed = true;
  }
  ~SessionClaim()
  {
    std::lock_guard<std::mutex> lock(session_mutex);
    session_claimed = false;
  }
};
class NativeSession
{
public:
  explicit NativeSession(const std::string & node_name)
  {
    if (agnocast::ok()) throw std::runtime_error("Agnocast was initialized outside RuntimeCore");
    agnocast::InitOptions init_options;
    init_options.configure_logging = false;
    agnocast::init(0, nullptr, init_options);
    try {
      rclcpp::NodeOptions options;
      options.start_parameter_services(false);
      options.start_parameter_event_publisher(false);
      options.use_clock_thread(false);
      node = std::make_shared<agnocast::Node>(node_name, options);
    } catch (...) {
      agnocast::shutdown();
      throw;
    }
  }
  ~NativeSession()
  {
    node.reset();
    agnocast::shutdown();
  }
  std::shared_ptr<agnocast::Node> node;

private:
  SessionClaim claim_;
};

class SharedBufferBackend final : public PublisherBackend
{
public:
  explicit SharedBufferBackend(NativePublisher::SharedPtr publisher)
  : publisher_(std::move(publisher))
  {
#ifdef BUILD_TESTING
    observation::record(observation::Event::kCreate);
#endif
  }

  BorrowedStorage borrow(const std::size_t size) override
  {
    if (loan_) throw std::logic_error("publisher already has a native loan");
#ifdef BUILD_TESTING
    observation::record(observation::Event::kBorrow);
#endif
    NativeLoan candidate;
    try {
      candidate = publisher_->borrow_loaned_message();
    } catch (...) {
#ifdef BUILD_TESTING
      observation::record(observation::Event::kBorrowFailed);
#endif
      throw;
    }
#ifdef BUILD_TESTING
    observation::record(
      observation::Event::kResize, reinterpret_cast<std::uintptr_t>(candidate.get()));
#endif
    try {
      candidate->data.resize(size);
    } catch (...) {
      candidate.reset();
#ifdef BUILD_TESTING
      observation::record(observation::Event::kResizeFailed);
#endif
      throw;
    }
    auto storage = BorrowedStorage{&candidate->sequence, candidate->data.data(), size};
    loan_ = std::move(candidate);
#ifdef BUILD_TESTING
    observation::record(
      observation::Event::kBorrowed, reinterpret_cast<std::uintptr_t>(loan_.get()),
      reinterpret_cast<std::uintptr_t>(storage.data));
#endif
    return storage;
  }

  void publish() override
  {
    if (!loan_) throw std::logic_error("publisher has no native loan");
#ifdef BUILD_TESTING
    observation::record(observation::Event::kPublish);
#endif
    try {
      publisher_->publish(std::move(loan_));
    } catch (...) {
      loan_.reset();
      throw;
    }
#ifdef BUILD_TESTING
    observation::record(observation::Event::kPublished);
#endif
  }

  void cancel() noexcept override
  {
    if (!loan_) return;
#ifdef BUILD_TESTING
    observation::record(observation::Event::kCancel);
#endif
    loan_.reset();
#ifdef BUILD_TESTING
    observation::record(observation::Event::kCancelled);
#endif
  }

private:
  NativePublisher::SharedPtr publisher_;
  NativeLoan loan_;
};

class SharedBufferFactory final : public PublisherBackendFactory
{
public:
  SharedBufferFactory(
    std::shared_ptr<agnocast::Node> node, std::string topic, const std::size_t depth)
  : node_(std::move(node)), topic_(std::move(topic)), depth_(depth) {}

  std::unique_ptr<PublisherBackend> create() override
  {
    auto publisher = node_->create_publisher<Message>(
      topic_, depth_, agnocast::PublisherOptions{});
    return std::make_unique<SharedBufferBackend>(std::move(publisher));
  }

private:
  std::shared_ptr<agnocast::Node> node_;
  std::string topic_;
  std::size_t depth_;
};

[[noreturn]] void throw_worker_failure(
  const char * operation, const PublisherWorkerResult & result)
{
  if (result.failure) std::rethrow_exception(result.failure);
  throw std::runtime_error(
          std::string(operation) + " failed with worker error " +
          std::to_string(static_cast<int>(result.error)));
}

}  // namespace
class RuntimeCore::Impl
{
public:
  Impl(std::shared_ptr<NativeSession> native_session, std::shared_ptr<PublisherWorker> native_worker)
  : session(std::move(native_session)), worker(std::move(native_worker)) {}

  mutable std::mutex mutex;
  RuntimeState state{RuntimeState::kOpen};
  std::shared_ptr<NativeSession> session;
  std::shared_ptr<PublisherWorker> worker;
  std::shared_ptr<SubscriberRuntime> subscribers;
  std::set<std::uintptr_t> publisher_ids;
  std::uintptr_t next_publisher_id{1U};
};

RuntimeCore::RuntimeCore(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

std::shared_ptr<RuntimeCore> RuntimeCore::create(const std::string & node_name)
{
  if (node_name.empty()) throw std::invalid_argument("node_name must not be empty");
  auto session = std::make_shared<NativeSession>(node_name);
  // LoanState retains this control block, so its deleter keeps the native session alive.
  auto worker = std::shared_ptr<PublisherWorker>(
    new PublisherWorker(), [session](PublisherWorker * value) {delete value;});
  auto runtime = std::shared_ptr<RuntimeCore>(
    new RuntimeCore(std::make_unique<Impl>(session, std::move(worker))));
  return runtime;
}

RuntimeCore::~RuntimeCore() {(void)close();}

std::shared_ptr<PublisherHandle> RuntimeCore::create_publisher(
  const std::string & topic, const std::size_t depth, const std::size_t max_payload_bytes)
{
  if (topic.empty()) throw std::invalid_argument("topic must not be empty");
  if (depth == 0U) throw std::invalid_argument("depth must be positive");
  if (max_payload_bytes == 0U) {
    throw std::invalid_argument("max_payload_bytes must be positive");
  }

  std::unique_ptr<PublisherHandle> handle;
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if (impl_->state != RuntimeState::kOpen || !impl_->worker || !impl_->session) {
    throw std::runtime_error("runtime is not open");
  }
  if (impl_->next_publisher_id == 0U) throw std::overflow_error("publisher identity exhausted");
  const auto publisher_id = impl_->next_publisher_id++;
  handle.reset(new PublisherHandle(shared_from_this(), publisher_id, max_payload_bytes));
  impl_->publisher_ids.insert(publisher_id);
  auto factory = std::unique_ptr<PublisherBackendFactory>(new SharedBufferFactory(
      impl_->session->node, topic, depth));
  const auto result = impl_->worker->add_publisher(publisher_id, factory);
  if (result.error != PublisherWorkerError::kNone) {
    impl_->publisher_ids.erase(publisher_id);
    throw_worker_failure("publisher creation", result);
  }
  lock.unlock();
  return std::shared_ptr<PublisherHandle>(std::move(handle));
}

std::shared_ptr<LoanState> RuntimeCore::borrow(
  const std::uintptr_t publisher_id, const std::size_t size)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->state != RuntimeState::kOpen || !impl_->worker) {
    throw std::runtime_error("runtime is not open");
  }
  if (impl_->publisher_ids.count(publisher_id) == 0U) {
    throw std::runtime_error("publisher is closed");
  }
  const auto result = impl_->worker->borrow(publisher_id, size);
  if (result.error != PublisherWorkerError::kNone) {
    throw_worker_failure("publisher borrow", {result.error, result.failure});
  }
  try {
    const LoanIdentity identity{publisher_id, result.region.generation, std::this_thread::get_id()};
    return std::make_shared<LoanState>(impl_->worker, identity, result.region);
  } catch (...) {
    const auto failure = std::current_exception();
    const auto cancel = impl_->worker->cancel(publisher_id, result.region.generation);
    if (cancel.error != PublisherWorkerError::kNone) {
      throw_worker_failure("loan allocation rollback", cancel);
    }
    std::rethrow_exception(failure);
  }
}

std::shared_ptr<SubscriptionHandle> RuntimeCore::create_subscription(
  const std::string & topic, const std::size_t depth,
  const std::size_t max_outstanding_leases, const std::size_t max_payload_bytes)
{
  if (topic.empty()) throw std::invalid_argument("topic must not be empty");
  if (depth == 0 || max_outstanding_leases == 0 || max_payload_bytes == 0) {
    throw std::invalid_argument("subscriber depth and limits must be positive");
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->state != RuntimeState::kOpen) throw std::runtime_error("runtime is not open");
  if (!impl_->subscribers) {
    impl_->subscribers = std::make_shared<SubscriberRuntime>(impl_->session->node, impl_->session);
  }
  return impl_->subscribers->create_subscription(
    topic, depth, max_outstanding_leases, max_payload_bytes);
}

PublisherWorkerResult RuntimeCore::remove_publisher(const std::uintptr_t publisher_id) noexcept
{
  try {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->publisher_ids.count(publisher_id) == 0U) return {PublisherWorkerError::kNone, nullptr};
    if (!impl_->worker) return {PublisherWorkerError::kStopped, nullptr};
    const auto result = impl_->worker->remove_publisher(publisher_id);
    if (
      result.error == PublisherWorkerError::kNone ||
      result.error == PublisherWorkerError::kNotFound)
    {
      impl_->publisher_ids.erase(publisher_id);
      return {PublisherWorkerError::kNone, nullptr};
    }
    return result;
  } catch (...) {
    return {PublisherWorkerError::kOperationFailed, std::current_exception()};
  }
}

PublisherWorkerResult RuntimeCore::close() noexcept
{
  try {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    if (impl_->state == RuntimeState::kClosed) return {PublisherWorkerError::kNone, nullptr};
    impl_->state = RuntimeState::kQuiescing;
    if (impl_->subscribers) impl_->subscribers->quiesce();
    const auto result = impl_->worker->stop();
    if (
      result.error != PublisherWorkerError::kNone &&
      result.error != PublisherWorkerError::kStopped)
    {
      return result;
    }
    impl_->publisher_ids.clear();
    // A Lease busy result occurs after publisher teardown and subscriber executor join.
    if (impl_->subscribers && !impl_->subscribers->close()) {
      return {PublisherWorkerError::kBusy,
        std::make_exception_ptr(std::runtime_error("runtime close: active Lease or view"))};
    }
    impl_->state = RuntimeState::kClosed;
    auto worker = std::move(impl_->worker);
    auto subscribers = std::move(impl_->subscribers);
    auto session = std::move(impl_->session);
    lock.unlock();
    subscribers.reset();
    worker.reset();
    session.reset();
    return {PublisherWorkerError::kNone, nullptr};
  } catch (...) {
    return {PublisherWorkerError::kOperationFailed, std::current_exception()};
  }
}

RuntimeState RuntimeCore::state() const noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->state;
}

PublisherHandle::~PublisherHandle() {(void)close();}

std::shared_ptr<LoanState> PublisherHandle::borrow(const std::size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_ || closed_) throw std::runtime_error("publisher is closing or closed");
  if (size > max_payload_bytes_) throw std::invalid_argument("payload exceeds configured limit");
  return runtime_->borrow(publisher_id_, size);
}

PublisherWorkerResult PublisherHandle::close() noexcept
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) return {PublisherWorkerError::kNone, nullptr};
    accepting_ = false;
    const auto result = runtime_->remove_publisher(publisher_id_);
    if (result.error == PublisherWorkerError::kNone) closed_ = true;
    return result;
  } catch (...) {
    return {PublisherWorkerError::kOperationFailed, std::current_exception()};
  }
}

}  // namespace rclpy_agnocast
