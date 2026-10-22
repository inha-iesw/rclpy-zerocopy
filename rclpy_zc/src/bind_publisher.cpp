#include "rclpy_agnocast/bindings.hpp"

#include "rclpy_agnocast/buffer_owner.hpp"
#include "rclpy_agnocast/runtime.hpp"
#include "types/root.hpp"

#include <pybind11/pybind11.h>

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace rclpy_agnocast
{
namespace
{
void throw_loan_error(const char * operation, const LoanError error)
{
  const std::string prefix = std::string(operation) + ": ";
  switch (error) {
    case LoanError::kNone:
      return;
    case LoanError::kActiveExports:
      throw py::buffer_error(prefix + "writable buffer aliases are still active");
    case LoanError::kWrongPublisher:
      throw py::value_error(prefix + "Loan belongs to a different publisher");
    case LoanError::kStaleGeneration:
      throw std::runtime_error(prefix + "Loan generation is stale");
    case LoanError::kWrongThread:
      throw std::runtime_error(prefix + "operation must run on the Loan creator thread");
    case LoanError::kInvalidPhase:
      throw std::runtime_error(prefix + "Loan has already entered a terminal state");
    case LoanError::kNoActiveExport:
      throw std::runtime_error(prefix + "writable export accounting is inconsistent");
  }
  throw std::runtime_error(prefix + "unknown Loan validation failure");
}
void check_loan_result(const char * operation, const LoanOperationResult & result)
{
  throw_loan_error(operation, result.loan_error);
  binding_detail::throw_worker_error(operation, {result.worker_error, result.failure});
}
py::memoryview writable_buffer(const std::shared_ptr<LoanState> & state)
{
  PyObject * const raw_exporter = create_writable_buffer_export(state);
  if (raw_exporter == nullptr) {
    throw py::error_already_set();
  }
  py::object exporter = py::reinterpret_steal<py::object>(raw_exporter);
  PyObject * const raw_view = PyMemoryView_FromObject(exporter.ptr());
  if (raw_view == nullptr) {
    throw py::error_already_set();
  }
  return py::reinterpret_steal<py::memoryview>(raw_view);
}
template<typename PublisherT>
void bind_publisher_methods(py::class_<PublisherT, std::shared_ptr<PublisherT>> & binding)
{
  binding
  .def("borrow", [](const std::shared_ptr<PublisherT> & publisher, const std::size_t size) {
      py::gil_scoped_release release;
      return publisher->borrow(size);
    }, py::arg("size"))
  .def("publish", [](
      const std::shared_ptr<PublisherT> & publisher, std::shared_ptr<LoanState> loan)
    {
      LoanOperationResult result{};
      {
        py::gil_scoped_release release;
        result = publisher->publish(loan);
      }
      check_loan_result("publish", result);
    }, py::arg("loan"))
  .def("cancel", [](
      const std::shared_ptr<PublisherT> & publisher, std::shared_ptr<LoanState> loan,
      const bool defer_if_exported)
    {
      LoanOperationResult result{};
      {
        py::gil_scoped_release release;
        result = publisher->cancel(loan, defer_if_exported);
      }
      check_loan_result("cancel", result);
    }, py::arg("loan"), py::arg("defer_if_exported") = false)
  .def("close", [](const std::shared_ptr<PublisherT> & publisher) {
      PublisherWorkerResult result{};
      {
        py::gil_scoped_release release;
        result = publisher->close();
      }
      binding_detail::throw_worker_error("publisher close", result);
    })
  .def_property_readonly("id", &PublisherT::publisher_id)
  .def_property_readonly("max_payload_bytes", &PublisherT::max_payload_bytes);
}
#ifdef BUILD_TESTING
struct TestObservations
{
  std::mutex mutex;
  std::uintptr_t data_address{0U};
  std::uint64_t published_sequence{0U};
  std::size_t publish_count{0U};
  std::size_t cancel_count{0U};
  std::thread::id backend_thread;
  std::thread::id terminal_thread;
};
class TestBackend final : public PublisherBackend
{
public:
  explicit TestBackend(std::shared_ptr<TestObservations> observations)
  : observations_(std::move(observations)), owner_(std::this_thread::get_id())
  {
    std::lock_guard<std::mutex> lock(observations_->mutex);
    observations_->backend_thread = owner_;
  }
  BorrowedStorage borrow(const std::size_t size) override
  {
    check_thread();
    data_.resize(size);
    std::lock_guard<std::mutex> lock(observations_->mutex);
    observations_->data_address = reinterpret_cast<std::uintptr_t>(data_.data());
    return {&sequence_, data_.data(), data_.size()};
  }
  void publish() override
  {
    check_thread();
    std::lock_guard<std::mutex> lock(observations_->mutex);
    observations_->published_sequence = sequence_;
    ++observations_->publish_count;
    observations_->terminal_thread = std::this_thread::get_id();
  }
  void cancel() noexcept override
  {
    std::lock_guard<std::mutex> lock(observations_->mutex);
    ++observations_->cancel_count;
    observations_->terminal_thread = std::this_thread::get_id();
  }
private:
  void check_thread() const
  {
    if (std::this_thread::get_id() != owner_) {
      throw std::logic_error("test backend operation changed native worker thread");
    }
  }
  std::shared_ptr<TestObservations> observations_;
  const std::thread::id owner_;
  std::uint64_t sequence_{0U};
  std::vector<std::uint8_t> data_;
};
class TestFactory final : public PublisherBackendFactory
{
public:
  explicit TestFactory(std::shared_ptr<TestObservations> observations)
  : observations_(std::move(observations)) {}
  std::unique_ptr<PublisherBackend> create() override
  {
    return std::make_unique<TestBackend>(observations_);
  }
private:
  std::shared_ptr<TestObservations> observations_;
};
class TestPublisher
{
public:
  explicit TestPublisher(const std::size_t maximum)
  : worker_(std::make_shared<PublisherWorker>()), observations_(std::make_shared<TestObservations>()),
    maximum_(maximum)
  {
    if (maximum == 0U) throw std::invalid_argument("max_payload_bytes must be positive");
    auto factory = std::unique_ptr<PublisherBackendFactory>(new TestFactory(observations_));
    binding_detail::throw_worker_error(
      "test publisher creation", worker_->add_publisher(publisher_id(), factory));
  }
  ~TestPublisher() {(void)close();}
  std::shared_ptr<LoanState> borrow(const std::size_t size)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (closed_) throw std::runtime_error("test publisher is closed");
    if (size > maximum_) throw std::invalid_argument("payload exceeds configured limit");
    const auto result = worker_->borrow(publisher_id(), size);
    binding_detail::throw_worker_error("test publisher borrow", {result.error, result.failure});
    try {
      return std::make_shared<LoanState>(
        worker_, LoanIdentity{publisher_id(), result.region.generation, std::this_thread::get_id()},
        result.region);
    } catch (...) {
      const auto failure = std::current_exception();
      binding_detail::throw_worker_error(
        "test Loan allocation rollback", worker_->cancel(publisher_id(), result.region.generation));
      std::rethrow_exception(failure);
    }
  }
  LoanOperationResult publish(const std::shared_ptr<LoanState> & loan)
  {
    if (!loan) throw std::invalid_argument("loan must not be null");
    return loan->publish(publisher_id());
  }
  LoanOperationResult cancel(const std::shared_ptr<LoanState> & loan, const bool defer)
  {
    if (!loan) throw std::invalid_argument("loan must not be null");
    return loan->cancel(publisher_id(), defer);
  }
  PublisherWorkerResult close() noexcept
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (closed_) return {PublisherWorkerError::kNone, nullptr};
    auto result = worker_->remove_publisher(publisher_id());
    if (result.error != PublisherWorkerError::kNone) return result;
    result = worker_->stop();
    if (
      result.error == PublisherWorkerError::kNone ||
      result.error == PublisherWorkerError::kStopped)
    {
      closed_ = true;
      return {PublisherWorkerError::kNone, nullptr};
    }
    return result;
  }
  std::uintptr_t publisher_id() const noexcept
  {return reinterpret_cast<std::uintptr_t>(observations_.get());}
  std::size_t max_payload_bytes() const noexcept {return maximum_;}
  std::uintptr_t data_address() const {return read(&TestObservations::data_address);}
  std::uint64_t published_sequence() const {return read(&TestObservations::published_sequence);}
  std::size_t publish_count() const {return read(&TestObservations::publish_count);}
  std::size_t cancel_count() const {return read(&TestObservations::cancel_count);}
  bool terminal_on_backend_thread() const
  {
    std::lock_guard<std::mutex> lock(observations_->mutex);
    return observations_->terminal_thread == observations_->backend_thread;
  }
private:
  template<typename ValueT>
  ValueT read(ValueT TestObservations::* member) const
  {
    std::lock_guard<std::mutex> lock(observations_->mutex);
    return observations_.get()->*member;
  }
  std::shared_ptr<PublisherWorker> worker_;
  std::shared_ptr<TestObservations> observations_;
  mutable std::mutex state_mutex_;
  const std::size_t maximum_;
  bool closed_{false};
};
#endif
}  // namespace
void bind_publisher(py::module_ & module)
{
  py::enum_<LoanPhase>(module, "LoanPhase")
  .value("FILLING", LoanPhase::kFilling)
  .value("CANCEL_PENDING", LoanPhase::kCancelPending)
  .value("PUBLISHING", LoanPhase::kPublishing)
  .value("CANCELLING", LoanPhase::kCancelling)
  .value("PUBLISHED", LoanPhase::kPublished)
  .value("CANCELLED", LoanPhase::kCancelled)
  .value("FAILED", LoanPhase::kFailed);

  py::class_<LoanState, std::shared_ptr<LoanState>>(module, "LoanState")
  .def_property("sequence", [](const std::shared_ptr<LoanState> & state) {
      std::uint64_t value{0U};
      throw_loan_error("read sequence", state->sequence(value));
      return value;
    }, [](const std::shared_ptr<LoanState> & state, const std::uint64_t value) {
      throw_loan_error("write sequence", state->set_sequence(value));
    })
  .def_property_readonly("buffer", &writable_buffer)
  .def_property_readonly("phase", &LoanState::phase)
  .def_property_readonly("active_writable_exports", &LoanState::active_writable_exports)
  .def("abandon", &LoanState::abandon);

  auto publisher = py::class_<PublisherHandle, std::shared_ptr<PublisherHandle>>(
    module, "PublisherHandle");
  bind_publisher_methods(publisher);
  publisher.def("borrow_message", [](const std::shared_ptr<PublisherHandle> & handle) {
      py::gil_scoped_release release;
      return handle->borrow_message();
    });

#ifdef BUILD_TESTING
  auto test_publisher = py::class_<TestPublisher, std::shared_ptr<TestPublisher>>(
    module, "_TestPublisher");
  test_publisher.def(py::init([](const std::size_t maximum) {
      py::gil_scoped_release release;
      return std::make_shared<TestPublisher>(maximum);
    }), py::arg("max_payload_bytes") = 1048576U);
  bind_publisher_methods(test_publisher);
  test_publisher
  .def_property_readonly("native_data_address", &TestPublisher::data_address)
  .def_property_readonly("published_sequence", &TestPublisher::published_sequence)
  .def_property_readonly("publish_count", &TestPublisher::publish_count)
  .def_property_readonly("cancel_count", &TestPublisher::cancel_count)
  .def_property_readonly("terminal_on_backend_thread", &TestPublisher::terminal_on_backend_thread);
#endif
}

}  // namespace rclpy_agnocast
