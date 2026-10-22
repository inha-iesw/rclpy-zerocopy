// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "native_publisher.hpp"
#include "root.hpp"
#include "rclpy_agnocast/types/field_ops.hpp"

namespace py = pybind11;
namespace rclpy_agnocast
{
namespace types
{
namespace
{
struct Child {std::vector<std::int32_t> values;};
struct TestMessage
{
  bool flag{false};
  std::int32_t signed_value{0};
  std::uint64_t unsigned_value{0};
  float real{0};
  std::array<bool, 2> bools{};
  std::array<std::int8_t, 2> i8{};
  std::array<std::uint8_t, 2> u8{};
  std::array<std::int16_t, 2> i16{};
  std::array<std::uint16_t, 2> u16{};
  std::array<std::int32_t, 2> i32{};
  std::array<std::uint32_t, 2> u32{};
  std::array<std::int64_t, 2> i64{};
  std::array<std::uint64_t, 2> u64{};
  std::array<float, 2> f32{};
  std::array<double, 2> f64{};
  std::vector<std::int32_t> values;
  std::vector<bool> packed;
  std::vector<Child> children;
  std::string text;
  std::u16string wide;
};

template<typename T>
const FieldOps * fixed_ops()
{
  static const auto ops = array_ops<std::array<T, 2>>(scalar_ops<T>());
  return &ops;
}
template<typename T, auto Member>
FieldMember member(const char * name, const FieldOps * ops)
{
  return {name, ops, [] (void * object) noexcept->void * {
      return std::addressof(static_cast<T *>(object)->*Member);
    }};
}

const FieldOps * message_fields()
{
  static const auto values = sequence_ops<std::vector<std::int32_t>>(scalar_ops<std::int32_t>(), 8);
  static const FieldMember child_members[] = {member<Child, &Child::values>("values", &values)};
  static const auto child = message_ops<Child>(child_members, 1);
  static const auto children = sequence_ops<std::vector<Child>>(&child, 4);
  static const auto packed = sequence_ops<std::vector<bool>>(scalar_ops<bool>(), 8);
  static const auto text = string_ops<std::string>();
  static const auto wide = string_ops<std::u16string>();
  static const FieldMember members[] = {
    member<TestMessage, &TestMessage::flag>("flag", scalar_ops<bool>()),
    member<TestMessage, &TestMessage::signed_value>("signed", scalar_ops<std::int32_t>()),
    member<TestMessage, &TestMessage::unsigned_value>("unsigned", scalar_ops<std::uint64_t>()),
    member<TestMessage, &TestMessage::real>("real", scalar_ops<float>()),
    member<TestMessage, &TestMessage::bools>("bools", fixed_ops<bool>()),
    member<TestMessage, &TestMessage::i8>("i8", fixed_ops<std::int8_t>()),
    member<TestMessage, &TestMessage::u8>("u8", fixed_ops<std::uint8_t>()),
    member<TestMessage, &TestMessage::i16>("i16", fixed_ops<std::int16_t>()),
    member<TestMessage, &TestMessage::u16>("u16", fixed_ops<std::uint16_t>()),
    member<TestMessage, &TestMessage::i32>("i32", fixed_ops<std::int32_t>()),
    member<TestMessage, &TestMessage::u32>("u32", fixed_ops<std::uint32_t>()),
    member<TestMessage, &TestMessage::i64>("i64", fixed_ops<std::int64_t>()),
    member<TestMessage, &TestMessage::u64>("u64", fixed_ops<std::uint64_t>()),
    member<TestMessage, &TestMessage::f32>("f32", fixed_ops<float>()),
    member<TestMessage, &TestMessage::f64>("f64", fixed_ops<double>()),
    member<TestMessage, &TestMessage::values>("values", &values),
    member<TestMessage, &TestMessage::packed>("packed", &packed),
    member<TestMessage, &TestMessage::children>("children", &children),
    member<TestMessage, &TestMessage::text>("text", &text),
    member<TestMessage, &TestMessage::wide>("wide", &wide),
  };
  static const auto root = message_ops<TestMessage>(members, std::size(members));
  return &root;
}

struct Trace : std::enable_shared_from_this<Trace>
{
  std::mutex mutex;
  TestMessage * live{nullptr};
  std::thread::id creator{std::this_thread::get_id()}, worker;
  std::size_t creates{0}, borrows{0}, publishes{0}, cancels{0}, destroys{0};
  bool worker_only{true}, fail_publish{false};
  void observe() {worker_only &= std::this_thread::get_id() == worker && worker != creator;}
};
struct Endpoint
{
  std::shared_ptr<Trace> trace;
  std::unique_ptr<TestMessage> root;
  void reset() {root.reset(); trace->live = nullptr;}
};

Status create(void * node, const char *, std::size_t, NativeEndpoint * output) noexcept
{
  return field_detail::protect(
    [&] {
      auto trace = static_cast<Trace *>(node)->shared_from_this();
      auto endpoint = std::make_unique<Endpoint>();
      endpoint->trace = trace;
      std::lock_guard<std::mutex> lock(trace->mutex);
      trace->worker = std::this_thread::get_id();
      trace->observe();
      ++trace->creates;
      output->borrow = [] (void * object, void ** root) noexcept->Status {
        return field_detail::protect(
          [&] {
            auto & value = *static_cast<Endpoint *>(object);
            std::lock_guard<std::mutex> lock(value.trace->mutex);
            value.trace->observe();
            value.root = std::make_unique<TestMessage>();
            *root = value.trace->live = value.root.get();
            ++value.trace->borrows;
          });
      };
      output->publish = [] (void * object) noexcept->Status {
        return field_detail::protect(
          [&] {
            auto & value = *static_cast<Endpoint *>(object);
            std::lock_guard<std::mutex> lock(value.trace->mutex);
            value.trace->observe();
            ++value.trace->publishes;
            value.reset();
            if (value.trace->fail_publish) {
              throw std::runtime_error("injected typed publish failure");
            }
          });
      };
      output->cancel = [] (void * object) noexcept->void {
        auto & value = *static_cast<Endpoint *>(object);
        std::lock_guard<std::mutex> lock(value.trace->mutex);
        value.trace->observe();
        ++value.trace->cancels;
        value.reset();
      };
      output->destroy = [] (void * object) noexcept->void {
        std::unique_ptr<Endpoint> value(static_cast<Endpoint *>(object));
        std::lock_guard<std::mutex> lock(value->trace->mutex);
        value->trace->observe();
        ++value->trace->destroys;
        value->reset();
      };
      output->object = endpoint.release();
    });
}

void check(const PublisherWorkerResult & result)
{
  if (result.failure) {std::rethrow_exception(result.failure);}
  if (result.error != PublisherWorkerError::kNone) {throw std::runtime_error("test worker busy");}
}
template<typename T>
void inspect_array(py::dict & result, const char * name, const std::array<T, 2> & value)
{
  result[name] = py::cast(value);
  result[(std::string(name) + "_address").c_str()] = reinterpret_cast<std::uintptr_t>(value.data());
}

// The endpoint substitutes only storage/transport; all worker, owner, and binding code is real.
class TestTypedPublisher
{
public:
  explicit TestTypedPublisher(std::size_t maximum)
  : maximum_(maximum)
  {
    static const TypeOps ops{message_fields(), create};
    descriptor_ = std::make_shared<TypeDescriptor>(
      TypeDescriptor{
            "binding_test/msg/Message", std::string(64, 'a'), "[]", sizeof(TestMessage),
            alignof(TestMessage), &ops});
    auto factory = native_publisher_factory(trace_.get(), "/binding_test", 2, descriptor_);
    check(worker_->add_publisher(id(), factory));
  }
  std::shared_ptr<RootOwner> borrow_message()
  {
    const auto result = worker_->borrow_root(id());
    check({result.error, result.failure});
    return RootOwner::create(worker_, id(), result.generation, result.root, *descriptor_, maximum_);
  }
  void wait_idle() {check(worker_->wait_until_idle());}
  void close()
  {
    if (!closed_) {check(worker_->remove_publisher(id())); closed_ = true;}
  }
  std::uintptr_t id() const {return reinterpret_cast<std::uintptr_t>(this);}
  std::size_t maximum() const {return maximum_;}
  bool fail_publish() const
  {
    std::lock_guard<std::mutex> lock(trace_->mutex);
    return trace_->fail_publish;
  }
  void fail_publish(bool value)
  {
    std::lock_guard<std::mutex> lock(trace_->mutex);
    trace_->fail_publish = value;
  }
  py::dict inspect() const
  {
    std::lock_guard<std::mutex> lock(trace_->mutex);
    py::dict result;
    result["live"] = trace_->live != nullptr;
    result["creates"] = trace_->creates;
    result["borrows"] = trace_->borrows;
    result["publishes"] = trace_->publishes;
    result["cancels"] = trace_->cancels;
    result["destroys"] = trace_->destroys;
    result["worker_only"] = trace_->worker_only;
    if (!trace_->live) {return result;}
    const auto & root = *trace_->live;
    result["flag"] = root.flag;
    result["signed"] = root.signed_value;
    result["unsigned"] = root.unsigned_value;
    result["real"] = root.real;
    inspect_array(result, "bools", root.bools);
    inspect_array(result, "i8", root.i8);
    inspect_array(result, "u8", root.u8);
    inspect_array(result, "i16", root.i16);
    inspect_array(result, "u16", root.u16);
    inspect_array(result, "i32", root.i32);
    inspect_array(result, "u32", root.u32);
    inspect_array(result, "i64", root.i64);
    inspect_array(result, "u64", root.u64);
    inspect_array(result, "f32", root.f32);
    inspect_array(result, "f64", root.f64);
    result["text_bytes"] = py::bytes(root.text);
    result["wide_units"] = std::vector<std::uint16_t>(root.wide.begin(), root.wide.end());
    py::list children;
    for (const auto & child : root.children) {children.append(py::cast(child.values));}
    result["child_values"] = children;
    return result;
  }

private:
  std::shared_ptr<Trace> trace_{std::make_shared<Trace>()};
  std::shared_ptr<PublisherWorker> worker_{std::make_shared<PublisherWorker>()};
  std::shared_ptr<const TypeDescriptor> descriptor_;
  std::size_t maximum_;
  bool closed_{false};
};
}  // namespace

void bind_typed_testing(py::module_ & module)
{
  py::class_<TestTypedPublisher>(module, "_TestTypedPublisher")
  .def(py::init<std::size_t>())
  .def(
    "borrow_message", &TestTypedPublisher::borrow_message,
    py::call_guard<py::gil_scoped_release>())
  .def("close", &TestTypedPublisher::close, py::call_guard<py::gil_scoped_release>())
  .def("wait_idle", &TestTypedPublisher::wait_idle, py::call_guard<py::gil_scoped_release>())
  .def("inspect", &TestTypedPublisher::inspect)
  .def_property_readonly("id", &TestTypedPublisher::id)
  .def_property_readonly("max_payload_bytes", &TestTypedPublisher::maximum)
  .def_property(
    "fail_publish", py::overload_cast<>(&TestTypedPublisher::fail_publish, py::const_),
    py::overload_cast<bool>(&TestTypedPublisher::fail_publish));
}
}  // namespace types
}  // namespace rclpy_agnocast
