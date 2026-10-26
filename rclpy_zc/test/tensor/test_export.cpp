// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclpy_agnocast/publisher_worker.hpp"
#include "rclpy_agnocast/types/field_ops.hpp"
#include "tensor/export.hpp"

namespace
{
namespace tensor = rclpy_agnocast::tensor;
using rclpy_agnocast::BorrowedStorage;
using rclpy_agnocast::PublisherBackend;
using rclpy_agnocast::PublisherBackendFactory;
using rclpy_agnocast::PublisherWorker;
using rclpy_agnocast::PublisherWorkerError;
using rclpy_agnocast::types::BufferSpan;
using rclpy_agnocast::types::FieldKind;
using rclpy_agnocast::types::FieldOps;
using rclpy_agnocast::types::FieldRef;
using rclpy_agnocast::types::RootOwner;
using rclpy_agnocast::types::RootPhase;
using rclpy_agnocast::types::scalar_ops;
using rclpy_agnocast::types::Status;
using rclpy_agnocast::types::TypeDescriptor;
using rclpy_agnocast::types::TypeOps;
using rclpy_agnocast::types::UnsupportedOperation;
constexpr std::uintptr_t kPublisher = 91;
enum class Fault
{
  kMisaligned, kSpanWidth, kSpanSize, kSpanEndian, kSpanFloat,
  kElementEndian, kElementFormat, kElementWidth, kNone
};
struct Trace
{
  alignas(std::int32_t) std::array<std::uint8_t, 12> bytes{};
  Fault fault{Fault::kNone};
  std::size_t elements{2};
  bool live{false};
  std::size_t cancels{0};
  std::size_t destroys{0};
  std::thread::id worker_thread;
  std::thread::id cancel_thread;
  std::thread::id destroy_thread;
};

class Backend final : public PublisherBackend
{
public:
  explicit Backend(std::shared_ptr<Trace> trace) : trace_(std::move(trace)) {}
  ~Backend() override
  {
    ++trace_->destroys;
    trace_->destroy_thread = std::this_thread::get_id();
  }
  BorrowedStorage borrow(std::size_t) override {throw std::logic_error("root only");}
  void * borrow_root() override {trace_->live = true; return trace_.get();}
  void publish() override {throw std::logic_error("cancel only");}
  void cancel() noexcept override
  {
    if (trace_->live) {++trace_->cancels;}
    trace_->live = false;
    trace_->cancel_thread = std::this_thread::get_id();
  }

private:
  std::shared_ptr<Trace> trace_;
};
class Factory final : public PublisherBackendFactory
{
public:
  explicit Factory(std::shared_ptr<Trace> trace) : trace_(std::move(trace)) {}
  std::unique_ptr<PublisherBackend> create() override
  {
    trace_->worker_thread = std::this_thread::get_id();
    return std::make_unique<Backend>(trace_);
  }

private:
  std::shared_ptr<Trace> trace_;
};

class NativeTensor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(0U, tensor::live_managers());
    const std::int32_t values[] = {11, 22};
    std::memcpy(trace->bytes.data(), values, sizeof(values));
    field.item_size = sizeof(values);
    field.element = &element;
    field.size = [](FieldRef ref, std::size_t * size) noexcept->Status {
        *size = static_cast<Trace *>(ref.object)->elements; return {};
      };
    field.dynamic_bytes = [](FieldRef, std::size_t * bytes) noexcept->Status {
        *bytes = 0; return {};
      };
    field.buffer = [](FieldRef ref, BufferSpan * span) noexcept->Status {
        auto & state = *static_cast<Trace *>(ref.object);
        *span = {state.bytes.data(), state.elements, 4, "i"};
        switch (state.fault) {
          case Fault::kMisaligned: span->data = state.bytes.data() + 1; break;
          case Fault::kSpanWidth: span->item_size = 8; break;
          case Fault::kSpanSize: span->size = 1; break;
          case Fault::kSpanEndian: span->format = ">i"; break;
          case Fault::kSpanFloat: span->format = "f"; break;
          default: break;
        }
        return {};
      };
    std::unique_ptr<PublisherBackendFactory> factory = std::make_unique<Factory>(trace);
    ASSERT_EQ(PublisherWorkerError::kNone, worker->add_publisher(kPublisher, factory).error);
    const auto loan = worker->borrow_root(kPublisher);
    ASSERT_EQ(PublisherWorkerError::kNone, loan.error);
    owner = RootOwner::create(worker, kPublisher, loan.generation, loan.root, descriptor, 4096);
  }
  void TearDown() override
  {
    EXPECT_EQ(0U, tensor::live_managers());
    owner.reset();
    EXPECT_EQ(PublisherWorkerError::kNone, worker->wait_until_idle().error);
    EXPECT_EQ(PublisherWorkerError::kNone, worker->remove_publisher(kPublisher).error);
    EXPECT_EQ(1U, trace->cancels);
    EXPECT_EQ(1U, trace->destroys);
    EXPECT_EQ(trace->worker_thread, trace->cancel_thread);
    EXPECT_EQ(trace->worker_thread, trace->destroy_thread);
  }
  void configure(Fault fault)
  {
    trace->fault = fault;
    element = *scalar_ops<std::int32_t>();
    if (fault == Fault::kElementEndian) {element.format = ">i";}
    if (fault == Fault::kElementFormat) {element.format = "f";}
    if (fault == Fault::kElementWidth) {element.item_size = 8;}
  }
  void export_and_release()
  {
    auto * output = tensor::export_dlpack(owner, {}, 0, {2});
    output->deleter(output);
  }
  std::shared_ptr<Trace> trace{std::make_shared<Trace>()};
  std::shared_ptr<PublisherWorker> worker{std::make_shared<PublisherWorker>()};
  FieldOps element{*scalar_ops<std::int32_t>()};
  FieldOps field{FieldKind::kArray};
  TypeOps operations{&field, nullptr};
  TypeDescriptor descriptor{
    "tensor_test/msg/Array", std::string(64, 'a'), "[]", sizeof(Trace), alignof(Trace),
    &operations};
  std::shared_ptr<RootOwner> owner;
};

class MalformedTensor : public NativeTensor, public ::testing::WithParamInterface<Fault> {};

TEST_F(NativeTensor, ZeroExtentDoesNotBypassStrideVolumeOverflow)
{
  trace->elements = 0;
  const std::int64_t huge = std::int64_t{1} << 62;
  const std::vector<std::vector<std::int64_t>> shapes = {
    {0, huge, huge}, {huge, 0, huge}, {huge, huge, 0}};
  for (const auto & shape : shapes) {
    EXPECT_THROW({
      auto * output = tensor::export_dlpack(owner, {}, 0, shape);
      output->deleter(output);
    }, std::invalid_argument);
    EXPECT_EQ(0U, owner->snapshot().exports);
    EXPECT_EQ(RootPhase::kPrepare, owner->snapshot().phase);
    EXPECT_EQ(0U, owner->info({}, 0).size);
  }
  auto * output = tensor::export_dlpack(owner, {}, 0, {0});
  EXPECT_EQ(nullptr, output->dl_tensor.data);
  EXPECT_EQ(1U, owner->snapshot().exports);
  output->deleter(output);
  EXPECT_EQ(0U, owner->snapshot().exports);
  owner->cancel(kPublisher, false);
}

TEST_F(NativeTensor, CapsulePinMakesRepeatedDeleterPreserveIndependentExport)
{
  const auto independent = owner->acquire_export({}, 0);
  EXPECT_EQ(trace->bytes.data(), independent.data);
  auto * output = tensor::export_dlpack(owner, {}, 0, {2});
  tensor::retain_capsule(output);
  EXPECT_EQ(1U, tensor::live_managers());
  EXPECT_EQ(2U, owner->snapshot().exports);
  output->deleter(output);
  EXPECT_EQ(1U, owner->snapshot().exports);
  output->deleter(output);
  EXPECT_EQ(1U, owner->snapshot().exports);
  EXPECT_EQ(2U, owner->info({}, 0).size);
  EXPECT_TRUE(trace->live);
  EXPECT_EQ(0U, trace->cancels);
  EXPECT_EQ(1U, tensor::live_managers());
  tensor::release_capsule(output, true);
  EXPECT_EQ(0U, tensor::live_managers());
  EXPECT_EQ(1U, owner->snapshot().exports);
  owner->release_export();
  EXPECT_EQ(0U, owner->snapshot().exports);
  owner->cancel(kPublisher, false);
}

TEST_F(NativeTensor, NativeThreadDeleterReturnsRootAfterCapsulePinIsGone)
{
  auto * output = tensor::export_dlpack(owner, {}, 0, {2});
  tensor::retain_capsule(output);
  tensor::release_capsule(output, false);
  EXPECT_EQ(1U, tensor::live_managers());
  EXPECT_EQ(1U, owner->snapshot().exports);
  EXPECT_EQ(trace->bytes.data(), output->dl_tensor.data);
  std::weak_ptr<RootOwner> lifetime = owner;
  owner.reset();
  EXPECT_FALSE(lifetime.expired());
  std::thread consumer([output] {output->deleter(output);});
  consumer.join();
  EXPECT_EQ(0U, tensor::live_managers());
  EXPECT_TRUE(lifetime.expired());
  EXPECT_EQ(PublisherWorkerError::kNone, worker->wait_until_idle().error);
  EXPECT_EQ(1U, trace->cancels);
  EXPECT_FALSE(trace->live);
  EXPECT_EQ(trace->worker_thread, trace->cancel_thread);
}

TEST_P(MalformedTensor, FailedExportReturnsReservationAndAllowsValidExport)
{
  configure(GetParam());
  if (GetParam() == Fault::kElementEndian || GetParam() == Fault::kElementFormat ||
    GetParam() == Fault::kElementWidth)
  {
    EXPECT_THROW(tensor::describe(owner->info({}, 0)), UnsupportedOperation);
    EXPECT_THROW(export_and_release(), UnsupportedOperation);
  } else {
    EXPECT_STREQ("int32", tensor::describe(owner->info({}, 0)).name);
    EXPECT_THROW(export_and_release(), std::invalid_argument);
  }
  EXPECT_EQ(0U, owner->snapshot().exports);
  EXPECT_EQ(0U, tensor::live_managers());
  EXPECT_EQ(2U, owner->info({}, 0).size);
  EXPECT_TRUE(trace->live);
  EXPECT_EQ(0U, trace->cancels);

  configure(Fault::kNone);
  const auto destroy = [](DLManagedTensor * value) {value->deleter(value);};
  std::unique_ptr<DLManagedTensor, decltype(destroy)> output(
    tensor::export_dlpack(owner, {}, 0, {2}), destroy);
  EXPECT_EQ(trace->bytes.data(), output->dl_tensor.data);
  EXPECT_EQ(1U, owner->snapshot().exports);
  EXPECT_EQ(kDLCPU, output->dl_tensor.device.device_type);
  EXPECT_EQ(1, output->dl_tensor.ndim);
  EXPECT_EQ(2, output->dl_tensor.shape[0]);
  EXPECT_EQ(nullptr, output->dl_tensor.strides);
  EXPECT_EQ(0U, output->dl_tensor.byte_offset);
  EXPECT_EQ(kDLInt, output->dl_tensor.dtype.code);
  EXPECT_EQ(32, output->dl_tensor.dtype.bits);
  output.reset();
  EXPECT_EQ(0U, owner->snapshot().exports);
  owner->cancel(kPublisher, false);
  EXPECT_EQ(1U, trace->cancels);
  EXPECT_FALSE(trace->live);
}

INSTANTIATE_TEST_SUITE_P(
  ProviderContracts, MalformedTensor, ::testing::Values(
    Fault::kMisaligned, Fault::kSpanWidth, Fault::kSpanSize, Fault::kSpanEndian,
    Fault::kSpanFloat, Fault::kElementEndian, Fault::kElementFormat, Fault::kElementWidth));
}  // namespace
