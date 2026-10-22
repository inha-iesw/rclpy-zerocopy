// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include "agnocast/agnocast.hpp"
#include "rclpy_agnocast_test_msgs/msg/containers.hpp"
#include "rclpy_agnocast_test_msgs/msg/nested.hpp"
#include "rclpy_agnocast_test_msgs/msg/scalars.hpp"

namespace
{
using rclpy_agnocast_test_msgs::msg::Containers;
using rclpy_agnocast_test_msgs::msg::Nested;
using rclpy_agnocast_test_msgs::msg::Scalars;
std::mutex output_mutex;

void emit(const std::string & fields)
{
  std::lock_guard<std::mutex> lock(output_mutex);
  std::cout << "EVENT " << fields << std::endl;
}
void expect(bool valid, const char * field)
{
  if (!valid) {throw std::runtime_error(field);}
}
template<typename C, typename T>
void sequence(const C & actual, std::initializer_list<T> expected, const char * field)
{
  expect(actual.size() == expected.size(), field);
  expect(std::equal(actual.begin(), actual.end(), expected.begin()), field);
}
void validate(const Scalars & value)
{
  expect(value.flag, "flag");
  expect(value.byte_value == 255, "byte_value");
  expect(value.character == 65, "character");
  expect(value.signed8 == -128, "signed8");
  expect(value.unsigned8 == 255, "unsigned8");
  expect(value.signed16 == -32768, "signed16");
  expect(value.unsigned16 == 65535, "unsigned16");
  expect(value.signed32 == std::numeric_limits<std::int32_t>::min(), "signed32");
  expect(value.unsigned32 == std::numeric_limits<std::uint32_t>::max(), "unsigned32");
  expect(value.signed64 == std::numeric_limits<std::int64_t>::min(), "signed64");
  expect(value.unsigned64 == std::numeric_limits<std::uint64_t>::max(), "unsigned64");
  expect(value.real32 == 1.25F, "real32");
  expect(value.real64 == -2.5, "real64");
}
void validate(const Containers & value)
{
  sequence(value.fixed, {1.25, -2.5, 0.0, 4.0}, "fixed");
  sequence(value.fixed_flags, {true, false, true, false}, "fixed_flags");
  sequence(value.bounded, {0.5F, -1.5F, 2.5F}, "bounded");
  sequence(value.values, {0, 1, 127, 128, 255}, "values");
  sequence(value.bounded_flags, {false, true, false}, "bounded_flags");
  sequence(value.flags, {true, false, true, false, true}, "flags");
  expect(value.text == "typed π buffer payload", "text");
  expect(value.bounded_text == "µ", "bounded_text");
  expect(value.wide_text == u"𝄞 typed wide payload", "wide_text");
  expect(value.bounded_wide_text == u"한글", "bounded_wide_text");
  sequence(value.texts, {"abcdefabcdefabcdefabcdef", "π"}, "texts");
  sequence(value.bounded_texts, {"0123456789abcdef", "x"}, "bounded_texts");
  sequence(value.wide_texts, {u"큰큰큰큰큰큰큰큰큰큰큰큰", u""}, "wide_texts");
  sequence(value.bounded_wide_texts, {u"한글", u"Z"}, "bounded_wide_texts");
}
void validate(const Nested & value)
{
  validate(value.child);
  validate(value.containers);
  for (const auto & child : value.fixed_children) {
    validate(child);
  }
  expect(value.children.size() == 2, "children_size");
  for (const auto & child : value.children) {
    validate(child);
  }
  expect(value.bounded_children.size() == 1, "bounded_children_size");
  validate(value.bounded_children[0]);
}

template<typename S>
std::size_t string_storage(const S & value)
{
  const auto data = reinterpret_cast<std::uintptr_t>(value.data());
  const auto object = reinterpret_cast<std::uintptr_t>(&value);
  if (data >= object && data < object + sizeof(S)) {return 0;}
  return (value.capacity() + 1) * sizeof(typename S::value_type);
}
template<typename C>
std::size_t strings_storage(const C & value)
{
  auto bytes = value.capacity() * sizeof(typename C::value_type);
  for (const auto & text : value) {
    bytes += string_storage(text);
  }
  return bytes;
}
std::size_t dynamic_storage(const Containers & value)
{
  return value.bounded.capacity() * sizeof(float) + value.values.capacity() +
         (value.bounded_flags.capacity() + 7) / 8 + (value.flags.capacity() + 7) / 8 +
         string_storage(value.text) + string_storage(value.bounded_text) +
         string_storage(value.wide_text) + string_storage(value.bounded_wide_text) +
         strings_storage(value.texts) + strings_storage(value.bounded_texts) +
         strings_storage(value.wide_texts) + strings_storage(value.bounded_wide_texts);
}
std::size_t accounted(const Scalars &) {return sizeof(Scalars);}
std::size_t accounted(const Containers & value)
{
  return sizeof(Containers) + dynamic_storage(value);
}
std::size_t accounted(const Nested & value)
{
  auto bytes = sizeof(Nested) + dynamic_storage(value.containers) +
    value.children.capacity() * sizeof(Scalars) + value.bounded_children.capacity() *
    sizeof(Containers);
  for (const auto & child : value.bounded_children) {
    bytes += dynamic_storage(child);
  }
  return bytes;
}
std::uintptr_t data_address(const Scalars &) {return 0;}
std::uintptr_t data_address(const Containers & value)
{return reinterpret_cast<std::uintptr_t>(value.values.data());}
std::uintptr_t data_address(const Nested & value) {return data_address(value.containers);}

template<typename Message>
void run(const std::string & topic)
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.use_clock_thread(false);
  auto node = std::make_shared<agnocast::Node>(
    "typed_inspector_" + std::to_string(getpid()), options);
  std::atomic<std::size_t> received{0};
  auto subscription = node->create_subscription<Message>(
    topic, 4,
    [&received](const agnocast::ipc_shared_ptr<const Message> & message) {
      try {
        validate(*message);
        const auto count = ++received;
        emit(
          "name=RX count=" + std::to_string(count) +
          " accounted_bytes=" + std::to_string(accounted(*message)) +
          " root_address=" + std::to_string(reinterpret_cast<std::uintptr_t>(message.get())) +
          " data_address=" + std::to_string(data_address(*message)) + " valid=true");
      } catch (const std::exception & error) {
        emit(std::string("name=ERROR field=") + error.what());
      }
    });
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(node);
  std::exception_ptr failure;
  std::thread worker([&] {
      try {
        executor.spin();
      } catch (...) {
        failure = std::current_exception();
      }
    });
  emit("name=READY pid=" + std::to_string(getpid()));
  std::string command;
  while (std::getline(std::cin, command)) {
    if (command == "quit") {break;}
    if (command == "connections") {
      const auto count = subscription->get_publisher_count();
      emit(
        "name=CONNECTED count=" + std::to_string(count) +
        " connected=" + (count ? "true" : "false"));
    } else if (command == "stats") {
      emit("name=STATS received=" + std::to_string(received));
    } else {
      emit("name=ERROR reason=unknown_command");
    }
  }
  executor.cancel();
  worker.join();
  if (failure) {std::rethrow_exception(failure);}
  executor.remove_node(node);
}
}  // namespace

/**
 * @brief Inspect independently typed fixture payloads in a separate native process.
 * @param[in] argc Argument count; fixture name and topic are required.
 * @param[in] argv Borrowed command arguments, valid throughout the call.
 * @return Zero after orderly teardown; nonzero after invalid input or native failure.
 */
int main(int argc, char ** argv)
{
  if (argc != 3) {return 2;}
  try {
    agnocast::init(0, nullptr);
    const std::string kind = argv[1];
    if (kind == "Scalars") {run<Scalars>(argv[2]);} else if (kind == "Containers") {
      run<Containers>(argv[2]);
    } else if (kind == "Nested") {run<Nested>(argv[2]);} else {
      throw std::invalid_argument("unknown_fixture");
    }
    agnocast::shutdown();
  } catch (const std::exception & error) {
    emit(std::string("name=ERROR reason=") + error.what());
    return 1;
  }
  return 0;
}
