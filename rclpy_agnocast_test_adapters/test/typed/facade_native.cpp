// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include <unistd.h>
#include <atomic>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include "agnocast/agnocast.hpp"
#include "native_fixture.hpp"

namespace
{
std::mutex output_mutex;

void emit(const std::string & fields)
{
  std::lock_guard<std::mutex> lock(output_mutex);
  std::cout << "EVENT " << fields << std::endl;
}

template<typename T>
void publish(const std::shared_ptr<agnocast::Node> & node, const std::string & topic)
{
  auto publisher = node->create_publisher<T>(topic, rclcpp::QoS(4).best_effort());
  emit("name=READY pid=" + std::to_string(getpid()));
  std::string command;
  while (std::getline(std::cin, command)) {
    if (command == "quit") {break;}
    if (command != "publish") {throw std::invalid_argument("unknown_command");}
    auto message = publisher->borrow_loaned_message();
    typed_fixture::fill(*message);
    typed_fixture::validate(*message);
    const auto bytes = typed_fixture::accounted(*message);
    publisher->publish(std::move(message));
    emit("name=PUBLISHED accounted_bytes=" + std::to_string(bytes) + " valid=true");
  }
}

template<typename T>
void subscribe(const std::shared_ptr<agnocast::Node> & node, const std::string & topic)
{
  std::atomic<std::size_t> received{0};
  auto subscription = node->create_subscription<T>(
    topic, rclcpp::QoS(4).best_effort(),
    [&received](const agnocast::ipc_shared_ptr<const T> & message) {
      try {
        typed_fixture::validate(*message);
        const auto count = ++received;
        emit(
          "name=RX count=" + std::to_string(count) +
          " accounted_bytes=" + std::to_string(typed_fixture::accounted(*message)) +
          " valid=true");
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

template<typename T>
void run(const std::string & mode, const std::string & topic)
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.use_clock_thread(false);
  auto node = std::make_shared<agnocast::Node>(
    "typed_facade_native_" + std::to_string(getpid()), options);
  if (mode == "publisher") {publish<T>(node, topic);} else if (mode == "subscriber") {
    subscribe<T>(node, topic);
  } else {
    throw std::invalid_argument("unknown_mode");
  }
}
}  // namespace

/**
 * @brief Drive native ROS message layouts independently of generated adapter code.
 * @param[in] argc Argument count; mode, message kind and topic are required.
 * @param[in] argv Borrowed argument strings, valid for the duration of main.
 * @return Zero after orderly shutdown; nonzero after a protocol or native failure.
 */
int main(int argc, char ** argv)
{
  if (argc != 4) {return 2;}
  try {
    agnocast::init(0, nullptr);
    const std::string kind = argv[2];
    if (kind == "Image") {
      run<typed_fixture::Image>(argv[1], argv[3]);
    } else if (kind == "PointCloud2") {
      run<typed_fixture::PointCloud2>(argv[1], argv[3]);
    } else if (kind == "Containers") {
      run<typed_fixture::Containers>(argv[1], argv[3]);
    } else {
      throw std::invalid_argument("unknown_kind");
    }
    agnocast::shutdown();
  } catch (const std::exception & error) {
    emit(std::string("name=ERROR reason=") + error.what());
    return 1;
  }
  return 0;
}
