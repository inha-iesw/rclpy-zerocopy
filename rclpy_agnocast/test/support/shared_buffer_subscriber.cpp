// Copyright 2026 rclpy zerocopy maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include "agnocast/agnocast.hpp"
#include "rclpy_agnocast_msgs/msg/shared_buffer.hpp"

namespace
{

using Message = rclpy_agnocast_msgs::msg::SharedBuffer;
using EventField = std::pair<std::string, std::string>;

std::mutex output_mutex;

void write_all(const std::string & text)
{
  std::size_t offset = 0U;
  while (offset < text.size()) {
    const auto written = ::write(STDOUT_FILENO, text.data() + offset, text.size() - offset);
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written <= 0) {
      throw std::runtime_error("failed to write subscriber event");
    }
    offset += static_cast<std::size_t>(written);
  }
}

void emit_event(
  const std::string & name, std::initializer_list<EventField> fields = {})
{
  std::ostringstream stream;
  stream << "EVENT name=" << name;
  for (const auto & [key, value] : fields) {
    stream << ' ' << key << '=' << value;
  }
  stream << '\n';

  const std::lock_guard<std::mutex> lock(output_mutex);
  write_all(stream.str());
}

std::string address_string(const void * address)
{
  std::ostringstream stream;
  stream << address;
  return stream.str();
}

std::size_t parse_depth(const std::string & text)
{
  std::size_t depth = 0U;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), depth, 10);
  if (
    text.empty() || result.ec == std::errc::result_out_of_range ||
    result.ec != std::errc{} || result.ptr != text.data() + text.size() || depth == 0U)
  {
    throw std::invalid_argument("DEPTH must be a positive unsigned decimal integer");
  }
  return depth;
}

struct PayloadValidation
{
  bool valid{true};
  std::uint64_t checksum{0U};
};

PayloadValidation validate_payload(const Message & message)
{
  PayloadValidation result;
  for (std::size_t index = 0U; index < message.data.size(); ++index) {
    const auto expected = static_cast<std::uint8_t>(
      message.sequence + static_cast<std::uint64_t>(index));
    const auto actual = message.data[index];
    result.checksum += static_cast<std::uint64_t>(actual);
    result.valid = result.valid && actual == expected;
  }
  return result;
}

class ExecutorWorker
{
public:
  explicit ExecutorWorker(agnocast::AgnocastOnlySingleThreadedExecutor & executor)
  : executor_(executor), failure_(), worker_([this]() {
        try {
          executor_.spin();
        } catch (...) {
          failure_ = std::current_exception();
        }
      })
  {
  }

  ~ExecutorWorker() {stop_noexcept();}

  ExecutorWorker(const ExecutorWorker &) = delete;
  ExecutorWorker & operator=(const ExecutorWorker &) = delete;

  void stop()
  {
    stop_noexcept();
    if (failure_) {
      std::rethrow_exception(failure_);
    }
  }

private:
  void stop_noexcept() noexcept
  {
    if (!worker_.joinable()) {
      return;
    }
    std::exception_ptr cancel_failure;
    try {
      executor_.cancel();
    } catch (...) {
      cancel_failure = std::current_exception();
    }
    worker_.join();
    if (!failure_) {
      failure_ = cancel_failure;
    }
  }

  agnocast::AgnocastOnlySingleThreadedExecutor & executor_;
  std::exception_ptr failure_;
  std::thread worker_;
};

int run_subscriber(const std::string & topic, const std::size_t depth)
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.use_clock_thread(false);
  auto node = std::make_shared<agnocast::Node>(
    "rclpy_agnocast_system_subscriber_" + std::to_string(getpid()), options);

  auto callback = [](const agnocast::ipc_shared_ptr<const Message> & message) {
      const auto validation = validate_payload(*message);
      emit_event(
        "RX",
        {{"sequence", std::to_string(message->sequence)},
          {"size", std::to_string(message->data.size())},
          {"bytes", std::to_string(message->data.size())},
          {"checksum", std::to_string(validation.checksum)},
          {"valid", validation.valid ? "true" : "false"},
          {"content_valid", validation.valid ? "true" : "false"},
          {"msg_addr", address_string(message.get())},
          {"data_addr", address_string(message->data.data())},
          {"entry_id", std::to_string(message.get_entry_id())}});
    };
  auto subscription = node->create_subscription<Message>(topic, depth, std::move(callback));
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(node);
  ExecutorWorker worker(executor);

  emit_event(
    "READY",
    {{"role", "subscriber"},
      {"pid", std::to_string(getpid())},
      {"topic", topic},
      {"depth", std::to_string(depth)}});

  std::string command;
  while (std::getline(std::cin, command)) {
    if (command == "connections") {
      const auto count = subscription->get_publisher_count();
      emit_event(
        "CONNECTED",
        {{"count", std::to_string(count)}, {"connected", count > 0U ? "true" : "false"}});
    } else if (command == "quit") {
      break;
    } else {
      emit_event("ERROR", {{"reason", "unknown_command"}});
    }
  }

  worker.stop();
  executor.remove_node(node);
  subscription.reset();
  node.reset();
  return 0;
}

}  // namespace

/**
 * @brief Run the typed SharedBuffer subscriber used by opt-in system tests.
 *
 * @param[in] argc Argument count; exactly a topic and positive depth are required.
 * @param[in] argv Argument vector retained by the process for the duration of this call.
 * @return Zero after graceful shutdown, one for a runtime failure, or two for invalid arguments.
 */
int main(int argc, char ** argv)
{
  if (argc != 3 || argv[1] == nullptr || argv[2] == nullptr || argv[1][0] == '\0') {
    std::cerr << "Usage: " << (argc > 0 ? argv[0] : "shared_buffer_subscriber")
              << " TOPIC DEPTH\n";
    return 2;
  }

  std::size_t depth = 0U;
  try {
    depth = parse_depth(argv[2]);
  } catch (const std::exception & error) {
    std::cerr << "shared_buffer_subscriber: " << error.what() << '\n';
    return 2;
  }

  bool initialized = false;
  try {
    agnocast::init(0, nullptr);
    initialized = true;
    const int status = run_subscriber(argv[1], depth);
    agnocast::shutdown();
    return status;
  } catch (const std::exception & error) {
    if (initialized) {
      agnocast::shutdown();
    }
    std::cerr << "shared_buffer_subscriber: " << error.what() << '\n';
    return 1;
  }
}
