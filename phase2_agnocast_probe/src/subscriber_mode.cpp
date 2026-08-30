#include "phase2_agnocast_probe/probe.hpp"

#include "agnocast/agnocast.hpp"

#include <unistd.h>

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace phase2_agnocast_probe
{
namespace
{

using HeldMessage = agnocast::ipc_shared_ptr<const Message>;

struct SubscriberState
{
  std::mutex mutex;
  std::vector<HeldMessage> held_messages;
  bool holding_enabled{true};
};

class ExecutorWorker
{
public:
  explicit ExecutorWorker(agnocast::AgnocastOnlySingleThreadedExecutor & executor)
  : executor_(executor), worker_([this]() {
      try {
        executor_.spin();
      } catch (...) {
        error_ = std::current_exception();
      }
    })
  {
  }

  ~ExecutorWorker() { stop(); }

  ExecutorWorker(const ExecutorWorker &) = delete;
  ExecutorWorker & operator=(const ExecutorWorker &) = delete;

  void stop()
  {
    executor_.cancel();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void rethrow_if_failed() const
  {
    if (error_) {
      std::rethrow_exception(error_);
    }
  }

private:
  agnocast::AgnocastOnlySingleThreadedExecutor & executor_;
  std::thread worker_;
  std::exception_ptr error_;
};

std::string address_string(const void * address)
{
  std::ostringstream stream;
  stream << address;
  return stream.str();
}

std::size_t release_held(SubscriberState & state)
{
  std::vector<HeldMessage> released_messages;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.holding_enabled = false;
    released_messages.swap(state.held_messages);
  }

  const std::size_t released_count = released_messages.size();
  released_messages.clear();
  return released_count;
}

void check_held(SubscriberState & state)
{
  std::vector<HeldMessage> snapshot;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    snapshot = state.held_messages;
  }

  for (const HeldMessage & message : snapshot) {
    const ValidationResult validation = validate_payload(*message);
    emit_event(
      "HELD_CHECK",
      {{"sequence", std::to_string(message->id)},
       {"bytes", std::to_string(message->data.size() * sizeof(std::int64_t))},
       {"msg_addr", address_string(message.get())},
       {"data_addr", address_string(message->data.data())},
       {"entry_id", std::to_string(message.get_entry_id())},
       {"checksum", std::to_string(validation.checksum)},
       {"valid", validation.valid ? "true" : "false"}});
  }
}

}  // namespace

int run_subscriber(const SubscriberConfig & config)
{
  SubscriberState state;
  auto node = std::make_shared<agnocast::Node>("phase2_agnocast_probe_subscriber");

  auto callback = [&state, hold_first_n = config.hold_first_n](
                    const agnocast::ipc_shared_ptr<const Message> & message) {
    const ValidationResult validation = validate_payload(*message);
    bool held = false;
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      if (state.holding_enabled && state.held_messages.size() < hold_first_n) {
        state.held_messages.push_back(message);
        held = true;
      }
    }

    emit_event(
      "RX",
      {{"sequence", std::to_string(message->id)},
       {"bytes", std::to_string(message->data.size() * sizeof(std::int64_t))},
       {"msg_addr", address_string(message.get())},
       {"data_addr", address_string(message->data.data())},
       {"entry_id", std::to_string(message.get_entry_id())},
       {"checksum", std::to_string(validation.checksum)},
       {"valid", validation.valid ? "true" : "false"},
       {"held", held ? "true" : "false"}});
  };

  auto subscription =
    node->create_subscription<Message>(config.topic, config.depth, std::move(callback));
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  executor.add_node(node);
  ExecutorWorker worker(executor);

  emit_event(
    "READY",
    {{"role", "subscriber"},
     {"pid", std::to_string(getpid())},
     {"topic", config.topic},
     {"depth", std::to_string(config.depth)},
     {"hold_first_n", std::to_string(config.hold_first_n)}});

  std::string command;
  while (std::getline(std::cin, command)) {
    if (command == "connections") {
      emit_event("CONNECTIONS", {{"count", std::to_string(subscription->get_publisher_count())}});
    } else if (command == "check") {
      check_held(state);
    } else if (command == "release") {
      emit_event("RELEASE", {{"count", std::to_string(release_held(state))}});
    } else if (command == "quit") {
      break;
    }
  }

  worker.stop();
  worker.rethrow_if_failed();
  executor.remove_node(node);
  release_held(state);
  subscription.reset();
  node.reset();
  return 0;
}

}  // namespace phase2_agnocast_probe
