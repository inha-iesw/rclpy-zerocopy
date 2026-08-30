#include "phase2_agnocast_probe/probe.hpp"

#include "agnocast/agnocast.hpp"

#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace phase2_agnocast_probe
{

int run_publisher(const PublisherConfig & config)
{
  agnocast::Node node("phase2_probe_publisher");
  auto publisher = node.create_publisher<Message>(config.topic, config.depth);

  emit_event(
    "READY",
    {{"role", "publisher"},
     {"pid", std::to_string(getpid())},
     {"topic", config.topic},
     {"depth", std::to_string(config.depth)}});

  std::int64_t sequence = config.start_sequence;
  std::string command;
  while (std::getline(std::cin, command)) {
    if (command == "connections") {
      emit_event(
        "CONNECTIONS",
        {{"count", std::to_string(publisher->get_subscription_count())},
         {"intra", std::to_string(publisher->get_intra_subscription_count())}});
      continue;
    }

    if (command == "publish") {
      auto message = publisher->borrow_loaned_message();
      const std::uint64_t checksum = fill_payload(*message, config.payload_bytes, sequence);
      const auto message_address = reinterpret_cast<std::uintptr_t>(message.get());
      const auto data_address = reinterpret_cast<std::uintptr_t>(message->data.data());

      publisher->publish(std::move(message));

      std::ostringstream message_address_text;
      message_address_text << "0x" << std::hex << message_address;
      std::ostringstream data_address_text;
      data_address_text << "0x" << std::hex << data_address;
      emit_event(
        "PUB",
        {{"sequence", std::to_string(sequence)},
         {"bytes", std::to_string(config.payload_bytes)},
         {"msg_addr", message_address_text.str()},
         {"data_addr", data_address_text.str()},
         {"checksum", std::to_string(checksum)}});

      sequence = sequence == std::numeric_limits<std::int64_t>::max() ?
        std::numeric_limits<std::int64_t>::min() : sequence + 1;
      continue;
    }

    if (command == "quit") {
      return 0;
    }

    emit_event("COMMAND_ERROR", {{"command", command}});
  }

  return 0;
}

}  // namespace phase2_agnocast_probe
