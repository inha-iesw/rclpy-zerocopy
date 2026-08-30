#include "phase2_agnocast_probe/probe.hpp"

#include "agnocast/agnocast.hpp"

#include <cstdint>
#include <new>
#include <string>
#include <utility>

namespace phase2_agnocast_probe
{

int run_exhaust(const ExhaustConfig & config)
{
  constexpr std::size_t recovery_bytes = 1024;
  agnocast::Node node("phase2_probe_exhaust");
  auto publisher = node.create_publisher<Message>(config.topic, 1);

  bool reserve_succeeded = false;
  try {
    auto message = publisher->borrow_loaned_message();
    message->data.reserve(config.reserve_bytes / sizeof(std::int64_t));
    reserve_succeeded = true;
  } catch (const std::bad_alloc &) {
  }

  if (reserve_succeeded) {
    emit_event("RESERVE_SUCCEEDED", {{"reserve_bytes", std::to_string(config.reserve_bytes)}});
  } else {
    emit_event("BAD_ALLOC", {{"reserve_bytes", std::to_string(config.reserve_bytes)}});
  }

  auto recovery = publisher->borrow_loaned_message();
  const std::uint64_t checksum = fill_payload(*recovery, recovery_bytes, 0);
  publisher->publish(std::move(recovery));
  emit_event(
    "RECOVERY_OK",
    {{"bytes", std::to_string(recovery_bytes)}, {"checksum", std::to_string(checksum)}});

  return reserve_succeeded ? 1 : 0;
}

}  // namespace phase2_agnocast_probe
