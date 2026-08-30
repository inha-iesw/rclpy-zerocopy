#include "phase2_agnocast_probe/probe.hpp"

#include "agnocast/agnocast.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

/** @brief Print the complete, strict command-line contract. */
void print_usage(std::ostream & output, const std::string & program)
{
  output << "Usage:\n"
         << "  " << program
         << " publisher TOPIC DEPTH PAYLOAD_BYTES START_SEQUENCE\n"
         << "  " << program << " subscriber TOPIC DEPTH HOLD_FIRST_N\n"
         << "  " << program << " exhaust TOPIC RESERVE_BYTES\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string program = argc > 0 && argv[0] != nullptr ? argv[0] : "phase2_probe";
  std::function<int()> action;

  try {
    if (argc < 2 || argv[1] == nullptr) {
      throw std::invalid_argument("a mode is required");
    }

    const std::string mode = argv[1];
    if (mode == "publisher") {
      if (argc != 6) {
        throw std::invalid_argument("publisher requires exactly four arguments");
      }
      phase2_agnocast_probe::PublisherConfig config{
        argv[2], phase2_agnocast_probe::parse_size(argv[3], "DEPTH"),
        phase2_agnocast_probe::parse_size(argv[4], "PAYLOAD_BYTES"),
        phase2_agnocast_probe::parse_int64(argv[5], "START_SEQUENCE")};
      if (config.topic.empty() || config.depth == 0U || config.payload_bytes == 0U) {
        throw std::invalid_argument("TOPIC, DEPTH, and PAYLOAD_BYTES must be non-empty/non-zero");
      }
      if (config.payload_bytes % sizeof(std::int64_t) != 0U) {
        throw std::invalid_argument("PAYLOAD_BYTES must be divisible by sizeof(int64_t)");
      }
      action = [config]() { return phase2_agnocast_probe::run_publisher(config); };
    } else if (mode == "subscriber") {
      if (argc != 5) {
        throw std::invalid_argument("subscriber requires exactly three arguments");
      }
      phase2_agnocast_probe::SubscriberConfig config{
        argv[2], phase2_agnocast_probe::parse_size(argv[3], "DEPTH"),
        phase2_agnocast_probe::parse_size(argv[4], "HOLD_FIRST_N")};
      if (config.topic.empty() || config.depth == 0U) {
        throw std::invalid_argument("TOPIC must be non-empty and DEPTH must be non-zero");
      }
      action = [config]() { return phase2_agnocast_probe::run_subscriber(config); };
    } else if (mode == "exhaust") {
      if (argc != 4) {
        throw std::invalid_argument("exhaust requires exactly two arguments");
      }
      phase2_agnocast_probe::ExhaustConfig config{
        argv[2], phase2_agnocast_probe::parse_size(argv[3], "RESERVE_BYTES")};
      if (config.topic.empty() || config.reserve_bytes == 0U) {
        throw std::invalid_argument("TOPIC must be non-empty and RESERVE_BYTES must be non-zero");
      }
      if (config.reserve_bytes % sizeof(std::int64_t) != 0U) {
        throw std::invalid_argument("RESERVE_BYTES must be divisible by sizeof(int64_t)");
      }
      action = [config]() { return phase2_agnocast_probe::run_exhaust(config); };
    } else {
      throw std::invalid_argument("unknown mode: " + mode);
    }
  } catch (const std::exception & error) {
    std::cerr << "phase2_probe: " << error.what() << '\n';
    print_usage(std::cerr, program);
    return 2;
  }

  bool initialized = false;
  try {
    agnocast::init(0, nullptr);
    initialized = true;
    const int status = action();
    agnocast::shutdown();
    return status;
  } catch (const std::exception & error) {
    if (initialized) {
      agnocast::shutdown();
    }
    std::cerr << "phase2_probe: " << error.what() << '\n';
    return 1;
  }
}
