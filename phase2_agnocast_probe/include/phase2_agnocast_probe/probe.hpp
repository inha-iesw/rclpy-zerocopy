#pragma once

#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>

namespace phase2_agnocast_probe
{

/** @brief Dynamic Agnocast sample message used by every probe mode. */
using Message = agnocast_sample_interfaces::msg::DynamicSizeArray;

/** @brief Configuration for deterministic loaned-message publication. */
struct PublisherConfig
{
  std::string topic;
  std::size_t depth;
  std::size_t payload_bytes;
  std::int64_t start_sequence;
};

/** @brief Configuration for reference-lifetime observation. */
struct SubscriberConfig
{
  std::string topic;
  std::size_t depth;
  std::size_t hold_first_n;
};

/** @brief Configuration for controlled shared-allocator exhaustion. */
struct ExhaustConfig
{
  std::string topic;
  std::size_t reserve_bytes;
};

/** @brief Result of checking a deterministic probe payload. */
struct ValidationResult
{
  bool valid;
  std::uint64_t checksum;
  std::string reason;
};

/**
 * @brief Emit one immediately flushed, machine-readable probe event.
 *
 * Each call writes one `EVENT name=<name> key=value...` line atomically with
 * respect to other threads in this process.
 *
 * @param[in] name Stable event name without whitespace.
 * @param[in] fields Key/value fields retained by the caller.
 * @return Nothing.
 */
void emit_event(
  const std::string & name,
  std::initializer_list<std::pair<std::string, std::string>> fields = {});

/**
 * @brief Directly resize and fill a loaned dynamic-array message.
 *
 * The message ID records `sequence`; element `i` receives a deterministic
 * value derived from `sequence + i` so another process can validate it.
 *
 * @param[in,out] message Loaned message retained by the caller.
 * @param[in] payload_bytes Exact payload size, divisible by `sizeof(int64_t)`.
 * @param[in] sequence Sequence stored in the message and payload pattern.
 * @return Unsigned wrapping checksum of all payload elements.
 * @throws std::invalid_argument If `payload_bytes` is not element-aligned.
 */
std::uint64_t fill_payload(
  Message & message, std::size_t payload_bytes, std::int64_t sequence);

/**
 * @brief Validate the payload pattern without copying the message data.
 *
 * @param[in] message Message borrowed by the caller for the duration of this call.
 * @return Pattern validity, observed checksum, and a stable reason string.
 */
ValidationResult validate_payload(const Message & message);

/**
 * @brief Parse a strict unsigned decimal size.
 *
 * @param[in] text Entire token to parse; signs and whitespace are rejected.
 * @param[in] name Field name included in parse errors.
 * @return Parsed size, including zero when explicitly supplied.
 * @throws std::invalid_argument If the token is not strict decimal text.
 * @throws std::out_of_range If the value cannot fit in `size_t`.
 */
std::size_t parse_size(const std::string & text, const std::string & name);

/**
 * @brief Parse a strict signed decimal 64-bit integer.
 *
 * @param[in] text Entire token to parse; whitespace and a leading plus are rejected.
 * @param[in] name Field name included in parse errors.
 * @return Parsed signed value.
 * @throws std::invalid_argument If the token is not strict decimal text.
 * @throws std::out_of_range If the value cannot fit in `int64_t`.
 */
std::int64_t parse_int64(const std::string & text, const std::string & name);

/**
 * @brief Run the publisher characterization mode until shutdown.
 *
 * @param[in] config Validated publisher configuration retained by the caller.
 * @return Process exit status.
 * @pre `agnocast::init()` has completed successfully.
 */
int run_publisher(const PublisherConfig & config);

/**
 * @brief Run the subscriber lifetime-characterization mode until shutdown.
 *
 * @param[in] config Validated subscriber configuration retained by the caller.
 * @return Process exit status.
 * @pre `agnocast::init()` has completed successfully.
 */
int run_subscriber(const SubscriberConfig & config);

/**
 * @brief Run controlled shared-allocator exhaustion and recovery checks.
 *
 * @param[in] config Validated exhaustion configuration retained by the caller.
 * @return Process exit status; expected allocation failure is reported as data.
 * @pre `agnocast::init()` has completed successfully.
 */
int run_exhaust(const ExhaustConfig & config);

}  // namespace phase2_agnocast_probe
