#include "phase2_agnocast_probe/probe.hpp"

#include <charconv>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <system_error>

namespace phase2_agnocast_probe
{
namespace
{

/** @brief Derive one portable, non-negative element from a sequence and index. */
std::int64_t expected_value(std::int64_t sequence, std::size_t index)
{
  constexpr auto mask = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
  const auto value = (static_cast<std::uint64_t>(sequence) + index) & mask;
  return static_cast<std::int64_t>(value);
}

}  // namespace

void emit_event(
  const std::string & name,
  std::initializer_list<std::pair<std::string, std::string>> fields)
{
  static std::mutex output_mutex;
  const std::lock_guard<std::mutex> lock(output_mutex);

  std::cout << "EVENT name=" << name;
  for (const auto & [key, value] : fields) {
    std::cout << ' ' << key << '=' << value;
  }
  std::cout << std::endl;
}

std::uint64_t fill_payload(
  Message & message, std::size_t payload_bytes, std::int64_t sequence)
{
  if (payload_bytes % sizeof(std::int64_t) != 0U) {
    throw std::invalid_argument("payload_bytes must be divisible by sizeof(int64_t)");
  }

  message.id = sequence;
  message.data.resize(payload_bytes / sizeof(std::int64_t));

  std::uint64_t checksum = 0U;
  for (std::size_t index = 0; index < message.data.size(); ++index) {
    const auto value = expected_value(sequence, index);
    message.data[index] = value;
    checksum += static_cast<std::uint64_t>(value);
  }
  return checksum;
}

ValidationResult validate_payload(const Message & message)
{
  bool valid = true;
  std::string reason = "ok";
  std::uint64_t checksum = 0U;

  for (std::size_t index = 0; index < message.data.size(); ++index) {
    const auto actual = message.data[index];
    const auto expected = expected_value(message.id, index);
    checksum += static_cast<std::uint64_t>(actual);
    if (valid && actual != expected) {
      valid = false;
      reason = "value_mismatch:index=" + std::to_string(index) +
        ",expected=" + std::to_string(expected) + ",actual=" + std::to_string(actual);
    }
  }

  return ValidationResult{valid, checksum, reason};
}

std::size_t parse_size(const std::string & text, const std::string & name)
{
  std::size_t value = 0U;
  const char * const begin = text.data();
  const char * const end = begin + text.size();
  const auto result = std::from_chars(begin, end, value, 10);

  if (result.ec == std::errc::result_out_of_range) {
    throw std::out_of_range(name + " is outside the size_t range");
  }
  if (text.empty() || result.ec != std::errc{} || result.ptr != end) {
    throw std::invalid_argument(name + " must be an unsigned decimal integer");
  }
  return value;
}

std::int64_t parse_int64(const std::string & text, const std::string & name)
{
  std::int64_t value = 0;
  const char * const begin = text.data();
  const char * const end = begin + text.size();
  const auto result = std::from_chars(begin, end, value, 10);

  if (result.ec == std::errc::result_out_of_range) {
    throw std::out_of_range(name + " is outside the int64_t range");
  }
  if (text.empty() || result.ec != std::errc{} || result.ptr != end) {
    throw std::invalid_argument(name + " must be a signed decimal integer");
  }
  return value;
}

}  // namespace phase2_agnocast_probe
