#pragma once

namespace g3_fd_reply_failure
{
/** @brief Counts of selected single-FD replies from core pool Unix sockets. */
struct Stats
{
  unsigned attempts;  ///< Selected calls, including later pass-through replies.
  unsigned injections;  ///< Calls changed by the selected failure mode.
  unsigned malformed_sent;  ///< Malformed replies whose sendmsg returned one byte; zero for send_error.
};

/**
 * @brief Observe the producer reply injection counters.
 *
 * G3_FD_REPLY_FAILURE=two_fds|truncated|send_error changes only the first three
 * selected core replies; subsequent replies pass through normally.
 * @return A snapshot of the counters for the active selector; zero when disabled.
 */
Stats stats();
}  // namespace g3_fd_reply_failure
