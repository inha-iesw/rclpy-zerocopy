#pragma once

namespace g3_import_failure
{
struct Stats
{
  unsigned failures;
  unsigned cleanup_calls;
  unsigned imports;
  unsigned retains;
  unsigned connections;
  unsigned receives;
};

// failures counts the selected injection, excluding the initiating
// INVALID_HANDLE returned by partial-import cases. FD receive cases fail the
// first three connected requests, then pass through for a real import.
Stats stats();

struct ReceiptStats
{
  unsigned fds = 0;
  bool cloexec = false;
  // EBADF at map return. After a successful import, the driver may have reused
  // this FD number; require this only for rejection before a successful import.
  bool closed = false;
  bool context_restored = false;
  unsigned retains = 0;
  unsigned pushes = 0;
  unsigned pops = 0;
  unsigned releases = 0;
  unsigned imports = 0;
  unsigned lifetime_retains = 0;
  unsigned lifetime_pushes = 0;
  unsigned lifetime_pops = 0;
  unsigned lifetime_releases = 0;
  // VMM stage/event-open counters are attempts; imported and the resource
  // return counters count successful calls on this map's thread only.
  unsigned imported = 0;
  unsigned granularities = 0;
  unsigned reserves = 0;
  unsigned maps = 0;
  unsigned accesses = 0;
  unsigned unmaps = 0;
  unsigned address_frees = 0;
  unsigned allocation_releases = 0;
  unsigned event_opens = 0;
  // Observe the received FD's first successful close, before any later driver
  // call can reuse its number. Unlike closed, this remains valid after import.
  unsigned fd_closes = 0;
};

// receipt_retain/receipt_push/receipt_import fail the selected operation once
// per map for the first three well-formed FD receipts, then pass through.
// Observation is local to the map caller, starts after receipt and ends before
// checksum work. lifetime_* count successful calls on all threads from the
// first receipt onward; read them after mapper drain for a complete balance.
void begin_map();
void end_map();
ReceiptStats receipt_stats();
}  // namespace g3_import_failure
