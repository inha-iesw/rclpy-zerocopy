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
}  // namespace g3_import_failure
