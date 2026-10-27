#pragma once

namespace g3_import_failure
{
struct Stats
{
  unsigned failures;
  unsigned cleanup_calls;
  unsigned imports;
  unsigned retains;
};

// failures counts the selected cleanup injection, excluding the initiating
// INVALID_HANDLE returned by partial-import cases.
Stats stats();
}  // namespace g3_import_failure
