#pragma once

#include <array>
#include <cstddef>

namespace g3_cleanup
{
enum class Operation : std::size_t
{
  EventDestroy, MemUnmap, MemAddressFree, MemRelease, EventQuery,
  ContextPush, ContextPop, MemSetAccess, MemCreate, PrimaryRetain, Count
};

using Counts = std::array<unsigned, static_cast<std::size_t>(Operation::Count)>;

// Each executable invocation selects one case; failures must never persist
// across independent cases in a shared CUDA process epoch.
void arm(Operation operation, unsigned occurrence, bool rollback, bool pop_mismatch);
Counts counts();
unsigned injections(Operation operation);
unsigned count(const Counts & snapshot, Operation operation);
void record_owned_names();
std::size_t owned_metadata_count();
std::size_t owned_socket_count();
bool unlink_owned_names();
}  // namespace g3_cleanup
