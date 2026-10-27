#include "import_failure.hpp"

#include <cuda.h>
#include <dlfcn.h>

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <cstring>

namespace
{
enum class Failure
{
  None, Unmap, AddressFree, Release, EventDestroy, ImportPop, PartialUnmap, PartialPop
};

Failure selected_failure()
{
  static const Failure selected = []() {
      const char * value = std::getenv("G3_IMPORT_FAILURE");
      if (!value) {return Failure::None;}
      if (std::strcmp(value, "unmap") == 0) {return Failure::Unmap;}
      if (std::strcmp(value, "address_free") == 0) {return Failure::AddressFree;}
      if (std::strcmp(value, "release") == 0) {return Failure::Release;}
      if (std::strcmp(value, "event_destroy") == 0) {return Failure::EventDestroy;}
      if (std::strcmp(value, "import_pop") == 0) {return Failure::ImportPop;}
      if (std::strcmp(value, "partial_unmap") == 0) {return Failure::PartialUnmap;}
      if (std::strcmp(value, "partial_pop") == 0) {return Failure::PartialPop;}
      return Failure::None;
    }();
  return selected;
}

std::atomic<bool> armed{false};
std::atomic<bool> injected{false};
std::atomic<CUevent> imported_event{nullptr};
std::atomic<unsigned> failure_count{0};
std::atomic<unsigned> cleanup_calls{0};
std::atomic<unsigned> import_calls{0};
std::atomic<unsigned> retain_calls{0};

template<typename Function>
Function original(const char * symbol)
{
  return reinterpret_cast<Function>(dlsym(RTLD_NEXT, symbol));
}

bool fail_cleanup(bool selected)
{
  if (!selected || !armed.load()) {return false;}
  cleanup_calls.fetch_add(1);
  if (!injected.exchange(true)) {
    failure_count.fetch_add(1);
    return true;
  }
  return false;
}
}  // namespace

namespace g3_import_failure
{
Stats stats()
{
  return {failure_count.load(), cleanup_calls.load(), import_calls.load(), retain_calls.load()};
}
}  // namespace g3_import_failure

// These calls operate on actual peer-imported VMM and IPC-event resources.
// Normal driver calls pass through, including every call when the environment
// selector is absent. Arm only after reaching the importer event-open stage;
// CUDA runtime and peer startup cleanup must not consume the injected failure.
extern "C" CUresult CUDAAPI cuIpcOpenEventHandle(CUevent * event, CUipcEventHandle handle)
{
  const auto selected = selected_failure();
  if (selected == Failure::PartialUnmap || selected == Failure::PartialPop) {
    armed.store(true);
    return CUDA_ERROR_INVALID_HANDLE;
  }
  static const auto real = original<decltype(&cuIpcOpenEventHandle)>("cuIpcOpenEventHandle");
  const CUresult result = real ? real(event, handle) : CUDA_ERROR_UNKNOWN;
  if (result == CUDA_SUCCESS && selected != Failure::None) {
    imported_event.store(*event);
    armed.store(true);
  }
  return result;
}

extern "C" CUresult CUDAAPI cuMemUnmap(CUdeviceptr pointer, std::size_t size)
{
  const auto selected = selected_failure();
  if (fail_cleanup(selected == Failure::Unmap || selected == Failure::PartialUnmap)) {
    return CUDA_ERROR_UNKNOWN;
  }
  static const auto real = original<decltype(&cuMemUnmap)>("cuMemUnmap");
  return real ? real(pointer, size) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemAddressFree(CUdeviceptr pointer, std::size_t size)
{
  if (fail_cleanup(selected_failure() == Failure::AddressFree)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemAddressFree)>("cuMemAddressFree");
  return real ? real(pointer, size) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemRelease(CUmemGenericAllocationHandle handle)
{
  if (fail_cleanup(selected_failure() == Failure::Release)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemRelease)>("cuMemRelease");
  return real ? real(handle) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuEventDestroy(CUevent event)
{
  // Consumer completion events are distinct owners and must still be cleaned
  // normally before this imported ready event reaches resource destruction.
  if (fail_cleanup(selected_failure() == Failure::EventDestroy &&
    event == imported_event.load()))
  {
    return CUDA_ERROR_UNKNOWN;
  }
  static const auto real = original<decltype(&cuEventDestroy)>("cuEventDestroy_v2");
  return real ? real(event) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuCtxPopCurrent(CUcontext * context)
{
  const auto selected = selected_failure();
  if (fail_cleanup(selected == Failure::ImportPop || selected == Failure::PartialPop)) {
    return CUDA_ERROR_UNKNOWN;
  }
  static const auto real = original<decltype(&cuCtxPopCurrent)>("cuCtxPopCurrent_v2");
  return real ? real(context) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemImportFromShareableHandle(
  CUmemGenericAllocationHandle * handle, void * os_handle, CUmemAllocationHandleType type)
{
  if (selected_failure() != Failure::None) {import_calls.fetch_add(1);}
  static const auto real = original<decltype(&cuMemImportFromShareableHandle)>(
    "cuMemImportFromShareableHandle");
  return real ? real(handle, os_handle, type) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuDevicePrimaryCtxRetain(CUcontext * context, CUdevice device)
{
  if (selected_failure() != Failure::None) {retain_calls.fetch_add(1);}
  static const auto real = original<decltype(&cuDevicePrimaryCtxRetain)>("cuDevicePrimaryCtxRetain");
  return real ? real(context, device) : CUDA_ERROR_UNKNOWN;
}
