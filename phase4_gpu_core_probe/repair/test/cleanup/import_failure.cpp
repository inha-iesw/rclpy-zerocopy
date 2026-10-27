#include "import_failure.hpp"

#include <cuda.h>
#include <dlfcn.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <cstring>

namespace
{
enum class Failure
{
  None, Unmap, AddressFree, Release, EventDestroy, ImportPop, PartialUnmap, PartialPop,
  ReceiveError, ReceiveEof
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
      if (std::strcmp(value, "recv_error") == 0) {return Failure::ReceiveError;}
      if (std::strcmp(value, "recv_eof") == 0) {return Failure::ReceiveEof;}
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
std::atomic<unsigned> connection_calls{0};
std::atomic<unsigned> receive_calls{0};
std::atomic<int> request_socket{-1};

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
  return {failure_count.load(), cleanup_calls.load(), import_calls.load(), retain_calls.load(),
    connection_calls.load(), receive_calls.load()};
}
}  // namespace g3_import_failure

extern "C" int connect(int socket, const sockaddr * address, socklen_t length)
{
  static const auto real = original<decltype(&connect)>("connect");
  if (!real) {errno = ENOSYS; return -1;}
  const int result = real(socket, address, length);
  const auto selected = selected_failure();
  constexpr char prefix[] = "/tmp/cuda_memory_pool_";
  if (result == 0 && (selected == Failure::ReceiveError || selected == Failure::ReceiveEof) &&
    address->sa_family == AF_UNIX && length >= sizeof(sockaddr_un) &&
    std::strncmp(reinterpret_cast<const sockaddr_un *>(address)->sun_path,
    prefix, sizeof(prefix) - 1) == 0)
  {
    connection_calls.fetch_add(1);
    request_socket.store(socket);
  }
  return result;
}

extern "C" ssize_t recvmsg(int socket, msghdr * message, int flags)
{
  // Inject before receiving SCM_RIGHTS; closing this connected socket must
  // return all request FDs without charging an imported CUDA resource.
  int expected = socket;
  if (request_socket.compare_exchange_strong(expected, -1) && receive_calls.fetch_add(1) < 3) {
    failure_count.fetch_add(1);
    if (selected_failure() == Failure::ReceiveEof) {return 0;}
    errno = EIO;
    return -1;
  }
  static const auto real = original<decltype(&recvmsg)>("recvmsg");
  if (!real) {errno = ENOSYS; return -1;}
  return real(socket, message, flags);
}

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
