#include "import_failure.hpp"

#include <cuda.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

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
  ReceiveError, ReceiveEof, ReceiptRetain, ReceiptPush, ReceiptImport,
  ReceiptGranularity, ReceiptReserve, ReceiptMap, ReceiptAccess, ReceiptBindCreate, ReceiptBindWait
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
      if (std::strcmp(value, "receipt_retain") == 0) {return Failure::ReceiptRetain;}
      if (std::strcmp(value, "receipt_push") == 0) {return Failure::ReceiptPush;}
      if (std::strcmp(value, "receipt_import") == 0) {return Failure::ReceiptImport;}
      if (std::strcmp(value, "receipt_granularity") == 0) {return Failure::ReceiptGranularity;}
      if (std::strcmp(value, "receipt_reserve") == 0) {return Failure::ReceiptReserve;}
      if (std::strcmp(value, "receipt_map") == 0) {return Failure::ReceiptMap;}
      if (std::strcmp(value, "receipt_access") == 0) {return Failure::ReceiptAccess;}
      if (std::strcmp(value, "receipt_bind_create") == 0) {return Failure::ReceiptBindCreate;}
      if (std::strcmp(value, "receipt_bind_wait") == 0) {return Failure::ReceiptBindWait;}
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

thread_local bool mapping = false;
thread_local bool received = false;
thread_local bool failed_map = false;
thread_local int received_fd = -1;
thread_local CUevent completion_event = nullptr;
thread_local CUcontext context_before = nullptr;
thread_local bool context_known = false;
thread_local g3_import_failure::ReceiptStats receipt;
std::atomic<bool> observing_lifetime{false};
std::atomic<unsigned> lifetime_retains{0};
std::atomic<unsigned> lifetime_pushes{0};
std::atomic<unsigned> lifetime_pops{0};
std::atomic<unsigned> lifetime_releases{0};

bool receipt_failure()
{
  const auto selected = selected_failure();
  return selected == Failure::ReceiptRetain || selected == Failure::ReceiptPush ||
         selected == Failure::ReceiptImport || selected == Failure::ReceiptGranularity ||
         selected == Failure::ReceiptReserve || selected == Failure::ReceiptMap ||
         selected == Failure::ReceiptAccess || selected == Failure::ReceiptBindCreate ||
         selected == Failure::ReceiptBindWait;
}

bool fail_receipt(Failure operation)
{
  if (!received || failed_map || selected_failure() != operation || failure_count.load() >= 3) {
    return false;
  }
  failed_map = true;
  failure_count.fetch_add(1);
  return true;
}

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

void begin_map()
{
  if (!receipt_failure()) {return;}
  receipt = {};
  received = false;
  failed_map = false;
  received_fd = -1;
  completion_event = nullptr;
  context_known = cuCtxGetCurrent(&context_before) == CUDA_SUCCESS;
  mapping = true;
}

void end_map()
{
  if (!mapping) {return;}
  received = false;
  mapping = false;
  CUcontext after = nullptr;
  receipt.context_restored = context_known && cuCtxGetCurrent(&after) == CUDA_SUCCESS &&
    after == context_before;
  const int saved_errno = errno;
  errno = 0;
  receipt.closed = received_fd >= 0 && fcntl(received_fd, F_GETFD) == -1 && errno == EBADF;
  errno = saved_errno;
}

ReceiptStats receipt_stats()
{
  auto result = receipt;
  result.lifetime_retains = lifetime_retains.load();
  result.lifetime_pushes = lifetime_pushes.load();
  result.lifetime_pops = lifetime_pops.load();
  result.lifetime_releases = lifetime_releases.load();
  return result;
}
}  // namespace g3_import_failure

extern "C" int connect(int socket, const sockaddr * address, socklen_t length)
{
  static const auto real = original<decltype(&connect)>("connect");
  if (!real) {errno = ENOSYS; return -1;}
  const int result = real(socket, address, length);
  const auto selected = selected_failure();
  constexpr char prefix[] = "/tmp/cuda_memory_pool_";
  if (result == 0 && (selected == Failure::ReceiveError || selected == Failure::ReceiveEof ||
    (mapping && receipt_failure())) &&
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
  const bool request = request_socket.compare_exchange_strong(expected, -1);
  const unsigned attempt = request ? receive_calls.fetch_add(1) : 0;
  if (request && !receipt_failure() && attempt < 3) {
    failure_count.fetch_add(1);
    if (selected_failure() == Failure::ReceiveEof) {return 0;}
    errno = EIO;
    return -1;
  }
  static const auto real = original<decltype(&recvmsg)>("recvmsg");
  if (!real) {errno = ENOSYS; return -1;}
  const auto result = real(socket, message, flags);
  if (request && mapping && result > 0) {
    const int saved_errno = errno;
    auto * header = CMSG_FIRSTHDR(message);
    if (!(message->msg_flags & MSG_CTRUNC) && header &&
      header->cmsg_level == SOL_SOCKET && header->cmsg_type == SCM_RIGHTS &&
      header->cmsg_len == CMSG_LEN(sizeof(int)) && !CMSG_NXTHDR(message, header))
    {
      std::memcpy(&received_fd, CMSG_DATA(header), sizeof(received_fd));
      receipt.fds = 1;
      receipt.cloexec = fcntl(received_fd, F_GETFD) == FD_CLOEXEC;
      received = true;
      observing_lifetime.store(true);
    }
    errno = saved_errno;
  }
  return result;
}

extern "C" int close(int fd)
{
  static const auto real = original<decltype(&close)>("close");
  if (!real) {errno = ENOSYS; return -1;}
  const int result = real(fd);
  if (received && fd == received_fd && receipt.fd_closes == 0 && result == 0) {
    ++receipt.fd_closes;
  }
  return result;
}

// These calls operate on actual peer-imported VMM and IPC-event resources.
// Normal driver calls pass through, including every call when the environment
// selector is absent. Cleanup injections arm only at the importer event-open
// stage; receipt injections arm on FD receipt. Startup must not consume either.
extern "C" CUresult CUDAAPI cuIpcOpenEventHandle(CUevent * event, CUipcEventHandle handle)
{
  if (received) {++receipt.event_opens;}
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
  const auto result = real ? real(pointer, size) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {++receipt.unmaps;}
  return result;
}

extern "C" CUresult CUDAAPI cuMemAddressFree(CUdeviceptr pointer, std::size_t size)
{
  if (fail_cleanup(selected_failure() == Failure::AddressFree)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemAddressFree)>("cuMemAddressFree");
  const auto result = real ? real(pointer, size) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {++receipt.address_frees;}
  return result;
}

extern "C" CUresult CUDAAPI cuMemRelease(CUmemGenericAllocationHandle handle)
{
  if (fail_cleanup(selected_failure() == Failure::Release)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemRelease)>("cuMemRelease");
  const auto result = real ? real(handle) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {++receipt.allocation_releases;}
  return result;
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
  const auto result = real ? real(event) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {
    if (event == imported_event.load()) {++receipt.ready_destroys;}
    if (event == completion_event) {++receipt.completion_destroys;}
  }
  return result;
}

extern "C" CUresult CUDAAPI cuEventCreate(CUevent * event, unsigned flags)
{
  if (received) {++receipt.completion_creates;}
  if (fail_receipt(Failure::ReceiptBindCreate)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuEventCreate)>("cuEventCreate");
  const auto result = real ? real(event, flags) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {
    ++receipt.completion_created;
    completion_event = *event;
  }
  return result;
}

extern "C" CUresult CUDAAPI cuStreamWaitEvent(CUstream stream, CUevent event, unsigned flags)
{
  if (received) {++receipt.ready_waits;}
  if (fail_receipt(Failure::ReceiptBindWait)) {return CUDA_ERROR_INVALID_HANDLE;}
  static const auto real = original<decltype(&cuStreamWaitEvent)>("cuStreamWaitEvent");
  return real ? real(stream, event, flags) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuCtxPopCurrent(CUcontext * context)
{
  const auto selected = selected_failure();
  if (fail_cleanup(selected == Failure::ImportPop || selected == Failure::PartialPop)) {
    return CUDA_ERROR_UNKNOWN;
  }
  static const auto real = original<decltype(&cuCtxPopCurrent)>("cuCtxPopCurrent_v2");
  const auto result = real ? real(context) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {++receipt.pops;}
  if (observing_lifetime.load() && result == CUDA_SUCCESS) {lifetime_pops.fetch_add(1);}
  return result;
}

extern "C" CUresult CUDAAPI cuMemImportFromShareableHandle(
  CUmemGenericAllocationHandle * handle, void * os_handle, CUmemAllocationHandleType type)
{
  if (selected_failure() != Failure::None) {import_calls.fetch_add(1);}
  if (received) {++receipt.imports;}
  if (fail_receipt(Failure::ReceiptImport)) {return CUDA_ERROR_INVALID_HANDLE;}
  static const auto real = original<decltype(&cuMemImportFromShareableHandle)>(
    "cuMemImportFromShareableHandle");
  const auto result = real ? real(handle, os_handle, type) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {++receipt.imported;}
  return result;
}

extern "C" CUresult CUDAAPI cuMemGetAllocationGranularity(
  std::size_t * granularity, const CUmemAllocationProp * properties,
  CUmemAllocationGranularity_flags option)
{
  if (received) {++receipt.granularities;}
  if (fail_receipt(Failure::ReceiptGranularity)) {return CUDA_ERROR_INVALID_VALUE;}
  static const auto real = original<decltype(&cuMemGetAllocationGranularity)>(
    "cuMemGetAllocationGranularity");
  return real ? real(granularity, properties, option) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemAddressReserve(
  CUdeviceptr * pointer, std::size_t size, std::size_t alignment,
  CUdeviceptr address, unsigned long long flags)
{
  if (received) {++receipt.reserves;}
  if (fail_receipt(Failure::ReceiptReserve)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuMemAddressReserve)>("cuMemAddressReserve");
  return real ? real(pointer, size, alignment, address, flags) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemMap(
  CUdeviceptr pointer, std::size_t size, std::size_t offset,
  CUmemGenericAllocationHandle handle, unsigned long long flags)
{
  if (received) {++receipt.maps;}
  if (fail_receipt(Failure::ReceiptMap)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuMemMap)>("cuMemMap");
  return real ? real(pointer, size, offset, handle, flags) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemSetAccess(
  CUdeviceptr pointer, std::size_t size, const CUmemAccessDesc * descriptor, std::size_t count)
{
  if (received) {++receipt.accesses;}
  if (fail_receipt(Failure::ReceiptAccess)) {return CUDA_ERROR_INVALID_VALUE;}
  static const auto real = original<decltype(&cuMemSetAccess)>("cuMemSetAccess");
  return real ? real(pointer, size, descriptor, count) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuDevicePrimaryCtxRetain(CUcontext * context, CUdevice device)
{
  if (selected_failure() != Failure::None) {retain_calls.fetch_add(1);}
  if (received) {++receipt.retains;}
  if (fail_receipt(Failure::ReceiptRetain)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuDevicePrimaryCtxRetain)>("cuDevicePrimaryCtxRetain");
  const auto result = real ? real(context, device) : CUDA_ERROR_UNKNOWN;
  if (observing_lifetime.load() && result == CUDA_SUCCESS) {lifetime_retains.fetch_add(1);}
  return result;
}

extern "C" CUresult CUDAAPI cuCtxPushCurrent(CUcontext context)
{
  if (fail_receipt(Failure::ReceiptPush)) {return CUDA_ERROR_INVALID_CONTEXT;}
  static const auto real = original<decltype(&cuCtxPushCurrent)>("cuCtxPushCurrent_v2");
  const auto result = real ? real(context) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {++receipt.pushes;}
  if (observing_lifetime.load() && result == CUDA_SUCCESS) {lifetime_pushes.fetch_add(1);}
  return result;
}

extern "C" CUresult CUDAAPI cuDevicePrimaryCtxRelease(CUdevice device)
{
  static const auto real = original<decltype(&cuDevicePrimaryCtxRelease)>(
    "cuDevicePrimaryCtxRelease_v2");
  const auto result = real ? real(device) : CUDA_ERROR_UNKNOWN;
  if (received && result == CUDA_SUCCESS) {++receipt.releases;}
  if (observing_lifetime.load() && result == CUDA_SUCCESS) {lifetime_releases.fetch_add(1);}
  return result;
}
