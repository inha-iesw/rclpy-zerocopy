#include "producer_failure.hpp"

#include <cuda.h>
#include <dlfcn.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

namespace
{
using g3_cleanup::Operation;
constexpr auto operation_count = static_cast<std::size_t>(Operation::Count);
std::array<std::atomic<unsigned>, operation_count> observed{};
std::array<std::atomic<unsigned>, operation_count> injected{};
Operation target = Operation::Count;
unsigned target_call = 0;
unsigned access_call = 0;
bool return_wrong_context = false;
bool collect_names = false;
std::vector<std::string> metadata_names;
std::vector<std::string> socket_names;

unsigned observe(Operation operation)
{
  return observed[static_cast<std::size_t>(operation)].fetch_add(1) + 1;
}

bool fail(Operation operation)
{
  const auto index = static_cast<std::size_t>(operation);
  const auto current = observe(operation);
  if ((operation == target && current == target_call) ||
    (operation == Operation::MemSetAccess && current == access_call))
  {
    injected[index].fetch_add(1);
    return true;
  }
  return false;
}

template<typename Function>
Function original(const char * symbol)
{
  return reinterpret_cast<Function>(dlsym(RTLD_NEXT, symbol));
}

bool pool_socket_address(const sockaddr * address, socklen_t size)
{
  constexpr char prefix[] = "/tmp/cuda_memory_pool_";
  if (size < offsetof(sockaddr_un, sun_path) + sizeof(prefix) - 1 ||
    address->sa_family != AF_UNIX) {return false;}
  const auto * local = reinterpret_cast<const sockaddr_un *>(address);
  return std::strncmp(local->sun_path, prefix, sizeof(prefix) - 1) == 0;
}
}  // namespace

namespace g3_cleanup
{
void arm(Operation operation, unsigned occurrence, bool rollback, bool pop_mismatch)
{
  target = operation;
  target_call = observed[static_cast<std::size_t>(operation)].load() + occurrence;
  access_call = rollback ? observed[static_cast<std::size_t>(Operation::MemSetAccess)].load() +
    1 : 0;
  return_wrong_context = pop_mismatch;
}

Counts counts()
{
  Counts result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = observed[index].load();
  }
  return result;
}

unsigned injections(Operation operation)
{
  return injected[static_cast<std::size_t>(operation)].load();
}

unsigned count(const Counts & snapshot, Operation operation)
{
  return snapshot[static_cast<std::size_t>(operation)];
}

void record_owned_names() {collect_names = true;}
std::size_t owned_metadata_count() {return metadata_names.size();}
std::size_t owned_socket_count() {return socket_names.size();}

bool owned_names_absent()
{
  for (const auto & name : metadata_names) {
    const int fd = shm_open(name.c_str(), O_RDONLY, 0);
    if (fd >= 0) {close(fd); return false;}
    if (errno != ENOENT) {return false;}
  }
  for (const auto & name : socket_names) {
    struct stat status{};
    if (lstat(name.c_str(), &status) == 0 || errno != ENOENT) {return false;}
  }
  return true;
}

bool unlink_owned_names()
{
  bool success = true;
  for (const auto & name : metadata_names) {
    if (shm_unlink(name.c_str()) != 0 && errno != ENOENT) {
      success = false;
    }
  }
  for (const auto & name : socket_names) {
    if (unlink(name.c_str()) != 0 && errno != ENOENT) {
      success = false;
    }
  }
  return success;
}
}  // namespace g3_cleanup

// A failed operation is skipped exactly once. A retry reaches the real driver
// and is counted, so a cleanup retry cannot be hidden by a permanent failure.
extern "C" CUresult CUDAAPI cuEventDestroy(CUevent event)
{
  if (fail(Operation::EventDestroy)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuEventDestroy)>("cuEventDestroy_v2");
  return real ? real(event) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemUnmap(CUdeviceptr pointer, std::size_t size)
{
  if (fail(Operation::MemUnmap)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemUnmap)>("cuMemUnmap");
  return real ? real(pointer, size) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemAddressFree(CUdeviceptr pointer, std::size_t size)
{
  if (fail(Operation::MemAddressFree)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemAddressFree)>("cuMemAddressFree");
  return real ? real(pointer, size) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemRelease(CUmemGenericAllocationHandle handle)
{
  if (fail(Operation::MemRelease)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemRelease)>("cuMemRelease");
  return real ? real(handle) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuEventQuery(CUevent event)
{
  if (fail(Operation::EventQuery)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuEventQuery)>("cuEventQuery");
  return real ? real(event) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuCtxPushCurrent(CUcontext context)
{
  if (fail(Operation::ContextPush)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuCtxPushCurrent)>("cuCtxPushCurrent_v2");
  return real ? real(context) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuCtxPopCurrent(CUcontext * context)
{
  const bool selected = fail(Operation::ContextPop);
  if (selected && !return_wrong_context) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuCtxPopCurrent)>("cuCtxPopCurrent_v2");
  const CUresult result = real ? real(context) : CUDA_ERROR_UNKNOWN;
  // Pop the actual stack first; only corrupt the returned identity. This
  // distinguishes a successful driver call with a broken ownership contract.
  if (selected && return_wrong_context && result == CUDA_SUCCESS) {*context = nullptr;}
  return result;
}

extern "C" CUresult CUDAAPI cuMemSetAccess(
  CUdeviceptr pointer, std::size_t size, const CUmemAccessDesc * descriptor, std::size_t count)
{
  if (fail(Operation::MemSetAccess)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemSetAccess)>("cuMemSetAccess");
  return real ? real(pointer, size, descriptor, count) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemCreate(
  CUmemGenericAllocationHandle * handle, std::size_t size,
  const CUmemAllocationProp * properties, unsigned long long flags)
{
  if (fail(Operation::MemCreate)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuMemCreate)>("cuMemCreate");
  return real ? real(handle, size, properties, flags) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemAddressReserve(
  CUdeviceptr * pointer, std::size_t size, std::size_t alignment,
  CUdeviceptr address, unsigned long long flags)
{
  if (fail(Operation::MemAddressReserve)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuMemAddressReserve)>("cuMemAddressReserve");
  return real ? real(pointer, size, alignment, address, flags) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemMap(
  CUdeviceptr pointer, std::size_t size, std::size_t offset,
  CUmemGenericAllocationHandle handle, unsigned long long flags)
{
  if (fail(Operation::MemMap)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuMemMap)>("cuMemMap");
  return real ? real(pointer, size, offset, handle, flags) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuMemExportToShareableHandle(
  void * handle, CUmemGenericAllocationHandle allocation,
  CUmemAllocationHandleType type, unsigned long long flags)
{
  if (fail(Operation::MemExport)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuMemExportToShareableHandle)>(
    "cuMemExportToShareableHandle");
  return real ? real(handle, allocation, type, flags) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuEventCreate(CUevent * event, unsigned flags)
{
  if (fail(Operation::EventCreate)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuEventCreate)>("cuEventCreate");
  return real ? real(event, flags) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuIpcGetEventHandle(CUipcEventHandle * handle, CUevent event)
{
  if (fail(Operation::IpcGetEventHandle)) {return CUDA_ERROR_UNKNOWN;}
  static const auto real = original<decltype(&cuIpcGetEventHandle)>("cuIpcGetEventHandle");
  return real ? real(handle, event) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuDevicePrimaryCtxRetain(CUcontext * context, CUdevice device)
{
  if (fail(Operation::PrimaryRetain)) {return CUDA_ERROR_OUT_OF_MEMORY;}
  static const auto real = original<decltype(&cuDevicePrimaryCtxRetain)>("cuDevicePrimaryCtxRetain");
  return real ? real(context, device) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuDevicePrimaryCtxRelease(CUdevice device)
{
  (void)observe(Operation::PrimaryRelease);
  static const auto real = original<decltype(&cuDevicePrimaryCtxRelease)>("cuDevicePrimaryCtxRelease_v2");
  return real ? real(device) : CUDA_ERROR_UNKNOWN;
}

// Record names only after successful creation by this process. The PID comes
// from the same namespace as the allocator; no global orphan scan is used.
extern "C" int shm_open(const char * name, int flags, mode_t mode)
{
  static const auto real = original<decltype(&shm_open)>("shm_open");
  if (!real) {errno = ENOSYS; return -1;}
  const int result = real(name, flags, mode);
  if (result >= 0 && collect_names && (flags & O_CREAT) && (flags & O_EXCL)) {
    const auto prefix = "/rclpy_zc_gpu_v1_" + std::to_string(getpid()) + "_";
    if (std::strncmp(name, prefix.c_str(), prefix.size()) == 0) {
      metadata_names.emplace_back(name);
    }
  }
  return result;
}

extern "C" int socket(int domain, int type, int protocol) noexcept
{
  if (collect_names && domain == AF_UNIX &&
    (type & ~SOCK_CLOEXEC) == SOCK_STREAM && fail(Operation::Socket))
  {
    errno = EMFILE;
    return -1;
  }
  static const auto real = original<decltype(&socket)>("socket");
  if (!real) {errno = ENOSYS; return -1;}
  return real(domain, type, protocol);
}

extern "C" int bind(int socket, const sockaddr * address, socklen_t size) noexcept
{
  static const auto real = original<decltype(&bind)>("bind");
  if (!real) {errno = ENOSYS; return -1;}
  const bool pool_socket = collect_names && pool_socket_address(address, size);
  if (pool_socket && fail(Operation::Bind)) {errno = EADDRINUSE; return -1;}
  const int result = real(socket, address, size);
  if (result == 0 && pool_socket) {
    const auto * local = reinterpret_cast<const sockaddr_un *>(address);
    socket_names.emplace_back(local->sun_path);
  }
  return result;
}

extern "C" int listen(int socket, int backlog) noexcept
{
  sockaddr_un address{};
  socklen_t size = sizeof(address);
  if (collect_names && getsockname(socket, reinterpret_cast<sockaddr *>(&address), &size) == 0 &&
    pool_socket_address(reinterpret_cast<const sockaddr *>(&address), size) && fail(Operation::Listen))
  {
    errno = EIO;
    return -1;
  }
  static const auto real = original<decltype(&listen)>("listen");
  if (!real) {errno = ENOSYS; return -1;}
  return real(socket, backlog);
}

// Historical cores scan unrelated shared-memory names during initialise.
// Suppress that legacy scavenger in RED runs as well as the repaired tests.
extern "C" DIR * opendir(const char * path)
{
  if (std::strcmp(path, "/dev/shm") == 0) {errno = EACCES; return nullptr;}
  static const auto real = original<decltype(&opendir)>("opendir");
  return real ? real(path) : nullptr;
}
