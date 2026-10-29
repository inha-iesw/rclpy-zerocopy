#include <cuda.h>
#include <cuda_runtime_api.h>
#include <dirent.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"

namespace
{
struct Observation
{
  unsigned receives = 0;
  unsigned retains = 0;
  unsigned pushes = 0;
  unsigned pops = 0;
  unsigned releases = 0;
  unsigned imports = 0;
  int flags = 0;
  unsigned fd_count = 0;
  std::array<int, 8> fds{};
  std::array<int, 8> fd_flags{};
};

// Only the calling test thread receives or invokes CUDA while armed.
thread_local bool observing = false;
thread_local bool single_fd_capacity = false;
thread_local Observation observed;
enum class Failure {Retain, Push, Import};
thread_local Failure failure = Failure::Retain;

template<typename Function>
Function original(const char * symbol)
{
  return reinterpret_cast<Function>(dlsym(RTLD_NEXT, symbol));
}
}  // namespace

extern "C" ssize_t recvmsg(int socket, msghdr * message, int flags)
{
  static const auto real = original<decltype(&recvmsg)>("recvmsg");
  if (!real) {errno = ENOSYS; return -1;}
  // Force the kernel to install exactly one FD while still reporting truncation.
  // The returned flags/data are untouched; this isolates MSG_CTRUNC rejection
  // from the independently tested multiple-FD rejection.
  if (observing && single_fd_capacity) {message->msg_controllen = CMSG_LEN(sizeof(int));}
  const auto result = real(socket, message, flags);
  if (observing) {
    ++observed.receives;
    observed.flags = message->msg_flags;
    if (result > 0) {
      for (auto * header = CMSG_FIRSTHDR(message); header;
        header = CMSG_NXTHDR(message, header))
      {
        if (header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS) {continue;}
        const auto count = (header->cmsg_len - CMSG_LEN(0)) / sizeof(int);
        for (std::size_t index = 0; index < count && observed.fd_count < observed.fds.size();
          ++index)
        {
          int fd = -1;
          std::memcpy(&fd, CMSG_DATA(header) + index * sizeof(int), sizeof(fd));
          observed.fds[observed.fd_count] = fd;
          observed.fd_flags[observed.fd_count++] = fcntl(fd, F_GETFD);
        }
      }
    }
  }
  return result;
}

extern "C" CUresult CUDAAPI cuDevicePrimaryCtxRetain(CUcontext * context, CUdevice device)
{
  if (observing) {
    ++observed.retains;
    // A /dev/null descriptor tests receipt, never a synthetic successful import.
    if (failure == Failure::Retain) {return CUDA_ERROR_OUT_OF_MEMORY;}
  }
  static const auto real = original<decltype(&cuDevicePrimaryCtxRetain)>("cuDevicePrimaryCtxRetain");
  return real ? real(context, device) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuCtxPushCurrent(CUcontext context)
{
  if (observing && ++observed.pushes == 1 && failure == Failure::Push) {
    return CUDA_ERROR_INVALID_CONTEXT;
  }
  static const auto real = original<decltype(&cuCtxPushCurrent)>("cuCtxPushCurrent_v2");
  return real ? real(context) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuDevicePrimaryCtxRelease(CUdevice device)
{
  if (observing) {++observed.releases;}
  static const auto real = original<decltype(&cuDevicePrimaryCtxRelease)>(
    "cuDevicePrimaryCtxRelease_v2");
  return real ? real(device) : CUDA_ERROR_UNKNOWN;
}

extern "C" CUresult CUDAAPI cuCtxPopCurrent(CUcontext * context)
{
  static const auto real = original<decltype(&cuCtxPopCurrent)>("cuCtxPopCurrent_v2");
  const auto result = real ? real(context) : CUDA_ERROR_UNKNOWN;
  if (observing && result == CUDA_SUCCESS) {++observed.pops;}
  return result;
}

extern "C" CUresult CUDAAPI cuMemImportFromShareableHandle(
  CUmemGenericAllocationHandle * handle, void * fd, CUmemAllocationHandleType type)
{
  if (observing) {++observed.imports; return CUDA_ERROR_INVALID_HANDLE;}
  static const auto real = original<decltype(&cuMemImportFromShareableHandle)>(
    "cuMemImportFromShareableHandle");
  return real ? real(handle, fd, type) : CUDA_ERROR_UNKNOWN;
}

namespace
{
using Ref = ros2_cuda_ipc_core::buffer_metadata::BufferRef;
using Metadata = ros2_cuda_ipc_core::buffer_metadata::BufferMetadata;
using Mapper = ros2_cuda_ipc_core::subscriber::BufferMapper;

int open_fd_count()
{
  DIR * directory = opendir("/proc/self/fd");
  if (!directory) {return -1;}
  int count = 0;
  while (const auto * entry = readdir(directory)) {
    if (entry->d_name[0] != '.') {++count;}
  }
  closedir(directory);
  return count;
}

struct Reply
{
  const char * name;
  unsigned fd_count;
  bool separate_headers;
  bool truncated;
  bool single_fd_capacity = false;
};

// A bounded accept and joined sender keep per-request FDs out of the baseline.
class Sender
{
public:
  Sender(int listener, int source, Reply reply)
  : thread_([this, listener, source, reply] {
      pollfd ready{listener, POLLIN, 0};
      if (poll(&ready, 1, 2000) != 1) {return;}
      const int peer = accept4(listener, nullptr, nullptr, SOCK_CLOEXEC);
      if (peer < 0) {return;}
      char byte = 'f';
      iovec payload{&byte, 1};
      alignas(cmsghdr) char control[CMSG_SPACE(8 * sizeof(int))]{};
      msghdr message{};
      message.msg_iov = &payload;
      message.msg_iovlen = 1;
      if (reply.fd_count != 0) {
        message.msg_control = control;
        message.msg_controllen = reply.separate_headers ?
          2 * CMSG_SPACE(sizeof(int)) : CMSG_SPACE(reply.fd_count * sizeof(int));
        auto * header = CMSG_FIRSTHDR(&message);
        const unsigned headers = reply.separate_headers ? 2 : 1;
        const unsigned per_header = reply.separate_headers ? 1 : reply.fd_count;
        for (unsigned group = 0; group < headers; ++group) {
          header->cmsg_level = SOL_SOCKET;
          header->cmsg_type = SCM_RIGHTS;
          header->cmsg_len = CMSG_LEN(per_header * sizeof(int));
          for (unsigned index = 0; index < per_header; ++index) {
            std::memcpy(CMSG_DATA(header) + index * sizeof(int), &source, sizeof(source));
          }
          header = CMSG_NXTHDR(&message, header);
        }
      }
      sent_ = sendmsg(peer, &message, MSG_NOSIGNAL);
      close(peer);
    }) {}

  ~Sender() {if (thread_.joinable()) {thread_.join();}}
  ssize_t finish() {thread_.join(); return sent_;}

private:
  ssize_t sent_ = -1;
  std::thread thread_;
};

class ReceivedFds : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
    failure = Failure::Retain;
    char path[] = "/tmp/g3_fd_receive_XXXXXX";
    ASSERT_NE(mkdtemp(path), nullptr);
    directory_ = path;
    msg_.vmm_socket_path = directory_ + "/reply.sock";
    source_ = open("/dev/null", O_RDONLY | O_CLOEXEC);
    ASSERT_GE(source_, 0);
    listener_ = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    ASSERT_GE(listener_, 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    ASSERT_LT(msg_.vmm_socket_path.size(), sizeof(address.sun_path));
    std::strcpy(address.sun_path, msg_.vmm_socket_path.c_str());
    ASSERT_EQ(bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    socket_owned_ = true;
    ASSERT_EQ(listen(listener_, 1), 0);
    static uint32_t block_id = 2000000;
    msg_.publisher_pid = getpid();
    msg_.block_id = block_id++;
    msg_.device_id = 0;
    msg_.byte_size = 64;
    name_ = ros2_cuda_ipc_core::buffer_metadata::shm_name_for_block(
      msg_.publisher_pid, msg_.block_id);
    mapping_ = Metadata::create(name_);
    ASSERT_TRUE(mapping_);
    auto reservation = Ref::reserve_for_publish(mapping_);
    ASSERT_TRUE(reservation);
    msg_.uid = reservation->uid;
    ASSERT_TRUE(Ref::commit_publish(mapping_, msg_.uid));
  }

  void TearDown() override
  {
    observing = false;
    if (mapping_) {mapping_.reset(); EXPECT_EQ(shm_unlink(name_.c_str()), 0);}
    if (listener_ >= 0) {EXPECT_EQ(close(listener_), 0);}
    if (source_ >= 0) {EXPECT_EQ(close(source_), 0);}
    if (socket_owned_) {EXPECT_EQ(unlink(msg_.vmm_socket_path.c_str()), 0);}
    if (!directory_.empty()) {EXPECT_EQ(rmdir(directory_.c_str()), 0);}
  }

  void expect_reply(Mapper & mapper, Reply reply, unsigned expected_retains)
  {
    const int baseline = open_fd_count();
    ASSERT_GT(baseline, 0);
    CUcontext context_before = nullptr;
    ASSERT_EQ(cuCtxGetCurrent(&context_before), CUDA_SUCCESS);
    observed = {};
    single_fd_capacity = reply.single_fd_capacity;
    observing = true;
    Sender sender(listener_, source_, reply);
    EXPECT_FALSE(mapper.map(msg_, CU_STREAM_LEGACY));
    EXPECT_EQ(sender.finish(), 1);
    observing = false;
    EXPECT_EQ(observed.receives, 1u);
    EXPECT_EQ((observed.flags & MSG_CTRUNC) != 0, reply.truncated);
    constexpr unsigned natural_capacity =
      (CMSG_SPACE(sizeof(int)) - CMSG_LEN(0)) / sizeof(int);
    const unsigned capacity = reply.single_fd_capacity ? 1 : natural_capacity;
    const unsigned expected_fds = reply.fd_count > capacity ? capacity : reply.fd_count;
    EXPECT_EQ(observed.fd_count, expected_fds);
    EXPECT_EQ(observed.retains, expected_retains);
    EXPECT_EQ(observed.pushes, expected_retains == 0 || failure == Failure::Retain ?
      0u : (failure == Failure::Push ? 1u : 2u));
    EXPECT_EQ(observed.pops, expected_retains != 0 && failure == Failure::Import ? 2u : 0u);
    EXPECT_EQ(observed.releases, expected_retains != 0 && failure != Failure::Retain ? 1u : 0u);
    EXPECT_EQ(observed.imports, expected_retains != 0 && failure == Failure::Import ? 1u : 0u);
    EXPECT_EQ(Ref::current_refcount(mapping_), 0u);
    EXPECT_FALSE(mapper.poisoned());
    EXPECT_FALSE(ros2_cuda_ipc_core::detail::CudaDeviceContext::cleanup_failed());
    CUcontext context_after = nullptr;
    EXPECT_EQ(cuCtxGetCurrent(&context_after), CUDA_SUCCESS);
    EXPECT_EQ(context_after, context_before);
    EXPECT_EQ(open_fd_count(), baseline);
    for (unsigned index = 0; index < observed.fd_count; ++index) {
      EXPECT_EQ(observed.fd_flags[index], FD_CLOEXEC);
      errno = 0;
      const int flags = fcntl(observed.fds[index], F_GETFD);
      EXPECT_EQ(flags, -1) << "received fd=" << observed.fds[index];
      EXPECT_EQ(errno, EBADF);
      // After reporting RED, return only the FDs this receive demonstrably leaked.
      if (flags >= 0) {close(observed.fds[index]);}
    }
    EXPECT_GE(fcntl(source_, F_GETFD), 0);
  }

private:
  int listener_ = -1;
  int source_ = -1;
  bool socket_owned_ = false;
  std::string directory_;
  std::string name_;
  ros2_cuda_ipc_msgs::msg::BufferCore msg_;
  std::shared_ptr<Metadata> mapping_;
};

class InvalidReceivedFds : public ReceivedFds, public ::testing::WithParamInterface<Reply> {};

TEST_P(InvalidReceivedFds, RejectsBeforeCudaAndReturnsDescriptorsAndAdmission)
{
  Mapper mapper({1, 64});
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    SCOPED_TRACE(attempt);
    expect_reply(mapper, GetParam(), 0);
  }
  // H=1 must admit the subsequent well-formed response through the same mapper.
  expect_reply(mapper, {"Single", 1, false, false}, 1);
  EXPECT_TRUE(mapper.try_close());
}

INSTANTIATE_TEST_SUITE_P(
  Ancillary, InvalidReceivedFds,
  ::testing::Values(
    Reply{"TwoFds", 2, false, false}, Reply{"SeparateRights", 2, true, false},
    Reply{"Truncated", 8, false, true},
    Reply{"TruncatedWithSingleInstalledFd", 8, false, true, true},
    Reply{"NoRights", 0, false, false}),
  [](const ::testing::TestParamInfo<Reply> & info) {return info.param.name;});

TEST_F(ReceivedFds, SingleFdClosesWhenPrimaryRetainFailsAndAllowsRetry)
{
  Mapper mapper({1, 64});
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    expect_reply(mapper, {"Single", 1, false, false}, 1);
  }
  EXPECT_TRUE(mapper.try_close());
}

TEST_F(ReceivedFds, SingleFdClosesWhenContextPushFailsAndAllowsRetry)
{
  Mapper mapper({1, 64});
  failure = Failure::Push;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    expect_reply(mapper, {"Single", 1, false, false}, 1);
  }
  failure = Failure::Import;
  expect_reply(mapper, {"Single", 1, false, false}, 1);
  EXPECT_TRUE(mapper.try_close());
}

TEST_F(ReceivedFds, SingleFdClosesWhenImportFailsAndAllowsRetry)
{
  Mapper mapper({1, 64});
  failure = Failure::Import;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    expect_reply(mapper, {"Single", 1, false, false}, 1);
  }
  EXPECT_TRUE(mapper.try_close());
}
}  // namespace
