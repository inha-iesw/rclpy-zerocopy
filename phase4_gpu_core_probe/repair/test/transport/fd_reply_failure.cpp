#include "fd_reply_failure.hpp"

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
enum class Failure {None, TwoFds, Truncated, SendError};

Failure selected_failure()
{
  static const Failure selected = []() {
      const char * value = std::getenv("G3_FD_REPLY_FAILURE");
      if (!value) {return Failure::None;}
      if (std::strcmp(value, "two_fds") == 0) {return Failure::TwoFds;}
      if (std::strcmp(value, "truncated") == 0) {return Failure::Truncated;}
      if (std::strcmp(value, "send_error") == 0) {return Failure::SendError;}
      return Failure::None;
    }();
  return selected;
}

std::atomic<unsigned> attempts{0};
std::atomic<unsigned> injections{0};
std::atomic<unsigned> malformed_sent{0};

bool core_reply(int socket, const msghdr * message)
{
  if (!message) {return false;}
  msghdr probe = *message;
  auto * header = CMSG_FIRSTHDR(&probe);
  if (!header || header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS ||
    header->cmsg_len != CMSG_LEN(sizeof(int)) || CMSG_NXTHDR(&probe, header))
  {
    return false;
  }
  sockaddr_un address{};
  socklen_t length = sizeof(address);
  constexpr char prefix[] = "/tmp/cuda_memory_pool_";
  return getsockname(socket, reinterpret_cast<sockaddr *>(&address), &length) == 0 &&
         address.sun_family == AF_UNIX &&
         length >= offsetof(sockaddr_un, sun_path) + sizeof(prefix) - 1 &&
         std::strncmp(address.sun_path, prefix, sizeof(prefix) - 1) == 0;
}
}  // namespace

namespace g3_fd_reply_failure
{
Stats stats()
{
  return {attempts.load(), injections.load(), malformed_sent.load()};
}
}  // namespace g3_fd_reply_failure

extern "C" ssize_t sendmsg(int socket, const msghdr * message, int flags)
{
  static const auto real = reinterpret_cast<decltype(&sendmsg)>(dlsym(RTLD_NEXT, "sendmsg"));
  if (!real) {errno = ENOSYS; return -1;}
  const auto selected = selected_failure();
  if (selected == Failure::None || !core_reply(socket, message)) {
    return real(socket, message, flags);
  }
  if (attempts.fetch_add(1) >= 3) {return real(socket, message, flags);}
  injections.fetch_add(1);
  if (selected == Failure::SendError) {errno = EPIPE; return -1;}

  // Duplicate the sender-owned VMM FD in real SCM_RIGHTS data. The kernel
  // installs receiver-owned copies; neither the source FD nor message changes.
  int source = -1;
  std::memcpy(&source, CMSG_DATA(CMSG_FIRSTHDR(message)), sizeof(source));
  const unsigned count = selected == Failure::TwoFds ? 2 : 8;
  alignas(cmsghdr) char control[CMSG_SPACE(8 * sizeof(int))]{};
  msghdr reply = *message;
  reply.msg_control = control;
  reply.msg_controllen = CMSG_SPACE(count * sizeof(int));
  auto * header = CMSG_FIRSTHDR(&reply);
  header->cmsg_level = SOL_SOCKET;
  header->cmsg_type = SCM_RIGHTS;
  header->cmsg_len = CMSG_LEN(count * sizeof(int));
  for (unsigned index = 0; index < count; ++index) {
    std::memcpy(CMSG_DATA(header) + index * sizeof(int), &source, sizeof(source));
  }
  const auto result = real(socket, &reply, flags);
  if (result == 1) {malformed_sent.fetch_add(1);}
  return result;
}
