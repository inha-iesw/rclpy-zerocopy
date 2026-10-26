// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#include "provenance.hpp"
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace rclpy_agnocast
{
namespace
{
constexpr char kShmPrefix[] = "/dev/shm/agnocast@";

struct Mapping
{
  std::uintptr_t begin, end;
  unsigned int major, minor;
  unsigned long long inode;
  long long publisher;
};
[[noreturn]] void reject()
{
  throw std::runtime_error(
          "subscriber Tensor requires a read-only Agnocast mapping from another process");
}
std::vector<Mapping> readonly_mappings()
{
  std::ifstream maps("/proc/self/maps");
  if (!maps) {reject();}
  std::vector<Mapping> result;
  std::string line;
  while (std::getline(maps, line)) {
    unsigned long long begin, end, offset, inode;
    unsigned int major, minor;
    char perms[5]{};
    int consumed = 0;
    if (std::sscanf(
        line.c_str(), "%llx-%llx %4s %llx %x:%x %llu %n",
        &begin, &end, perms, &offset, &major, &minor, &inode, &consumed) != 7 ||
      std::string(perms) != "r--s" || !inode)
    {
      continue;
    }
    auto name = line.substr(static_cast<std::size_t>(consumed));
    const std::string deleted = " (deleted)";
    if (name.size() >= deleted.size() &&
      name.compare(name.size() - deleted.size(), deleted.size(), deleted) == 0)
    {
      name.resize(name.size() - deleted.size());
    }
    if (name.compare(0, (sizeof(kShmPrefix) - 1), kShmPrefix) != 0) {continue;}
    const auto digits = name.substr((sizeof(kShmPrefix) - 1));
    if (digits.empty() || digits.find_first_not_of("0123456789") != std::string::npos) {continue;}
    long long publisher;
    try {publisher = std::stoll(digits);} catch (const std::exception &) {continue;}
    if (publisher <= 0 || publisher > std::numeric_limits<pid_t>::max() ||
      publisher == getpid()) {continue;}
    result.push_back({begin, end, major, minor, inode, publisher});
  }
  return result;
}
bool same_file(const Mapping & a, const Mapping & b)
{
  return a.major == b.major && a.minor == b.minor && a.inode == b.inode &&
         a.publisher == b.publisher;
}
Mapping covering(
  const std::vector<Mapping> & maps, const void * pointer, std::size_t bytes,
  const Mapping * expected = nullptr)
{
  auto address = reinterpret_cast<std::uintptr_t>(pointer);
  if (!bytes || bytes > std::numeric_limits<std::uintptr_t>::max() - address) {reject();}
  const auto end = address + bytes;
  Mapping origin{};
  bool found = false;
  for (const auto & map : maps) {
    if (address < map.begin || address >= map.end) {continue;}
    if (!found) {origin = map; found = true;}
    if (!same_file(origin, map) || (expected && !same_file(*expected, map))) {reject();}
    if (end <= map.end) {return origin;}
    address = map.end;
  }
  reject();
}
}  // namespace

void require_readonly_mapping(
  const void * root, std::size_t root_bytes, const void * data, std::size_t data_bytes)
{
  const auto maps = readonly_mappings();
  const auto origin = covering(maps, root, root_bytes);
  if (data_bytes) {(void)covering(maps, data, data_bytes, &origin);}
}
}  // namespace rclpy_agnocast
