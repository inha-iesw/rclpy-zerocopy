#include "agnocast/agnocast.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "ros2_cuda_ipc_core/transport/message_utils.hpp"

#include <unistd.h>

#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "descriptor.hpp"
#include "checksum.hpp"
#include "../test/support.hpp"
#ifdef G3_CLEANUP_INJECTION
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "../test/cleanup/import_failure.hpp"
#include "../test/transport/fd_reply_failure.hpp"
#endif

namespace
{
using Message = g3_transport::Message;
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;
using Mapper = ros2_cuda_ipc_core::subscriber::BufferMapper;
using Read = ros2_cuda_ipc_core::subscriber::ReadHandle;

void enqueue_checksum(const Read & read, unsigned * result, unsigned * sample, cudaStream_t stream)
{
  g3_test::check(cudaMemsetAsync(result, 0, sizeof(unsigned), stream));
  g3_transport::checksum(read.data<uint8_t>(), result, stream);
  g3_test::check(cudaGetLastError());
  g3_test::check(cudaMemcpyAsync(
      sample, result, sizeof(unsigned), cudaMemcpyDeviceToHost, stream));
}

void resources()
{
  const auto count = std::distance(
    std::filesystem::directory_iterator("/proc/self/fd"), std::filesystem::directory_iterator());
  std::ifstream maps("/proc/self/maps");
  std::string line;
  unsigned metadata = 0;
  while (std::getline(maps, line)) {
    if (line.find("/rclpy_zc_gpu_v1_") != std::string::npos) {++metadata;}
  }
  std::cout << "RESOURCES " << count << " " << metadata << std::endl;
}

void drain(Mapper & mapper)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!mapper.try_close()) {
    if (mapper.poisoned() || std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("GPU subscriber did not drain; peer restart required");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

int publisher(const std::string & topic, std::size_t blocks)
{
  agnocast::Node node("g3_gpu_publisher");
  auto endpoint = node.create_publisher<Message>(topic, 4);
  Manager manager({blocks, 64, 0});
  if (!manager.initialise()) {throw std::runtime_error("GPU pool initialisation failed");}
  g3_test::Stream stream;
  std::cout << "READY publisher " << getpid() << std::endl;
  std::string command;
  while (std::getline(std::cin, command)) {
    std::istringstream input(command);
    std::string action;
    input >> action;
    if (action == "publish" || action == "publish_pending" || action == "probe") {
      unsigned value = 1;
      input >> value;
      auto block = manager.acquire_for_publish(stream.driver());
      if (!block) {
        std::cout << (manager.poisoned() ? "POISONED" : "FULL") << std::endl;
        continue;
      }
      if (action == "probe") {
        block->cancel();
        g3_test::check(cudaStreamSynchronize(stream.value));
        std::cout << "AVAILABLE" << std::endl;
        continue;
      }
      if (action == "publish_pending") {
        // Start from known bytes; the following write remains behind this gate.
        g3_test::check(cudaMemsetAsync(block->device_ptr(), 0, 64, stream.value));
        g3_test::check(cudaStreamSynchronize(stream.value));
        stream.gate();
      }
      // Direct production in the core's VMM allocation, no payload upload.
      g3_test::check(cudaMemsetAsync(block->device_ptr(), value, 64, stream.value));
      auto descriptor = block->prepare_publish(stream.driver());
      if (!descriptor) {throw std::runtime_error("GPU preparation failed");}
      auto message = endpoint->borrow_loaned_message();
      ros2_cuda_ipc_core::transport::fill_buffer_core_message(descriptor.value(), message->core);
      message->dtype_code = 1;
      message->dtype_bits = 8;
      message->dtype_lanes = 1;
      message->shape = {64};
      message->strides = {1};
      message->byte_offset = 0;
      message->allocation_bytes = 64;
      const auto uid = message->core.uid;
      const auto id = message->core.block_id;
      endpoint->publish(std::move(message));
      std::cout << "PUB " << uid << " " << id << " " << value << " " <<
        descriptor.value().vmm_socket_path << std::endl;
    } else if (action == "connections") {
      std::cout << "CONNECTIONS " << endpoint->get_subscription_count() << std::endl;
    } else if (action == "resources") {
      resources();
#ifdef G3_CLEANUP_INJECTION
    } else if (action == "fd_reply_stats") {
      const auto counts = g3_fd_reply_failure::stats();
      std::cout << "FD_REPLY_STATS " << counts.attempts << " " << counts.injections << " " <<
        counts.malformed_sent << std::endl;
#endif
    } else if (action == "release") {
      stream.release();
      g3_test::check(cudaStreamSynchronize(stream.value));
      std::cout << "PRODUCED" << std::endl;
    } else if (action == "close" || action == "quit") {
      const bool closed = manager.try_close();
      std::cout << "CLOSED " << closed << std::endl;
      if (closed) {return 0;}
    } else {
      throw std::invalid_argument("unknown publisher command");
    }
  }
  return 0;
}

int subscriber(const std::string & topic, std::size_t limit)
{
  agnocast::Node node("g3_gpu_subscriber");
  auto endpoint = std::make_shared<agnocast::TakeSubscription<Message>>(
    &node, topic, rclcpp::QoS(4));
  auto mapper = std::make_unique<Mapper>(Mapper::Config{limit, 64});
  auto stream = std::make_unique<g3_test::Stream>();
  unsigned * sample = nullptr;
  unsigned * device_sum = nullptr;
  g3_test::check(cudaMalloc(reinterpret_cast<void **>(&device_sum), sizeof(unsigned)));
  g3_test::check(cudaHostAlloc(
      reinterpret_cast<void **>(&sample), sizeof(unsigned), cudaHostAllocDefault));
  *sample = 0;
  Message cached;
  std::deque<Read> reads;
  bool cached_valid = false;
  bool gated = false;
  bool warmed = false;
  std::cout << "READY subscriber " << getpid() << std::endl;
  std::string command;
  try {
    while (std::getline(std::cin, command)) {
      if (command == "stash" || command == "take") {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        agnocast::ipc_shared_ptr<const Message> message;
        do {
          message = endpoint->take(false);
          if (message) {break;}
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (std::chrono::steady_clock::now() < deadline);
        if (!message) {
          std::cout << "MISSING" << std::endl;
          continue;
        }
        cached = *message;
        cached_valid = true;
        // Descriptor storage is released before any GPU read lifetime below.
        message.reset();
        if (g3_transport::tensor_bytes(cached) != 64 || cached.dtype_code != 1 ||
          cached.dtype_bits != 8)
        {
          throw std::invalid_argument("probe requires a 64-byte uint8 Tensor");
        }
        if (command == "stash") {
          std::cout << "STASHED " << cached.core.uid << std::endl;
          continue;
        }
      }
      if (command == "take" || command == "map" || command == "map_only") {
        if (!cached_valid) {throw std::runtime_error("no descriptor cached");}
#ifdef G3_CLEANUP_INJECTION
        g3_import_failure::begin_map();
#endif
        auto read = mapper->map(cached.core, stream->driver());
#ifdef G3_CLEANUP_INJECTION
        g3_import_failure::end_map();
#endif
        if (!read) {
          std::cout << (mapper->poisoned() ? "POISONED" :
            mapper->closing() ? "CLOSING" : "REJECTED") << std::endl;
          continue;
        }
        if (!gated && command != "map_only") {
          // CUDA lazy module loading may synchronize the context on first
          // kernel use. Warm the checksum before installing the synthetic gate.
          if (!warmed) {
            enqueue_checksum(*read, device_sum, sample, stream->value);
            g3_test::check(cudaStreamSynchronize(stream->value));
            warmed = true;
          }
          *sample = std::numeric_limits<unsigned>::max();
          stream->gate();
          gated = true;
          enqueue_checksum(*read, device_sum, sample, stream->value);
        }
        reads.push_back(std::move(*read));
        std::cout << "MAPPED " << cached.core.uid << " " << reads.size() << std::endl;
      } else if (command == "drop") {
        reads.clear();
        std::cout << "DROPPED" << std::endl;
      } else if (command == "close_read") {
        std::cout << "READ_CLOSED " << mapper->try_close() << std::endl;
      } else if (command == "pending") {
        const auto status = cudaStreamQuery(stream->value);
        if (status != cudaErrorNotReady) {g3_test::check(status);}
        std::cout << (status == cudaErrorNotReady ? "PENDING" : "COMPLETE") << std::endl;
      } else if (command == "finish") {
        if (!gated && !reads.empty()) {
          enqueue_checksum(reads.front(), device_sum, sample, stream->value);
        }
        stream->release();
        reads.clear();
        g3_test::check(cudaStreamSynchronize(stream->value));
        std::cout << "DONE " << static_cast<unsigned>(*sample) << std::endl;
        stream = std::make_unique<g3_test::Stream>();
        gated = false;
      } else if (command == "remap") {
        mapper.reset();
        mapper = std::make_unique<Mapper>(Mapper::Config{limit, 64});
        std::cout << "REMAPPED" << std::endl;
      } else if (command == "wrong_limits") {
        try {
          Mapper other(Mapper::Config{limit + 1, 64});
          std::cout << "LIMITS_ACCEPTED" << std::endl;
        } catch (const std::logic_error &) {
          std::cout << "LIMITS_REJECTED" << std::endl;
        }
      } else if (command == "fastmap") {
        g3_test::Stream independent;
        auto read = mapper->map(cached.core, independent.driver());
        if (read) {
          g3_test::check(cudaStreamSynchronize(independent.value));
          read.reset();
          std::cout << "FAST_DONE" << std::endl;
        } else {
          std::cout << "REJECTED" << std::endl;
        }
      } else if (command == "bad_event" || command == "oversize") {
        auto core = cached.core;
        if (command == "bad_event") {core.event_handle.fill(0);} else {core.byte_size = 65;}
        auto read = mapper->map(core, stream->driver());
        std::cout << (read ? "UNEXPECTED_MAP" : "REJECTED") << std::endl;
      } else if (command == "resources") {
        resources();
#ifdef G3_CLEANUP_INJECTION
      } else if (command == "receipt_stats") {
        const auto counts = g3_import_failure::receipt_stats();
        using namespace ros2_cuda_ipc_core::buffer_metadata;
        const auto metadata = BufferMetadata::attach(shm_name_for_block(
            cached.core.publisher_pid, cached.core.block_id));
        const auto refs = BufferRef::current_refcount(metadata);
        if (!refs) {throw std::runtime_error("missing receipt-test metadata");}
        std::cout << "RECEIPT_STATS " << counts.fds << " " << counts.cloexec << " " <<
          counts.closed << " " << counts.context_restored << " " << counts.retains << " " <<
          counts.pushes << " " << counts.pops << " " << counts.releases << " " <<
          counts.imports << " " << *refs << " " << counts.lifetime_retains << " " <<
          counts.lifetime_pushes << " " << counts.lifetime_pops << " " <<
          counts.lifetime_releases << std::endl;
      } else if (command == "vmm_stats") {
        const auto counts = g3_import_failure::receipt_stats();
        std::cout << "VMM_STATS " << counts.imported << " " << counts.granularities << " " <<
          counts.reserves << " " << counts.maps << " " << counts.accesses << " " <<
          counts.unmaps << " " << counts.address_frees << " " << counts.allocation_releases <<
          " " << counts.event_opens << " " << counts.fd_closes << std::endl;
      } else if (command == "fault_stats") {
        const auto counts = g3_import_failure::stats();
        std::cout << "FAULT_STATS " << counts.failures << " " << counts.cleanup_calls << " " <<
          counts.imports << " " << counts.retains << " " << counts.connections << " " <<
          counts.receives << " " << mapper->poisoned() << std::endl;
#endif
      } else if (command == "quit") {
        break;
      } else if (command != "stash") {
        throw std::invalid_argument("unknown subscriber command");
      }
    }
  } catch (...) {
    stream->release();
    reads.clear();
    g3_test::check(cudaStreamSynchronize(stream->value));
    (void)cudaFreeHost(sample);
    (void)cudaFree(device_sum);
    throw;
  }
  stream->release();
  reads.clear();
  g3_test::check(cudaStreamSynchronize(stream->value));
  // Keep CUDA/runtime and Agnocast alive until deferred read cleanup completes.
  drain(*mapper);
  g3_test::check(cudaFreeHost(sample));
  g3_test::check(cudaFree(device_sum));
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {return 2;}
  bool initialised = false;
  try {
    g3_test::check(cudaSetDevice(0));
    agnocast::init(0, nullptr);
    initialised = true;
    const int result = std::string(argv[1]) == "publisher" ?
      publisher(argv[2], std::stoul(argv[3])) : subscriber(argv[2], std::stoul(argv[3]));
    agnocast::shutdown();
    return result;
  } catch (const std::exception & error) {
    if (initialised) {agnocast::shutdown();}
    std::cerr << "g3_peer: " << error.what() << std::endl;
    return 1;
  }
}
