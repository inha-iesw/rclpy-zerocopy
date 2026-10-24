// Copyright 2026 rclpy zerocopy maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef RCLPY_AGNOCAST__TYPES__OPERATIONS_HPP_
#define RCLPY_AGNOCAST__TYPES__OPERATIONS_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace rclpy_agnocast
{
namespace types
{

/** @brief No-throw provider errors; no exception or allocation crosses the callback ABI. */
enum class ErrorCode {kOk, kValue, kIndex, kType, kMemory, kRuntime, kUnsupported};

/** @brief Owned, bounded error text, valid after the provider callback returns. */
struct Status
{
  ErrorCode code{ErrorCode::kOk};
  char message[240]{};
};

/** @brief Native field categories, independent of Python and container layout. */
enum class FieldKind {kScalar, kMessage, kArray, kSequence, kString, kWString};
/** @brief Scalar conversion category; width comes from FieldOps::item_size. */
enum class ScalarKind {kBool, kSigned, kUnsigned, kFloat};

/** @brief Scalar transport value; the field's ScalarKind selects the active member. */
union ScalarValue {
  bool boolean;
  std::int64_t signed_integer;
  std::uint64_t unsigned_integer;
  double real;
};

struct FieldOps;
/** @brief Ephemeral field reference, resolved anew under the root owner's lock. */
struct FieldRef
{
  void * object{nullptr};
  const FieldOps * ops{nullptr};
  std::size_t index{0};  ///< Used only by proxy references such as vector<bool> elements.
};

/** @brief Native contiguous storage; ownership remains with the root. */
struct BufferSpan
{
  void * data{nullptr};
  std::size_t size{0};  ///< Elements, not bytes.
  std::size_t item_size{0};
  const char * format{nullptr};  ///< Native-endian PEP 3118 scalar format.
};

/** @brief Named native member; get performs typed member access, never offset arithmetic. */
struct FieldMember
{
  const char * name;
  const FieldOps * ops;
  void * (*get)(void * object) noexcept;
};

/**
 * @brief Immutable generated operations, pinned with their provider for process lifetime.
 *
 * Every callback catches exceptions locally. References are non-owning and valid only while
 * the root is alive and its structure is unchanged. resize runs exclusively on the publisher
 * worker. Mutable loan access runs on its creator thread. dynamic_bytes excludes this object's
 * inline sizeof storage, includes capacity and nested allocations, and checks overflow.
 * minimum_resize_bytes reports a pre-allocation lower bound for this container alone;
 * nested storage and growth slack are checked by dynamic_bytes after resize.
 * size, child, read, buffer and dynamic_bytes must not mutate storage: subscriber roots
 * invoke these callbacks on immutable native messages, possibly from different threads.
 * Unavailable operations are null. bound is SIZE_MAX for unbounded containers.
 */
struct FieldOps
{
  FieldKind kind;
  ScalarKind scalar{ScalarKind::kUnsigned};
  std::size_t item_size{0};
  const char * format{nullptr};
  std::size_t bound{0};
  const FieldOps * element{nullptr};
  const FieldMember * members{nullptr};
  std::size_t member_count{0};
  Status (* size)(FieldRef ref, std::size_t * size) noexcept = nullptr;
  Status (* child)(FieldRef ref, std::size_t index, FieldRef * child) noexcept = nullptr;
  Status (* read)(FieldRef ref, ScalarValue * value) noexcept = nullptr;
  Status (* write)(FieldRef ref, ScalarValue value) noexcept = nullptr;
  Status (* resize)(FieldRef ref, std::size_t size) noexcept = nullptr;
  Status (* buffer)(FieldRef ref, BufferSpan * span) noexcept = nullptr;
  Status (* dynamic_bytes)(FieldRef ref, std::size_t * bytes) noexcept = nullptr;
  Status (* minimum_resize_bytes)(
    FieldRef ref, std::size_t size,
    std::size_t * bytes) noexcept = nullptr;
};

/**
 * @brief Type-erased native publisher owned exclusively by the existing publisher worker.
 *
 * All callbacks, including destroy, run on that worker. borrow returns a root owned by this
 * endpoint until publish/cancel. publish is terminal even on error; cancel/destroy cannot fail.
 * No callback receives Python objects. The runtime keeps the node/session alive until destroy.
 */
struct NativeEndpoint
{
  void * object{nullptr};
  Status (* borrow)(void * object, void ** root) noexcept = nullptr;
  Status (* publish)(void * object) noexcept = nullptr;
  void (* cancel)(void * object) noexcept = nullptr;
  void (* destroy)(void * object) noexcept = nullptr;
};

/**
 * @brief Transfer one immutable native reference from the provider to the runtime.
 *
 * receive owns owner on entry and must invoke release exactly once, including admission
 * rejection and allocation failures. root remains valid until release(owner) completes.
 */
struct NativeReceived
{
  const void * root{nullptr};
  void * owner{nullptr};
  void (* release)(void * owner) noexcept = nullptr;
};

/** @brief Native receive entry point; a failed Status carries no message ownership. */
using NativeReceive = void (*)(void * context, NativeReceived message, Status status) noexcept;

/**
 * @brief Type-erased subscription controlled by the subscriber runtime.
 *
 * Creation and graph queries run on the subscriber executor. destroy runs only after
 * callbacks are joined/detached and admitted references are gone; after executor join,
 * the closing caller may run destruction. The runtime retains the node/session throughout.
 * publisher_count reports native graph state without exceptions crossing the provider ABI.
 */
struct NativeSubscriptionEndpoint
{
  void * object{nullptr};
  void (* destroy)(void * object) noexcept = nullptr;
  Status (* publisher_count)(void * object, std::size_t * count) noexcept = nullptr;
};

/**
 * @brief Executable operations registered atomically with the type's schema metadata.
 *
 * create receives a non-null agnocast::Node pointer and a borrowed null-terminated topic.
 * It constructs a native publisher on the worker and writes a complete endpoint on success;
 * failures leave the output empty. root and callback code remain pinned for process lifetime.
 * subscribe additionally borrows a pointer to rclcpp::CallbackGroup::SharedPtr, copied by
 * the provider. context must remain alive until the endpoint is detached and destroyed.
 */
struct TypeOps
{
  const FieldOps * root;
  Status (* create)(
    void * node, const char * topic, std::size_t depth,
    NativeEndpoint * endpoint) noexcept;
  Status (* subscribe)(
    void * node, const char * topic, std::size_t depth, void * callback_group,
    void * context, NativeReceive receive,
    NativeSubscriptionEndpoint * endpoint) noexcept = nullptr;
};

/** @brief Runtime-owned exception for an explicitly unsupported field operation. */
class UnsupportedOperation : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/**
 * @brief Throw a runtime-owned standard exception corresponding to a provider status.
 * @param[in] status Provider result copied across the no-throw callback boundary.
 * @throws std::invalid_argument, std::out_of_range, std::bad_alloc, or std::runtime_error.
 */
void check_status(const Status & status);

}  // namespace types
}  // namespace rclpy_agnocast
#endif  // RCLPY_AGNOCAST__TYPES__OPERATIONS_HPP_
