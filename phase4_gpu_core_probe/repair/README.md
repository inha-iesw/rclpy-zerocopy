# Bounded GPU core repair and native Agnocast integration

The reviewed native experiment passed **109 cases, with three multi-GPU skips,
across 38 CTest targets** on the observed Thor configuration. Its own safety and
integration suite has 79 passes; reused upstream tests add 30 passes and three
skips. This establishes the tested safety boundaries;
the full G3 adoption gate remains open. Python GPU Tensor ownership (G4),
Orin validation and performance/copy profiling (G5) remain unfinished.
The original rejection diagnostics and upstream baseline are preserved in
[the parent README](../README.md).

## Source and compatibility

`prepare.py` reconstructs the locked upstream commit
`0e8933c8e58ffa7f1240a4c9efda3a2b84e5398c` with GNU patch, zero fuzz, and verifies
every resulting source file. It neither modifies the pristine tree nor invokes
Git. The MIT core patch is `patches/core-lifecycle.patch`, SHA256
`76400ff87477f1048ac1a4adb41229b14c34c4ddb2b76865c55376bcbf5845c0`
(23 core files).

The existing CUDA VMM allocation/import, SCM_RIGHTS FD transfer and IPC event
engine remain in use. C++ API changes require rebuilding native consumers:
`acquire_for_publish(CUstream)` binds the producer stream, `BufferMapper(Config)`
requires limits, and producer `try_close()` reports a busy/failed pool. Mapper `try_close()` seals
the shared read epoch and polls full read cleanup; `closing()` observes that seal
on every alias, including after successful drain. `CudaDeviceContext::cleanup_failed()`
replaces the earlier primary-release-only query and includes context restoration failures. Manager, pool and
mapper `poisoned()` queries report terminal failure. An unpoisoned mapper may still
be sealed; callers check `closing()` separately. False is not a promise that the
next operation will succeed. The metadata layout
remains 24 bytes, but the writer protocol changes; its private namespace is
`/rclpy_zc_gpu_v1_<pid>_<block>`. Legacy and repaired peers must not be mixed.
The original Python bindings/examples have not been ported to these APIs, and
their earlier 73-case baseline is not a repaired-source test result.
Upstream `BufferCore.msg` comments and `doc/buffer_metadata_protocol.md` describe
the legacy protocol; this patch and the contract below govern repaired peers.

`rclpy_agnocast_gpu_msgs/msg/GpuTensor.msg` carries the unchanged core descriptor
plus dtype, dense shape/element strides, byte offset and allocation size. Typed
Agnocast loans carry this CPU metadata. GPU production writes directly into the
core allocation; consumption uses its mapped read handle. The probe returns a
four-byte GPU checksum to CPU, without staging the 64-byte payload on CPU.
This is source and functional evidence for this path, not profiler proof.
CPU adapter ABI 3 and the Python CPU API are unchanged.

## Scope of the safety contract

- One live producer pool epoch per process. Outstanding blocks retain their pool.
  Pending producer events prevent reuse, including cancellation. Writer admission
  excludes readers while the UID changes, and stale/unpublished UIDs are rejected.
- Streams must belong to the selected device's primary context and remain valid
  through completion recording. Primary-context legacy/default and explicit streams
  are supported; per-thread default and foreign-context streams are rejected.
  Already-bound owners activate their retained primary context for cleanup on
  another host thread, restoring that thread's previous context afterward.
- All mapper aliases share mandatory `H` read slots and maximum message size `M`.
  A permit precedes import and remains charged through active, deferred and failed
  cleanup. Recreating a mapper cannot bypass a live epoch's limits or its shutdown seal. Strict mapping
  keeps no idle metadata/import cache; this has an unmeasured performance cost.
- Completion polling can release a finished read behind a stalled read. Pool close
  claims idle blocks exclusively and refuses pending writes or retained readers.
  Healthy busy final destruction retains one draining pool without poisoning it.
  A later `initialise()` retries its close before allocating a replacement; callers
  retry while old work/readers remain. There is no background publisher reaper.
  Failed mapping/event cleanup retains resources/references; primary-context release failure
  permanently blocks new GPU admission and retains in the process. Context pop
  failure or a mismatched popped context also disables admission. Ambiguous
  cleanup is never retried to make a failed epoch appear healthy.
- A dead consumer's reference is not reclaimed. Its fixed slot remains unavailable.
  A dead reader can therefore prevent the draining pool from ever closing.
  Recovery then requires restarting all involved GPU peers. There is no live-pool
  transparent crash recovery or holder ledger. The inherited global dead-PID
  metadata scavenger has been removed: a PID invisible in this namespace does
  not establish ownership of its IPC names. Existing-name collisions also fail
  initialization without unlinking that object, even when the PID matches.
  Earlier names created by the failed pool are rolled back. This preserves peer
  ownership; it does not establish full cross-PID-namespace deployment support.
  SIGKILL leaves metadata/socket names.
  Tests remove only recorded names after their known GPU peers have exited.

For manager/mapper callers the import mapping bound is at most H mappings of
`align(M, CUDA allocation granularity)` bytes each, including retained failures.
This is a mapping bound, not a physical VRAM bound. Primary-release poisoning
allows only already admitted owners to finish, bounded by H mapper admissions
plus the producer pool. Arbitrary use of low-level core helpers is outside this
experiment's contract.
The installed legacy `BufferMetadataCache`, `IpcHandleCache` and direct VMM importer
are unused by strict mapping and do not enforce its H bound. Subscriber shutdown follows the explicit contract below.

## Shutdown and restart contract

The process GPU runtime owner stops dispatching new reads, then calls mapper
`try_close()` to seal admission across every mapper alias. Individual endpoint
close only releases that endpoint's owners; it must not seal the process epoch.
Unlike producer close, even a busy false mapper result changes admission state.
`closing()` is true for all existing and newly joined aliases, including after
drain. A false result with `closing() && !poisoned()` means active/importing/deferred
reads still exist. Moved-from mappers return false from both close and closing.
Release read owners while their
streams remain valid, let GPU work complete, and retry without resetting CUDA.
Success means context restoration, completion event cleanup, imported-resource
cleanup and reference/permit release are complete. The process worker remains
idle; its queue being empty alone is not a drain condition. A fresh read epoch
requires every old mapper and owner to disappear. Mapper destruction alone does
not seal other aliases.

After subscriber drain, close the producer pool before CUDA/runtime teardown.
On poison or a dead reader, stop all involved GPU peers and restart them together;
a live producer cannot reclaim the stranded reference. Clean only metadata and
socket names recorded by the supervisor for those confirmed-dead peers. Do not
glob-delete names or infer ownership from `kill(pid, 0)` in another PID namespace.
This is an explicit restart/ownership policy, not an implemented production
supervisor, crash collector or transparent reset. Three controlled crash/restart
cycles preserve an unrelated pair's progress and remove only recorded test names.

## Reproduce

Run from the workspace root in a fresh shell. Host Ubuntu **24.04.5** was supplied
by the user; the tested Docker user space is Ubuntu **22.04.5**, ROS Humble,
Python 3.10.12, CUDA 13.2.86/driver 595.78 and Torch 2.13.0+cu132. The GPU is
Thor, capability 11.0, with kernel 6.8.12-1021-tegra/L4T R39.2.1. Architecture 110
below is specific to this observed configuration; it is not an Orin setting.

```bash
mkdir -p log/g3
export PYTHONDONTWRITEBYTECODE=1
python3 phase4_gpu_core_probe/repair/prepare.py > log/g3/repair-sources.json
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base log/g3/repair/core-build build \
  --base-paths build/g3/repaired/ros2_cuda_ipc/ros2_cuda_ipc_msgs \
    build/g3/repaired/ros2_cuda_ipc/ros2_cuda_ipc_core rclpy_agnocast_gpu_msgs \
  --build-base build/g3/repaired-build --install-base install/g3/repaired \
  --packages-select ros2_cuda_ipc_msgs ros2_cuda_ipc_core rclpy_agnocast_gpu_msgs \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
source install/g3/repaired/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base log/g3/repair/probe-build build \
  --base-paths phase4_gpu_core_probe --build-base build/g3/repair-probe \
  --install-base install/g3/repair-probe \
  --cmake-args -DG3_REPAIRED_CORE=ON -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CUDA_ARCHITECTURES=110
G3_RUN_TRANSPORT=1 ctest --test-dir build/g3/repair-probe/phase4_gpu_core_probe \
  --output-on-failure --no-tests=error
```

The process tests require `/dev/agnocast_rclpy` and the verified ARM64
`build/g2-device-arm64/lib/{libagnocast_heaphook.so,libagnocast.so}`. They disable
bridge/discovery and preload that hook. Before loading a module, check
`build/g2-device-arm64/artifacts.json`, source identity, ELF architecture,
vermagic and the running kernel. The old module inside the nested fork is x86_64.
This run temporarily loaded only the matching `agnocast_rclpy` module and unloaded
it afterward; the pre-existing `agnocast` module was preserved. Absent opt-in,
the 18 transport cases skip; this is not an equivalent safety result.
The nested Agnocast fork is still uncommitted; an outer checkout alone does not
reconstruct those device artifacts.

The isolated cleanup and collision executables require exactly one selected case
per process and return an error for an empty filter. CTest registers each filter
explicitly; the runner regression also checks an empty XML cannot mask exit failure.

## Observed results and remaining work

| Test group | Cases | Evidence |
|---|---:|---|
| Metadata reservations | 7 | Exclusive generation, stale reader, overflow, dead reader and 1,000 concurrent admission races |
| Producer lifetime | 4 | Pending cancel/prepare, active close refusal, single live pool |
| Producer stream mismatch | 1 | Rejected preparation preserves a cancellable block without poison |
| Pool retirement and orphan ownership | 6 | Pending prepare/cancel, last block after manager, retained reader, competing replacements and preservation of foreign metadata names |
| Existing metadata name collision | 2 | First/second block collisions preserve the reopened inode, link count and bytes; owned earlier blocks roll back |
| Mapper shutdown state | 3 | Shared seal visibility, moved-from close refusal and fresh epoch only after the final alias disappears |
| Bound default/legacy lifetime | 6 | Prepare/cancel/read completion on a CUDA-unused thread, with context restoration |
| Quarantine | 1 | Injected `cuEventRecord` failure invalidates the block and blocks 20 replacement pools after synchronization |
| Producer context validation | 3 | Primary default allowed; per-thread/foreign contexts rejected |
| Primary release failure | 1 | Injected driver failure blocks 20 replacements; one unresolved release attempt |
| Dense descriptor | 6 | dtype, span, allocation, layout and overflow validation |
| Producer cleanup faults | 11 | Event/query/context/VMM cleanup, partial allocation rollback, context pop failure/mismatch; no cleanup retry or replacement allocation |
| Subscriber completion faults | 4 | Record, query, destroy, failed wait plus failed event cleanup; charged reference retention |
| Subscriber context restoration | 4 | Releaser/worker pop gates and faults; no drain before restoration |
| Isolated runner | 2 | Missing producer/subscriber case filters return failure despite valid zero-test XML |
| Actual Agnocast processes | 18 | Existing nine cases, shared-epoch drain, three restart cycles with an independent pair, and seven actual imported-resource/context cleanup faults |
| Reused upstream core | 30 PASS + 3 SKIP | Context, pool, publisher message, metadata cache, IPC cache and read-handle oracles |

The custom suite is **79 PASS, zero failures/errors/skips**. Together with the
upstream oracles this is **112 collected = 109 PASS + 3 SKIP**, with all 38 CTest
targets passing. All three skips require multiple GPUs. The checkpoint
used 32 fixed blocks: 64 warmup deliveries followed by 1,000 measured deliveries,
checking every checksum and increasing per-block UIDs. Settled before/after
FD/metadata-map counts were publisher **123/32**, subscriber **59/0**. These
measurements do not count CUDA VMM mappings, events or physical memory.

Review corrected two lifetime defects: ordinary busy destruction previously
poisoned the process, and bound default/legacy owners revalidated against the
destruction thread before activating their retained context. Eleven isolated
regression scenarios failed against the previous `351b8cd7...` static core and
pass against this patch. The read-completion thread fixture exercises real CUDA
events and metadata references with a synthetic, undereferenced pointer; payload
and IPC import coverage belongs to the separate transport tests.

The previous seven-target/31-case execution result is historical. Its quarantine
test used a mismatched stream, so it did **not** establish failed-preparation
quarantine: the busy-destruction defect made it pass. The replacement test injects
an actual recording failure, verifies invalidation and poison, and synchronizes
before destroying the manager to exclude transient event timing.

Six upstream test sources now build directly against the repaired core. This
exposed two metadata-cache failures caused by a hardcoded legacy namespace; its
default attachment now uses `shm_name_for_block()`. The manager, mapper, BufferRef
and VMM helper tests still require API/protocol migration. The skipped upstream
`AcceptsSpecialStreamsWhenCallerContextIsOnAnotherDevice` case also needs a new
oracle for strict initial binding before multi-GPU validation. These results do
not establish compatibility with the full upstream test suite or sibling packages.

The primary-release test first failed against the earlier `fc52e218...` patch:
20 replacement pools succeeded and the injected failure was called 21 times.
The `351b8cd7...` repair fixed that defect; the current patch still passes the same
failure scenario with one call. The shim interposes only that
test executable; this is a controlled failure, not an observed real driver fault.
The pending-producer ordering case queries a stream containing only the core
ready wait, before any consumer gate/kernel. The consumer lifetime cases use a
bounded host callback gate. Kernel initialization is warmed before that gate and
the checksum sample is reset so warmup cannot satisfy the completion assertion.

The latest CPU regression reran `CMAKE_BUILD_PARALLEL_LEVEL=2 bash scripts/verify`:
12 packages, 39 CTest targets, **466 PASS + 118 opt-in SKIP**, zero failures/errors.
This is a verifier execution, not a new finalize result. The repaired build covers
core/messages/envelope and native probes, not the old Python/Torch binding.

The cleanup/drain change reproduced **10 failing cases** against the preceding
`0fe547b3...` patch/static core: three VMM rollback faults, two final producer
context restoration faults, failed-bind event cleanup, and four subscriber pop
ordering/failure cases. All pass on the current core. The old core lacks mapper
close, so its subscriber RED build observes the same charged-read/ref completion
condition directly; the current build uses public `try_close()`. These synthetic
completion fixtures never dereference their pointer. Seven separate actual IPC
import cases exercise Unmap/AddressFree/Release/imported EventDestroy, successful
import context pop, and partial-import Unmap/pop failures. Each verifies the
initial cleanup-call count and no additional imports/retains through 20 mapper
recreations. The legacy orphan scan is suppressed in producer RED tests to avoid
removing unrelated names.

The follow-up review fixes reproduced **four failing cases** against the preceding
`850d9d6a...` patch/build: two name collisions and two missing test filters. Collision
tests disable the metadata logger because its error output can change `errno` and
hide the old EEXIST branch. The original inode loses its link and the published
name resolves to a replacement in both RED cases. All four pass on this patch;
three new mapper-state contracts and actual transport drain also pass.

Latest evidence, previous source/static library, RED logs/XML, frozen final XML,
source/binary hashes and device cleanup are under `log/g3/review-fixes/`.
The preceding cleanup/drain evidence remains under `log/g3/cleanup-drain/`.
The verified `agnocast_rclpy` module was loaded temporarily, then unloaded after
all peers exited and module refcount reached zero. The pre-existing `agnocast`
module and 13 sockets were preserved; no new socket names remain. Earlier
`log/g3/repair-review/` and baseline records remain historical evidence.

G3 remains open. This matrix covers the listed cleanup paths, not every allocation,
event-creation, FD/system-call or asynchronous CUDA failure. Remaining work includes
upstream manager/mapper/BufferRef/VMM oracle/API migration, the production supervisor
and recorded-name recovery procedure, remaining failure combinations, and a complete
operation-specific diagnostic audit. Current FD/metadata counts do not establish
physical VRAM/event/VMM accounting or copy profiling. G4 still needs the Python GPU
provider, Tensor aliases and framework-stream lifetime tests. Orin/CUDA 12,
native Ubuntu 24.04/Jazzy, performance/copy trace and G5 remain unverified.
