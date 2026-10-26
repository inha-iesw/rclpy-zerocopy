# Bounded GPU core repair and native Agnocast integration

The reviewed native experiment passed **73 cases, with three multi-GPU skips,
across 15 CTest targets** on the observed Thor configuration. Its own safety and
integration suite has 43 passes; reused upstream tests add 30 passes and three
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
`0fe547b39e369fbcf0a66f71c79e61e503f8f847bca195a86e9854e83aaed366`
(23 core files).

The existing CUDA VMM allocation/import, SCM_RIGHTS FD transfer and IPC event
engine remain in use. C++ API changes require rebuilding native consumers:
`acquire_for_publish(CUstream)` binds the producer stream, `BufferMapper(Config)`
requires limits, and `try_close()` reports a busy/failed pool. Manager, pool and
mapper `poisoned()` queries distinguish terminal failure from retryable activity.
False is not a promise that the next operation will succeed. The metadata layout
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
  cleanup. Recreating a mapper cannot bypass a live epoch's limits. Strict mapping
  keeps no idle metadata/import cache; this has an unmeasured performance cost.
- Completion polling can release a finished read behind a stalled read. Pool close
  claims idle blocks exclusively and refuses pending writes or retained readers.
  Healthy busy final destruction retains one draining pool without poisoning it.
  A later `initialise()` retries its close before allocating a replacement; callers
  retry while old work/readers remain. There is no background publisher reaper.
  Failed mapping/event cleanup retains resources/references; primary-context release failure
  permanently blocks new GPU admission and retains in the process.
- A dead consumer's reference is not reclaimed. Its fixed slot remains unavailable.
  A dead reader can therefore prevent the draining pool from ever closing.
  Recovery then requires restarting all involved GPU peers. There is no live-pool
  transparent crash recovery or holder ledger. The inherited dead-PID metadata
  scavenger removes names, not GPU references. SIGKILL can also leave socket names;
  operational orphan cleanup remains a separate requirement. Tests explicitly
  remove their known dead peers' IPC names after those peers exit.

For manager/mapper callers the import mapping bound is at most H mappings of
`align(M, CUDA allocation granularity)` bytes each, including retained failures.
This is a mapping bound, not a physical VRAM bound. Primary-release poisoning
allows only already admitted owners to finish, bounded by H mapper admissions
plus the producer pool. Arbitrary use of low-level core helpers is outside this
experiment's contract.
The installed legacy `BufferMetadataCache`, `IpcHandleCache` and direct VMM importer
are unused by strict mapping and do not enforce its H bound. Subscriber deferred
release still lacks a public shutdown/drain API; production exit ordering remains
part of the open G3 work.

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
the nine transport cases skip; this is not an equivalent safety result.
The nested Agnocast fork is still uncommitted; an outer checkout alone does not
reconstruct those device artifacts.

## Observed results and remaining work

| Test group | Cases | Evidence |
|---|---:|---|
| Metadata reservations | 7 | Exclusive generation, stale reader, overflow, dead reader and 1,000 concurrent admission races |
| Producer lifetime | 4 | Pending cancel/prepare, active close refusal, single live pool |
| Producer stream mismatch | 1 | Rejected preparation preserves a cancellable block without poison |
| Pool retirement | 5 | Pending prepare/cancel, last block after manager, retained reader and competing replacements |
| Bound default/legacy lifetime | 6 | Prepare/cancel/read completion on a CUDA-unused thread, with context restoration |
| Quarantine | 1 | Injected `cuEventRecord` failure invalidates the block and blocks 20 replacement pools after synchronization |
| Producer context validation | 3 | Primary default allowed; per-thread/foreign contexts rejected |
| Primary release failure | 1 | Injected driver failure blocks 20 replacements; one unresolved release attempt |
| Dense descriptor | 6 | dtype, span, allocation, layout and overflow validation |
| Actual Agnocast processes | 9 | Pending producer/consumer, stale generation, H limit, fair completion, ordinary import errors, FD disconnect, SIGKILL/restart and checkpoint |
| Reused upstream core | 30 PASS + 3 SKIP | Context, pool, publisher message, metadata cache, IPC cache and read-handle oracles |

The custom suite is **43 PASS, zero failures/errors/skips**. Together with the
upstream oracles this is **76 collected = 73 PASS + 3 SKIP**, with all 15 CTest
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

The prior CPU regression was 12 packages, 39 CTest targets,
**466 PASS + 118 opt-in SKIP**; it was not rerun for these core/probe changes.
The repaired build includes core/messages/envelope, not the old upstream
Python/Torch binding. Current RED/GREEN evidence, source and static-library hashes,
frozen XML/logs and module cleanup are under `log/g3/repair-review/assessment.json`
and `log/g3/repair-review/verification/`. The original baseline/rejection and
`log/g3/repair-assessment.json` remain historical records. Nine inactive sockets
created by this review's RED runs were removed; the 13 pre-existing sockets were
preserved. This test cleanup is not a production orphan collector.

Remaining G3 work includes the broader cleanup-failure matrix and production
crash/orphan/reset policy. Poison transitions now emit diagnostics, but the full
operation-specific cleanup logging audit remains open. G4 still needs the Python GPU provider, Tensor alias
ownership and framework-stream lifetime tests. No Orin/CUDA 12 build, native
Ubuntu 24.04/Jazzy test, production performance, copy trace or G5 result is claimed.
