# G3 GPU core adoption probe

The pinned `ros2_cuda_ipc` candidate is **rejected for unmodified G3 adoption**.
The ordinary C++ GPU fanout works, but the candidate can reissue a GPU pointer
while an earlier producer write is still pending. A separate
[bounded repair and native Agnocast experiment](repair/README.md) now passes
43 safety/integration cases; six reused upstream targets add 30 passes and three
multi-GPU skips. The latest repair fixes teardown poisoning and cross-thread
default-stream cleanup. Full G3 adoption and the Python GPU API remain open.
The default build below preserves the original rejection diagnostics;
`G3_REPAIRED_CORE=ON` selects the repaired experiment in a separate build tree.

`upstream.lock.json` fixes commit `0e8933c8e58ffa7f1240a4c9efda3a2b84e5398c`,
the source archive, component licenses/versions, descriptor contract hashes,
and DLPack v1.3 commit `84d107bf416c6bab9ae68ad285876600d230490d`.
The six evaluated upstream packages declare MIT and share the root MIT license;
DLPack declares Apache-2.0. The preparation script verifies every extracted
source file against its locked archive. It does not patch upstream or invoke Git.
CUDA remains an external SDK dependency, not a vendored component.

## Reproduce on the evaluated host

Run from `/workspace/rclpy_zerocopy_ws`. The evaluated host is ARM64 NVIDIA
Thor/L4T R39.2.1; host Ubuntu 24.04.5, tested Docker user space Ubuntu 22.04.5,
Humble/rclpy 3.3.22, CUDA 13.2.86/driver
595.78, Python 3.10.12, PyTorch 2.13.0+cu132, GCC 11.4.0 and CMake 3.22.1.
This does not establish Orin/JetPack 6 compatibility. The tests require a real
CUDA GPU; absent hardware is a failed prerequisite rather than a safety PASS.

```bash
mkdir -p log/g3
export PYTHONDONTWRITEBYTECODE=1
python3 phase4_gpu_core_probe/scripts/prepare_upstream.py > log/g3/sources.json
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base log/g3/upstream-build build \
  --base-paths build/g3/upstream/ros2_cuda_ipc-0e8933c8e58ffa7f1240a4c9efda3a2b84e5398c \
  --build-base build/g3/baseline --install-base install/g3/baseline \
  --packages-up-to ros2_cuda_ipc_py multi_process_image_fanout ros2_cuda_ipc_pointcloud2 \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CUDA_ARCHITECTURES=110 -DBUILD_MOCK=OFF \
    -DFETCHCONTENT_SOURCE_DIR_DLPACK="$PWD/build/g3/upstream/dlpack-84d107bf416c6bab9ae68ad285876600d230490d"
source install/g3/baseline/setup.bash
colcon --log-base log/g3/upstream-test test \
  --base-paths build/g3/upstream/ros2_cuda_ipc-0e8933c8e58ffa7f1240a4c9efda3a2b84e5398c \
  --build-base build/g3/baseline \
  --packages-select ros2_cuda_ipc_core ros2_cuda_ipc_image \
    ros2_cuda_ipc_pointcloud2 ros2_cuda_ipc_py multi_process_image_fanout
colcon test-result --test-result-base build/g3/baseline --verbose
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base log/g3/probe-build build \
  --base-paths phase4_gpu_core_probe --build-base build/g3/probe \
  --install-base install/g3/probe \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
source install/g3/probe/setup.bash
ctest --test-dir build/g3/probe/phase4_gpu_core_probe --output-on-failure --no-tests=error
```

The DLPack override prevents upstream `FetchContent` from cloning a repository.
`BUILD_MOCK=OFF` prevents DLPack's example executable from being written inside
its source tree; it does not change the header-only DLPack dependency.
The first exploratory upstream build used its default FetchContent path; that
checkout's HEAD was checked against the same locked DLPack revision.
`PYTHONDONTWRITEBYTECODE` keeps pytest bytecode out of the verified source tree.
The new package is outside the dependencies selected by the existing CPU
`scripts/verify`; CPU build/import does not acquire CUDA dependencies.

## Original baseline evidence and limits

On 2026-10-26 KST, the unchanged upstream built six packages. Its 16 CTest
targets contained **73 cases: 72 PASS, one optional CuPy SKIP, no failures or
errors**. `colcon test-result` reports 89 because it also counts the 16 CTest
wrappers. Use the testcase XML for the nonduplicated total.

The three probe CTest targets contained **six passing cases**. Four intentionally
confirm candidate defects/limitations, one is a synchronized cancellation
control, and one runs the unchanged C++ publisher plus two independent C++ GPU
consumers. The observer receives scalar status/checksum messages; the demo's
full-image CPU preview is not launched. This is baseline functionality, not a
performance result or an Agnocast GPU integration test.

The existing CPU `scripts/verify` also passed: 12 packages, 39 CTest targets,
584 cases = 466 PASS + 118 device opt-in SKIP, no failures/errors. No kernel
module was loaded. These skips remain unexecuted device cases for the current
CPU source, independent of the real CUDA tests above.

| Observation | Method | Meaning |
|---|---|---|
| Cancel reissues the same pointer with a pending producer write | Real GPU; gated stream, queued device memset, public manager API | Cancellation alone does not establish producer quiescence |
| Prepared publication reissues the same pointer after 150 ms while ready is pending | Real GPU; same gate, successful `prepare_publish`, one-block pool | The 100 ms grace is not a producer-completion fence; a descriptor notification failure/drop cannot make reuse safe |
| Old UID reader survives new writer generation | Explicit white-box paused-prefix simulation calling the real reader API | The reservation CAS and UID store are not atomic with reader admission; this is not a full concurrent GPU reproduction |
| Consumer exit strands a shared reference | Real `fork`/`_exit`, metadata only, no CUDA in child | Core has no per-holder crash reclamation; this is not a GPU crash/recovery test |

The producer probes observe **early pointer reissue**, not measured payload
corruption. Their callback gates are released and streams synchronized before
pool teardown. Passing these diagnostics means the unsafe behavior was
reproduced, **not that G3 safety passed**. If a later revision fixes the behavior,
preserve these pinned rejection diagnostics and run the corresponding safety
assertions in the separate repaired build; do not loosen either mode to preserve
a green result.

Additional source-audit concerns have not been promoted to runtime evidence:
the singleton import cache has no capacity/eviction policy, deferred completion
is unbounded with one blocking worker, and reset lacks an active-reader drain
protocol. Core import normally closes received FDs; mapping/event retention is
not evidence that every message leaks an FD.

For exact contracts see the pinned upstream files:

- `src/buffer_metadata/buffer_ref.cpp`: reservation claim at line 126, UID update
  at 138, reader admission at 153–177; only count/grace govern reuse.
- `src/publisher/gpu_buffer_manager.cpp`: preparation records ready then commits;
  cancellation returns an ordinary unprepared reservation.
- `src/subscriber/read_handle.cpp`: native read ownership defers release until
  consumer stream completion, with failed completions retained.
- `doc/buffer_metadata_protocol.md` and `doc/future_work.md`: crash and drain limits.

Paths above are relative to `ros2_cuda_ipc_core/`, except the `doc/` files.
Full logs, environment, XML-derived counts and the adoption verdict are in
`log/g3/`; build/test XML remains under `build/g3/{baseline,probe}`.

## Original adoption decision and subsequent repair

At the original evaluation, G3 was blocked. Moving the descriptor through
Agnocast could not repair the original core's generation transition. A core
change or another candidate first had to establish atomic generation/admission,
producer quiescence, bounded imports/completion and safe crash/reset policies. Producer
stream synchronization in an adapter may address individual producer paths,
but does not fix concurrent stale-reader admission or crash reclamation.
The subsequent bounded core repair and actual native Agnocast descriptor tests
are documented in [repair/README.md](repair/README.md). They preserve the existing
GPU engine, CPU ABI and the original rejection record. Remaining G3/G4 limits
are explicit there.
