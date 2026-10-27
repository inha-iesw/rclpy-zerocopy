# G3 bounded-core evidence history

The [README](README.md) owns the current contract, build procedure, results and
remaining gates. This file preserves earlier results and the source identity
that each result tested. The raw `log/` evidence is Git-ignored; this tracked
summary does not replace its XML, binaries or hashes. All stages below left
the full G3 adoption gate open. Recorded PASS is not a new execution or finalize.
Dates follow the design log's KST stage dates.

| Stage | Date | Patch prefix | Recorded GPU result |
|---|---|---|---|
| Initial bounded repair (§22) | 2026-10-26 | `351b8cd7` | 7 targets, 31 PASS |
| Lifetime review fixes (§23) | 2026-10-26 | `0fe547b3` | 15 targets, 73 PASS + 3 SKIP |
| Cleanup/drain (§24) | 2026-10-27 | `850d9d6a` | 34 targets, 102 PASS + 3 SKIP |
| Collision/closing/filter (§25) | 2026-10-27 | `76400ff8` | 38 targets, 109 PASS + 3 SKIP |
| Upstream test migration (§26) | 2026-10-27 | `76400ff8`, unchanged | 42 targets, 127 PASS + 3 SKIP |
| Lineage and documentation (§27) | 2026-10-27 | `76400ff8`, unchanged | 43 targets, 142 PASS + 3 SKIP, including 15 host-only lineage cases |
| Initialisation and request failures (§28) | 2026-10-27 | `76400ff8`, unchanged | 52 targets, 154 PASS + 3 SKIP, including 15 host-only lineage cases |

## Initial bounded repair and primary-release comparison

Initial patch SHA256:
`351b8cd7d40bd5153d9763321c4a99fd0047f47cba7d79d245ed0455ee82f2f0`
(21 changed core files).

The primary-release injection first failed against
`fc52e218638ceb43cd01fd622200fd4e0969010192019068ae37cf501fa39a61`:
20 replacement pools succeeded and the failing release was called 21 times.
The `351b8cd7` repair rejected replacements and attempted release once. This
was an injected failure in the test executable, not an observed driver fault.
Evidence: `log/g3/repair-assessment.json`, including `primary_release_injection`.

The 31-pass run is a real historical result, but its quarantine test used a
mismatched stream. It did not establish failed-prepare quarantine; the old
busy-destructor defect made that path appear to pass. The later test injects an
actual ready-event recording failure, checks invalidation and poison, and
synchronizes before manager destruction. See the historical correction in
`log/g3/repair-review/assessment.json`.

## Lifetime review fixes

Patch SHA256:
`0fe547b39e369fbcf0a66f71c79e61e503f8f847bca195a86e9854e83aaed366`
(23 changed core files).

Ordinary busy destruction previously poisoned the process. Bound default/legacy
owners also checked the destruction thread before activating their retained
primary context. Five retirement and six cross-thread lifetime scenarios failed
against `351b8cd7` and passed on `0fe547b3`. These are corresponding scenarios,
not byte-identical test sources: GREEN added public `poisoned()` assertions.

At that stage, six upstream test sources were reused. Two metadata-cache cases
first failed because of a hardcoded legacy namespace; attachment was changed to
`shm_name_for_block()`. Manager, mapper, BufferRef and VMM migration, plus the
strict-binding special-stream oracle, were still pending **at that stage**.
The later upstream migration completed that work; multi-GPU execution remains
limited as described in the current README.

Results were custom 43 PASS plus upstream 30 PASS/3 multi-GPU SKIP. CPU was not
rerun in this stage; 466 PASS/118 opt-in SKIP referred to the earlier verifier.
Evidence: `log/g3/repair-review/{assessment.json,previous/,verification/}`.
Nine RED-run sockets were removed in a limited cleanup recorded for this stage.

## Cleanup/drain

Patch SHA256:
`850d9d6ad38ce1de8685dcfdbb1508ddc3a15c54c1e1ace1876f86cd749b4c32`.

Ten cases failed against `0fe547b3` and passed after repair: three VMM rollback
faults, two final producer context restoration faults, failed-bind event cleanup,
and four subscriber context-pop ordering/failure cases. The old mapper had no
close API, so RED observed the same charged-read/reference completion condition
directly; GREEN used public `try_close()`. Producer RED suppressed the old orphan
scan to preserve unrelated names.

Synthetic read-completion fixtures exercised real CUDA events and references
without dereferencing their pointer. Seven separate actual IPC import cases
covered Unmap/AddressFree/Release/imported EventDestroy, successful import context
pop and partial-import Unmap/pop failures. They checked initial cleanup calls
and no new imports/retains through 20 mapper recreations.

The pending-producer case queried a stream containing the ready wait before a
consumer gate/kernel. Consumer lifetime cases warmed kernel initialization before
a bounded host callback gate and reset the checksum sample after warmup.

Results were custom 72 PASS plus upstream 30 PASS/3 multi-GPU SKIP. The CPU verifier
was rerun: 12 packages, 39 targets, 584 collected = 466 PASS + 118 opt-in SKIP.
Evidence: `log/g3/cleanup-drain/{assessment.json,verification/,device-session.json}`.

## Collision, closing visibility and empty filters

Patch SHA256:
`76400ff87477f1048ac1a4adb41229b14c34c4ddb2b76865c55376bcbf5845c0`.

Two name-collision and two empty-filter cases failed against `850d9d6a` and
passed after repair. Three additional mapper-state cases were new GREEN cases,
not part of those four RED cases. Results were custom 79 PASS plus upstream
30 PASS/3 multi-GPU SKIP. The CPU verifier again passed 466 cases with 118 skips.

Collision RED disabled the metadata logger because ERROR output could alter
`errno` and hide the old EEXIST deletion branch. The old inode lost its link and
the published name referred to a replacement. Final collision XML and
`red/collision.log` are authoritative; `red/ctest.log` retains the earlier masked
collision PASS and empty-filter RED. Do not treat every saved RED log as the
same execution. The corrected test reopens the name and checks inode, link count
and full bytes. Actual transport drain also passed.

Evidence: `log/g3/review-fixes/{assessment.json,red/,verification/,device-session.json}`.

## Upstream test migration

The production patch remained `76400ff8` above. The existing repaired static core
was reused, SHA256
`ae3c9ddd5b508bc551d70d8eb2fab6ccf4339dcff99e942393a75924f31a562b`.

Five migrated targets contained 22 cases: 20 PASS + 2 multi-GPU SKIP. Five unchanged
targets added 28 PASS + 1 multi-GPU SKIP. With the custom 79 cases, the whole run
had 42 targets and 130 collected = 127 PASS + 3 SKIP. Direct migrated GTest,
whole-probe CTest and the CPU verifier were separately executed; CPU again
reported 466 PASS + 118 opt-in SKIP.

The first VMM fixture recorded ready after leaving the primary context and failed
strict default-stream binding. Moving recording inside that context passed. This
was a fixture migration failure, not a new production-core RED/GREEN result.
Read-handle fixtures use synthetic pointers. VMM's complete 4,096-byte CPU readback
is an import/data oracle, not copy-free evidence or a mapper H-bound proof.

Evidence: `log/g3/upstream-migration/{assessment.json,reconstruction.json,device-session.json,verification/}`.

## Lineage and documentation

The production patch and static core stayed unchanged. A tracked lineage lock
classified all 13 upstream core test files, recording the base revision, original
hashes, six adapted paths and the separate helper. Preparation and CMake configure
now validate that relationship. CMake uses the lock as its executable source registry.
The prior archive/whole-tree pin verification remains in place.

Fourteen contract tests failed before implementation and passed afterward. A
further negative test demonstrated that classifying a test source as support
could remove its registration; validation was tightened and all 15 cases passed.
These are host validation development results, not new GPU-core defect evidence.

Actual build comparisons preserved the source, compile/link flags and linked
dependencies of 11 native executables, and the commands, timeouts and XML paths
of all 42 existing CTest entries. Only the lineage pytest entry was added.
The final probe had 145 cases = 142 PASS + 3 multi-GPU SKIP across 43 targets.
The CPU verifier again reported 466 PASS + 118 device opt-in SKIP. Direct migrated
GTests and the complete transport suite passed. Temporary module cleanup preserved
the existing module and 13 sockets, with no new metadata/socket names remaining.

README history was separated from its current contract. G4 runtime ownership and
GpuTensor's existing wire invariants were documented; message field declarations
and order were preserved. Runtime/adapter implementation, broad harness extraction
and moving the experiment peer were not part of this stage.

Evidence: `log/g3/lineage-integration/`, including `assessment.json`,
`build-contract-comparison.json`, RED/GREEN XML, `device-session.json` and `verification/`.

## Initialisation and request failures

The production patch and repaired static core remained unchanged. The upstream
MIT permission notice was copied verbatim beside the six adapted test sources.
Manager coverage now checks GPU-complete, reader-free rejection at 0/99,999 us,
same-block reuse at 100,000 us, a fresh UID and publication after cancellation.

Seven isolated producer cases fail the second block's address reserve, allocation,
map, access, FD export, event creation or IPC event handle creation. They check a
single injected failure, allocation/release call balance, actual recorded names,
stable FD counts, and both replacement-manager and same-manager restart.
Subscriber event-create and ready-wait failures preserve publication ownership
for a successful retry, then return the reference and H=1 permit on drain.
The latter fixture uses a synthetic pointer and establishes no payload coverage.

Two real Agnocast transport cases inject receive error/EOF after successful Unix
connect. Three rejected requests preserve FD/metadata-map counts; the fourth
imports the original descriptor, verifies the GPU checksum, and closes normally.
Failure occurs before SCM_RIGHTS receipt, so this is not malformed ancillary-data
or already-received FD cleanup coverage. These are coverage additions against
the existing repaired core, not a new production-defect RED/GREEN comparison.

Results: 52 targets, 157 collected = 154 PASS + 3 multi-GPU SKIP; custom 90 PASS,
upstream 49 PASS/3 SKIP and host lineage 15 PASS. CPU verification again passed
466 cases with 118 device opt-in skips. Manager/BufferRef and the new isolated
initialisation/bind filters also ran directly. After the full GPU run, socket
creation-count assertions were added and the affected producer/manager targets
were rebuilt and rerun. Current XML and direct-run evidence include those assertions.
The matching temporary module was unloaded at refcount zero; the existing module,
13 sockets and original metadata inventory were preserved.

Evidence: `log/g3/initialisation-faults/{assessment.json,reconstruction.json,
device-session.json,verification/}`. Full G3 adoption, complete diagnostics and
production recovery remain open. No finalize or Git mutation ran in this stage.

## Shared checkpoint and device observations

Recorded transport checkpoints used 32 blocks, 64 warmup and 1,000 measured
deliveries, validating checksums and increasing per-block UIDs. Settled publisher
FD/metadata-map counts were 123/32 and subscriber counts 59/0 before and after.
These observations do not count physical VRAM, VMM mappings or CUDA events and
are not copy traces. Later transport suites also covered three controlled
crash/restart cycles while an unrelated pair continued to progress.

Device-session records identify verified ARM64 artifacts and temporary
`agnocast_rclpy` module loads. Peers exited and module refcount reached zero before
unload; the pre-existing `agnocast` module and 13 sockets were preserved. Cleanup
uses recorded test-owned names, not a namespace scan. Per-stage exceptions and
source/binary hashes remain in the associated raw evidence.
