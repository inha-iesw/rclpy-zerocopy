# G3 bounded-core evidence history

The [README](README.md) owns the current contract, build procedure, test inventory
and remaining gates. This file owns execution results and the source identity
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
| Producer socket/context failures (§29) | 2026-10-28 | `76400ff8`, unchanged | Targeted 25 targets, 26 PASS; not a full probe |
| Producer review corrections (§29 follow-up) | 2026-10-28 | `76400ff8`, unchanged | Targeted 25 targets, 26 PASS; existing 13 initialisation filters preserved |
| FD receipt and reply recovery (§30) | 2026-10-29 | `24eb33e1` | 59 targets, 171 PASS + 3 multi-GPU SKIP, including 23 actual transport and 15 host lineage cases |
| FD receipt review corrections (§30 follow-up) | 2026-10-29 | `cc8185be` | 59 targets, 171 PASS + 3 multi-GPU SKIP; existing receive error/EOF and send-error cases now assert diagnostics |

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

## Producer socket and initial context failures

The canonical core patch and repaired static archive remained unchanged. Six
isolated cases were added: second-block socket/bind/listen, second-event context
push, initial primary-context retain and allocation-context push. The retain
interposer previously observed calls only; it now returns the selected failure.
Bind/listen injection is restricted to pool socket paths; socket injection targets
AF_UNIX stream creation. The bind error is injected without a competing socket
object, and the socket error does not exercise the CLOEXEC fallback.

All initialisation cases check rollback without poison, recorded metadata/socket
removal, FD counts and both replacement-manager and same-manager restart. The
second-block cases balance successful CUDA allocation/release calls, now also
including context push/pop and primary retain/release. Early-context failures
occur before VMM allocation; retain failure creates no metadata either. Calling
thread top-of-stack context identity is checked after rollback and successful retry;
the push/pop counts, rather than identity alone, detect same-primary-context push leaks.
These are coverage additions, not a new production-core defect RED/GREEN result.

The pre-edit seven-case run was observed in the session but its log was not
separately frozen in this evidence directory. The preserved final verification
passed **26 cases across 25 CTest targets**: 13 initialisation, 11 existing terminal
cleanup and two isolated-runner checks. The six new filters also passed directly.
CMake collected 58 targets, but the remaining probe targets and actual transport
were not rerun. This result must not be added to the earlier 154 PASS and presented
as a single full-probe execution. No kernel module was loaded or unloaded.

The CPU verifier was separately rerun: 12 packages, 39 targets, 584 collected =
466 PASS + 118 device opt-in SKIP, zero failures/errors. The context-identity
assertion was added after the first targeted run; the final build, CTest and direct
XML contain that assertion. Evidence: `log/g3/initialisation-system-faults/`, including
`assessment.json`, `reconstruction.json`, build logs and frozen test XML/source hashes.

Direct-run logs identify socket/bind/listen and primary-retain/allocation-context
errors. The second-event context-push error is still reported at the event-creation
boundary. Current diagnostic, FD and recovery gaps are maintained in the README.
Finalize, commit-plan and Git mutations were not run.

## Producer initialisation review corrections

Review `code-review-20261028-093809.md` identified renewed execution-history
duplication in README and six new LOW items. README now describes contracts and
registered coverage; execution totals and stage history live here. The stage
index includes §29. The unpreserved pre-edit baseline is explicitly distinguished
from frozen evidence, and current FD/parser gaps are maintained in README.

The primary-release wrapper now observes calls without recording fictitious
injections. Context cases use their own explicit occurrence/metadata/release
parameters. A shared fixture owns warm-up, baseline, rollback and restart checks;
both suites acquire two distinct replacement blocks. The second-event context
case also confirms one IPC event handle existed before failure. Header/comments
state the process-wide socket selection and the separate roles of top-of-stack
identity and push/pop balance.

Rebuilt producer executables passed **26 cases across 25 CTest targets**; all 13
initialisation filters also passed directly, each selecting exactly one case.
All 58 CTest registrations remained identical. A standalone driver-call probe
selected PrimaryRelease against the previous frozen and current interposers:
both forwarded one successful release, but the former reported one injection
and failed the observer oracle; the latter reported zero and passed. This is a
harness correction, not a GPU-core defect result or an additional registered case.

README's inventory totals 96 custom, 52 upstream and 15 lineage cases; it contains
no PASS/SKIP execution totals. The canonical patch/static core are unchanged.
CPU verification, the full probe, transport, kernel module operations, finalize
and Git mutation were not rerun for these review corrections. Earlier CPU results
remain in `log/g3/initialisation-system-faults/`; current evidence is separately
preserved in `log/g3/initialisation-review-fixes/`, including `assessment.json`,
registration comparison, direct/CTest XML and the before/after observer probe.

## FD receipt and reply recovery

The six-case real Unix-socket reproduction against patch `76400ff8` recorded
two passes and four failures. Two FDs, two sender-side SCM_RIGHTS headers, and
eight FDs truncated to the receiver buffer each leaked an installed FD. A
separate capacity-limited receive installed exactly one FD with `MSG_CTRUNC`:
it did not leak, but incorrectly proceeded to CUDA primary-context retention.
The reproduction's source, static archive, executable and XML remain in
`log/g3/fd-receive/red/`; the two-header sender may be coalesced by Linux.

The canonical patch now walks every returned SCM_RIGHTS FD, rejects anything
other than one FD without control truncation, and closes all installed FDs on
rejection. Patch SHA256 is
`24eb33e1c1a41433ddd5a347b21f77cb754675bc05403e6398d056a59265abd1`.
Only the importer source changed relative to the preceding reconstructed core;
all 23 patched files reconstruct with zero fuzz and verified bytes. The core
and every linked probe target were rebuilt. The original six cases passed;
two added context-push/import failure cases bring the native receipt group to
eight. These use `/dev/null`, deliberate CUDA failures, FD-number/CLOEXEC/EBADF
and count oracles, refcount/H=1 retries, primary-release and push/pop counts,
context identity and non-poison checks. They do not simulate successful import.

Three new actual Agnocast process cases send two/eight copies of the producer's
VMM FD, or inject `EPIPE` before `sendmsg`, for the first three requests. The
malformed responses use real `sendmsg`; the test asserts each one-byte send
succeeded, so an unrelated send failure cannot masquerade as parser rejection.
Both peers' settled FD/metadata-map counts return to baseline. The fourth request
uses the same descriptor, original VMM FD and H=1 mapper, imports successfully,
checks the GPU checksum and drains normally. The injection selects only a
single-FD SCM_RIGHTS reply from the core's Unix-socket path and preserves the
sender's FD and `MSG_NOSIGNAL` flags. No producer implementation change was needed.

After tightening successful-pop and malformed-send observations, the final full
probe passed **171 cases with three multi-GPU skips across 59 CTest targets**:
107 custom, 49 upstream and 15 host lineage passes. GTest's `result="skipped"`
attribute is included in this classification. Actual transport passed all 23
cases, including the 1,000-delivery checkpoint and crash/restart cases. The final
eight native receipt cases and one VMM child-process test also passed directly;
these overlap the full run. CPU `scripts/verify` passed 12 packages/39 targets,
584 collected = 466 PASS + 118 device opt-in SKIP, with no failures/errors.

Before device execution, three ARM64 artifacts, 24 source/prior-artifact hashes,
ELF machine, kernel/vermagic and module srcversion matched the recorded identity.
The temporary module was unloaded at refcount zero after each full run; the
pre-existing module, 13 sockets and metadata inventory were preserved.
`log/g3/fd-receive/assessment.json`, `device-session.json`, `reconstruction.json`,
`green/` and `verification/` preserve results and source/binary/evidence hashes.

Full G3 adoption remains open. Successful GPU retry after each post-receipt CUDA
failure, remaining CUDA/system-call combinations, operation-specific diagnostics
and production recovery remain separate work. G4/G5 and Orin are unfinished.
Finalize and Git mutation were not run for this change.

## FD receipt review corrections

Review `code-review-20261029-043150.md` reported three new LOW items. EOF had
used the errno left by an earlier call, `ScopedFd` implicitly allowed copying,
and the reply-injection header omitted its three-call limit and counter meanings.
The importer now diagnoses `recvmsg == 0` without errno, preserves the negative
return's error diagnostic, deletes both copy operations, and documents ownership
of the returned FD. The header states the selectors, first-three-reply limit,
pass-through-inclusive attempts and successful malformed-send count.

The existing receive-error/EOF and producer send-error tests now inspect the
subscriber log after normal peer exit. Before the core change, receive-error
passed while both EOF cases failed the new diagnostic assertions. With the
updated core, the two EOF cases each report three explicit EOF messages without
the previous errno diagnostic; the receive-error case retains its error messages.
Compile-only probes include the actual importer: copy construction and assignment
compiled before the change and are rejected specifically as deleted afterward;
ordinary construction/reset still compiles. These probes add no registered cases.

The new canonical patch SHA256 is
`cc8185bed6cda674be7db84623dafe6d87867e4ed3a7e243dc1477bf1df0bf89`.
Only the importer changed relative to the preceding repaired tree. Reconstruction
again verified every byte with zero fuzz; the installed core archive and all
linked probe targets were rebuilt after the final patch edit.

The final full run passed **171 cases with three multi-GPU skips across 59 CTest
targets**, including all 23 actual transport cases. The eight native FD cases
also passed directly and overlap that total. CPU `scripts/verify` passed
12 packages/39 targets, 584 collected = 466 PASS + 118 device opt-in SKIP,
without failures or errors. RED and GREEN device runs verified the ARM64
artifacts/source/kernel identity, unloaded the temporary module at refcount zero,
and preserved the existing module, 13 sockets and metadata inventory.

Evidence is separate from the original §30 run:
`log/g3/fd-receive-review-fixes/{assessment.json,reconstruction.json,
core-identity.json,ownership-probe.json,red/,green/,verification/}`.
The cumulative LOW/refactoring/design suggestions were not expanded into this
fix. Full G3, the documented post-receipt CUDA-success retry gap and G4/G5 remain
open. Finalize and Git mutation were not run.

## Post-receipt CUDA failure recovery (2026-10-29)

Three actual Agnocast peer cases now receive a valid producer VMM FD before
injecting primary-context retain, first context push, or VMM import failure.
Each fails the first three maps, then imports the same cached UID through the
same H=1 mapper, verifies the GPU checksum and drains normally. Startup calls
cannot consume these failures: the map-thread observation arms only after a
well-formed, single-FD SCM_RIGHTS receipt from the producer socket.

Every rejection verifies CLOEXEC and immediate EBADF, context restoration,
zero read references, nonpoison, and settled FD/metadata-map counts on both peers.
Retain failure makes no push/import/release calls. Push failure releases its one
successful primary retain. Import failure returns that retain and balances two
successful push/pop pairs, including the partial-import cleanup guard. The fourth
map performs a real import with three successful push/pop pairs. After mapper
drain, all-thread successful retain/release counts and push/pop counts balance;
worker polling counts are variable and are not asserted as an exact total.

The initial targeted run passed all three new cases. Read-only inspection found
that a successful import can close its received FD before later driver work
reuses the same number. The final test therefore requires EBADF on synchronous
rejection only and uses settled FD counts for successful import. After that
assertion adjustment and a rebuild, the full probe passed **174 cases with three
multi-GPU skips across 59 CTest targets**: 110 custom, 49 upstream and 15 host
lineage passes. Actual transport passed all 26 cases, including the 1,000-delivery
checkpoint and crash/restart scenarios. The targeted three cases overlap this
total. CPU `scripts/verify` passed 12 packages/39 targets, **584 collected =
466 PASS + 118 device opt-in SKIP**, without failures or errors.

The core patch remains `cc8185bed6cda674be7db84623dafe6d87867e4ed3a7e243dc1477bf1df0bf89`.
No core modification was needed. All 140 reconstructed source hashes and both
built/installed core archive hashes match the previous verified reconstruction;
this run rebuilt the probe, not the core or a new empty build tree.

Device runs verified all five recorded artifacts and 24 source/prior-artifact
hashes, ELF architecture, kernel/vermagic and module srcversion. Each temporary
module was unloaded after peers exited and its refcount reached zero. The
pre-existing module, 13 sockets and metadata inventory were preserved.
Evidence is in `log/g3/post-receipt-recovery/`, including `assessment.json`,
`targeted/`, the initial `full/`, the final `final/`, and frozen `verification/`.

These results close the three post-receipt CUDA-success retry cases. Remaining
failure combinations, operation-specific diagnostics and production recovery
still keep G3 open; G4/G5 and Orin remain unfinished. Finalize and Git mutation
were not run for this change.

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
