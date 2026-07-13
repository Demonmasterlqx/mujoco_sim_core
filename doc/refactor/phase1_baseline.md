# Phase-one baseline and verification

Baseline source is commit `5a7e8cd3f78406883ea7e72c3c1ad5b27fd467b8`. Its recorded functional baseline is 9 packages and 50 passing tests. The refactor preserves the lockstep, timeout/recovery, Observer scheduling, snapshot, sensor, and teardown semantics described in the phase-one plan.

Verification environment: Linux x86_64, MuJoCo 3.10.0 bundled distribution, CMake/CTest through colcon. On 2026-07-13 the current Core test suite and all 12 Server tests passed. The optional-build script passed GUI ON + sensor ON, GUI OFF + sensor ON, and GUI OFF + sensor OFF, including install consumers, headless smoke, and dependency exclusion.

An independent 9-package build from an empty build/install tree passed. A
headless strict build passed with `-Wall -Wextra -Wpedantic -Werror`; unused
warnings were disabled only because configuration-generator output contains
unused helpers. ASAN/UBSAN passed all 46 Core tests and all 12 Server tests,
including sensor, viewer, installed-consumer, lifecycle and status E2E tests.

GCC TSAN requires `setarch x86_64 -R` on this host because its default address
layout conflicts with TSAN shadow memory. With that invocation, all 46 Core
tests, all 60 Adapter SDK unit tests, and the scheduler/callback/logging
concurrency tests pass. The first
executable TSAN run exposed a scheduler lost-wakeup caused by protecting the
condition predicate with a different mutex than the wait. The scheduler now
keeps the predicate and wake generation under its own mutex while explicitly
releasing the application lock during the wait; the repaired test passes under
TSAN.

Tick cache tests assert one serialization per immutable handle, identical
handle/bytes on retransmission, and invalidation after completion, model reload,
replacement, unregister, and stop. Runtime metrics expose serializations, cache
hits/misses/bytes, avoided layout bytes, hash mismatches, blocking waits, and
scheduler wakeups.

## Performance acceptance

The reproducible fixture is
`core/mujoco_sim_core/test/performance_benchmark.cpp` with
`test/testdata/performance_8_controller.xml`. It runs 8 Controllers, 30
observations per Controller, a 1 kHz timestep, 10,000 steps (10 seconds of
simulation time), and three publications per Tick (initial plus the two
publications reached at a 5 ms retransmission cadence during a 10 ms barrier).
Both versions execute the same commits and MuJoCo model. CPU and RSS are process
values from `getrusage`; barrier P99 covers Tick construction/publication work,
all eight commits, and `mj_step()`.

The baseline executable was built from a `git archive` of
`5a7e8cd3f78406883ea7e72c3c1ad5b27fd467b8` in `/tmp`, against that commit's
installed Core and Protocol packages. Five sequential runs of each executable
produced these medians:

| Metric | Baseline | V2 refactor | Change |
| --- | ---: | ---: | ---: |
| Protobuf serializations | 240,000 | 80,000 | -66.7% |
| Average Tick bytes | 1740.740 | 641.987 | -63.1% |
| Maximum Tick bytes | 1741 | 642 | -63.1% |
| CPU seconds | 7.69664 | 2.04879 | -73.4% |
| Barrier P99 (us) | 1183.59 | 365.478 | -69.1% |
| Maximum RSS (KiB) | 11,136 | 10,624 | -4.6% |

Every V2 run reported exactly 80,000 serializations, one for each
Adapter/Tick, while publishing 240,000 messages. Each run also reported
90,380,000 layout bytes avoided. Thus CPU does not exceed baseline, RSS does
not increase, and barrier P99 improves rather than approaching the allowed 5%
regression limit.
