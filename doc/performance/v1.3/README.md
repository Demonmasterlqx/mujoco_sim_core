# Protocol V3 performance evidence

Both Core benchmark binaries were built with GCC 13.3 in `Release` and run five
times sequentially with `taskset -c 0` on the same host. The baseline is the
detached V1.2 commit `106e9f3`; V1.3 is the current worktree. Both use
`performance_8_controller.xml`, eight Controllers, 30 observation scalars per
Controller, 10,000 steps, and 240,000 modeled publishes.

| Metric (median) | V1.2 | V1.3 | Change |
| --- | ---: | ---: | ---: |
| Tick bytes | 641.987 | 320 | -50.2% |
| CPU seconds | 0.314618 | 0.053888 | -82.9% |
| Wall seconds | 0.317217 | 0.054181 | -82.9% |
| Barrier p99 (us) | 35.256 | 6.221 | -82.4% |
| Maximum RSS (KiB) | 10,840 | 8,576 | -20.9% |

The original scenario-B raw runs are retained in `v12_core_benchmark.ndjson`
and `v13_core_benchmark.ndjson`; the codec run is retained in
`v13_codec.json`.
The V3 dense Tick is 80 header bytes plus 30 x 8
payload bytes. The warmed codec allocation gate runs 10,000 iterations with
zero `operator new` calls.

`message_rate_e2e` runs 10,000 Controller commits against a 1 ms model with one
default Observer. It requires zero Controller heartbeats in active Commit
windows, 1,000 +/- 5% Observation frames, approximately 4 Hz Observer heartbeat,
approximately 10 Hz periodic status, subscriber queries proportional to the
one-second sampling timer, exactly one Tick serialization per issue generation,
and a deliberately induced retry that hits the encoded cache.

Absolute codec p95 values are emitted by `data_frame_performance_test` and must
remain below 50 us for Tick encode and below 30 us for Tick decode, Commit
encode, and Commit decode. Server summaries separately report maximum Tick
encode and Core Commit validation times for real transport runs.

## Scenarios A-D

The expanded deterministic fixtures use the exact V1.2 commit
`106e9f3e2392dee0c8c2eaeca3cf2d3d632b905e` and the V1.3 worktree. Both were
built with GCC 13.3 in `Release`, pinned to CPU 0, warmed for 2,000 steps, and
measured for 10,000 steps. Five sequential raw runs per version and scenario
are stored in:

- `v12_scenarios_abc.ndjson` and `v13_scenarios_abc.ndjson`;
- `v12_scenario_d.ndjson` and `v13_scenario_d.ndjson`.

Scenarios A-C exercise Core directly, so their event queue high watermark is
zero and no eCAL transport process is started in either version. Scenario D
adds the real asynchronous `SensorRuntime`, a 100 Hz non-blocking latest-value
LiDAR queue of depth one, and an allocation-free counting publisher. It uses
the same model and controller/observer workload as C. This keeps discovery and
network scheduling outside the CPU comparison while measuring the actual
capture, framing, queue, and worker path identically in both versions.

| Scenario | Metric (median) | V1.2 | V1.3 | Change |
| --- | --- | ---: | ---: | ---: |
| A | total bytes | 22,350,000 | 11,520,000 | -48.5% |
| A | CPU seconds | 0.089150 | 0.025330 | -71.6% |
| A | RTF | 111.24 | 390.27 | +250.8% |
| A | barrier p99 (us) | 26.488 | 6.763 | -74.5% |
| A | allocations/step | 200.0 | 16.0 | -92.0% |
| B | total bytes | 158,480,000 | 83,840,000 | -47.1% |
| B | CPU seconds | 0.368837 | 0.062438 | -83.1% |
| B | RTF | 26.88 | 158.90 | +491.1% |
| B | barrier p99 (us) | 94.834 | 18.805 | -80.2% |
| B | allocations/step | 1005.0 | 85.0 | -91.5% |
| C | total bytes | 23,010,000 | 11,856,000 | -48.5% |
| C | CPU seconds | 0.088990 | 0.026336 | -70.4% |
| C | RTF | 111.43 | 375.11 | +236.6% |
| C | barrier p99 (us) | 20.417 | 4.558 | -77.7% |
| C | allocations/step | 211.6 | 17.2 | -91.9% |
| D | total bytes | 25,760,705 | 13,803,449 | -46.4% |
| D | CPU seconds | 0.132018 | 0.047572 | -64.0% |
| D | RTF | 75.07 | 207.52 | +176.4% |
| D | barrier p99 (us) | 66.412 | 39.523 | -40.5% |
| D | allocations/step | 215.138 | 19.928 | -90.7% |
| D | queue high watermark | 1 | 1 | 0% |

Scenario D is intentionally non-blocking: the simulation outruns its 100 Hz
worker, so frame counts vary with scheduling and dropped captures are expected.
The median retained runs published 785 V1.2 frames and 552 V1.3 frames; bytes,
CPU, barrier latency, and queue bounds remain directly measured rather than
extrapolated.

## Wire-byte acceptance

The required 80-byte V3 data-frame header plus eight 8-byte command scalars
makes an eight-scalar Commit 144 bytes. The equivalent measured V1.2 Commit is
195 bytes, so this is a 26.2% reduction and passes the 25% Commit
byte-reduction target while retaining the complete fixed identity header.

Tick bytes are 50.2% lower, passing the 50% target. Total data-plane bytes are
between 46.4% and 48.5% lower across scenarios A-D, so every scenario passes
the 45% aggregate target.
