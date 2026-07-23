# Viewer decoupling performance evidence

## Result

The GUI-to-headless WBR RTF acceptance ratio is **0.974436436**, which passes
the required minimum of 0.95:

```text
mean headless RTF = 0.999466608
mean GUI RTF      = 0.973916679
acceptance_ratio  = 0.973916679 / 0.999466608 = 0.974436436
```

Each run completed a 6000-State WBR Controller session with nonzero commands,
both scripted stand transitions, and `final_fsm=normal`. Only
`event=server_summary` intervals containing `online_controllers=1` contribute
to the RTF mean. A run's RTF is the arithmetic mean of its qualifying
intervals, and the mode mean is the arithmetic mean of its three run means.

| Mode | Run | ONLINE intervals | Mean RTF | ONLINE wall s | ONLINE CPU s | ONLINE CPU | Max RSS KiB |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| headless | 1 | 6 | 0.999393190 | 6.000642220 | 1.450 | 24.164% | 325748 |
| headless | 2 | 6 | 0.999379073 | 6.002726585 | 1.530 | 25.488% | 325428 |
| headless | 3 | 6 | 0.999627562 | 6.001234800 | 1.580 | 26.328% | 325424 |
| GUI | 1 | 6 | 0.974347361 | 6.002986697 | 28.740 | 478.762% | 892144 |
| GUI | 2 | 6 | 0.973967682 | 6.004309408 | 28.140 | 468.663% | 892608 |
| GUI | 3 | 6 | 0.973434994 | 6.004520029 | 29.080 | 484.302% | 891948 |

RTF, wall time, and CPU are restricted to qualifying Controller-ONLINE
intervals. CPU is the process-wide user-plus-system delta and may exceed 100%
when several cores are active. Max RSS covers the complete 10000-tick Server
lifetime. Xvfb uses Mesa software rendering, so GUI CPU and RSS primarily
characterize this headless test environment.

Across the recorded GUI ONLINE intervals, all 1092 capture attempts were
submitted. There were zero capture skips, pending replacements, and sync
failures. Capture averaged 92.36 us when weighted by attempts; the largest
interval maximum was 3240.067 us. Viewer sync averaged 5.56 ms when weighted by
completed syncs; the largest interval maximum was 14487.204 us. Sync duration
includes waiting for the MuJoCo viewer mutex and does not block the simulation
thread.

The machine was otherwise idle during the retained series. Discovery-failed
setup attempts made before the final single-process benchmark orchestration
were discarded and are not represented in `results.csv`.

## Method

- Date: 2026-07-23, timezone Asia/Shanghai
- Source: final reviewed working tree based on commit
  `ff09d06a4dcb9024b81a189d9bd8cc61987c11de`, including the isolated display
  model, signature validation, and TSan race fix described by this change
- OS: Ubuntu 24.04, Linux `6.8.1-1015-realtime`, x86_64
- CPU: AMD Ryzen 7 7840HS, 8 cores / 16 threads
- Compiler: GCC 13.3
- Build: `Release`, GUI ON, Sensor ON, tests ON
- Display: `xvfb-run -a`, Mesa software rendering
- Viewer cadence: 16 ms
- Server budget: 10000 ticks per run
- Adapter budget: 6000 delivered State frames per run
- Summary interval: 1000 ms, steady clock
- Run order: headless 1, GUI 1, headless 2, GUI 2, headless 3, GUI 3

At each emitted summary boundary, the benchmark sampled aggregate process user
and system ticks from `/proc/<pid>/stat`. CPU seconds for a qualifying interval
are the tick delta from the preceding summary divided by `CLK_TCK`; CPU percent
is that delta divided by the summary's `interval_wall_s`. GNU `time` recorded
whole-process maximum RSS and retained whole-lifetime timing in the ignored raw
artifacts.

The Release workspace was built with:

```bash
source install/setup.bash
colcon build \
  --build-base build/viewer-decoupling-release \
  --install-base install/viewer-decoupling-release \
  --merge-install \
  --packages-up-to mujoco_sim_wbr_lqr_demo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

For each mode and run, the Server command was equivalent to:

```bash
mujoco-sim-server \
  --config install/viewer-decoupling-release/share/mujoco_sim_wbr_lqr_demo/config/wbr_server.yaml \
  --server-name <unique-name> \
  --max-ticks 10000 \
  --log-summary-period-ms 1000 \
  --viewer-sync-period-ms 16 \
  --headless|--gui
```

GUI commands were wrapped in `xvfb-run -a`. The Adapter command was equivalent
to:

```bash
mujoco-sim-wbr-lqr-adapter \
  --config install/viewer-decoupling-release/share/mujoco_sim_wbr_lqr_demo/config/wbr_lqr.yaml \
  --server-name <same-unique-name> \
  --adapter-id <unique-adapter> \
  --ticks 6000 \
  --log-every-ticks 0
```

`results.csv` retains the ONLINE aggregates and whole-process maximum RSS used
above.
