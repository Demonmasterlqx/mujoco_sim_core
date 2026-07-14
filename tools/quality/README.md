# SimulationCore quality gates

Install the pinned analyzer in an isolated environment:

```bash
python3 -m venv build/quality-venv
build/quality-venv/bin/pip install -r tools/quality/requirements.txt
```

Run the checked-in complexity and file-size baseline gate:

```bash
build/quality-venv/bin/python \
  tools/quality/check_simulation_core_complexity.py \
  --lizard build/quality-venv/bin/lizard \
  --baseline tools/quality/simulation_core_complexity_baseline.csv \
  --root .
```

The gate rejects functions above CCN 15 or NLOC 80, `.cpp` files above 500
lines, and increases in NLOC or CCN relative to the baseline. Regenerate the
baseline only after reviewing an intentional complexity reduction or a new
function that is within both hard limits:

```bash
build/quality-venv/bin/lizard --csv \
  core/mujoco_sim_core/src core/mujoco_sim_core/include \
  > tools/quality/simulation_core_complexity_baseline.csv
```

The checker identifies functions by source path and full signature, so moving a
function within its file does not discard its baseline. Its regression tests
run directly with the standard library:

```bash
python3 tools/quality/test_check_simulation_core_complexity.py
```

`.github/workflows/simulation-core-quality.yml` installs the pinned analyzer,
enables `MUJOCO_SIM_ENABLE_QUALITY_GATES` and `MUJOCO_SIM_BUILD_DOCS`, runs Core
CTest, and builds Doxygen with undocumented public APIs treated as errors.

On Linux kernels with high address-space randomization, run ThreadSanitizer
lifecycle tests under `setarch -R`. The checked-in suppression excludes only
reports whose stack enters the prebuilt, uninstrumented eCAL shared library:

```bash
setarch "$(uname -m)" -R env \
  TSAN_OPTIONS="halt_on_error=1:suppressions=$PWD/tools/quality/tsan.supp" \
  ctest --test-dir build/mujoco_sim_server_tsan --output-on-failure \
    -R 'server_test|server_event_queue_test|server_lifecycle_e2e'
```
