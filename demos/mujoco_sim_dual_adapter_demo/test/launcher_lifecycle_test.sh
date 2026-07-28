#!/usr/bin/env bash
set -euo pipefail

launcher="$1"
fake_child="$2"
server_config="$3"
wbr_config="$4"
double_config="$5"
scene="$6"
test_dir="$(mktemp -d /tmp/mujoco_sim_dual_launcher_test.XXXXXX)"

cleanup() {
  rm -rf -- "${test_dir}"
}
trap cleanup EXIT

DUAL_ADAPTER_STARTUP_TIMEOUT_S=5 \
DUAL_ADAPTER_GRACE_TIMEOUT_S=1 \
DUAL_ADAPTER_TERM_TIMEOUT_S=1 \
DUAL_ADAPTER_KILL_TIMEOUT_S=1 \
bash "${launcher}" \
  --server-bin "${fake_child}" \
  --wbr-bin "${fake_child}" \
  --double-bin "${fake_child}" \
  --server-config "${server_config}" \
  --wbr-config "${wbr_config}" \
  --double-config "${double_config}" \
  --scene "${scene}" \
  --output-dir "${test_dir}/output" \
  --server-name launcher_lifecycle_test \
  --mode headless \
  --wbr-ticks 20 \
  --timeout-s 10 \
  >"${test_dir}/launcher.log" 2>&1

grep -Fq "dual adapter demo completed" "${test_dir}/launcher.log"
grep -Fq "event=adapter_activated adapter_id=wbr_lqr_controller" \
  "${test_dir}/output/logs/server.log"
grep -Fq "Command layout:" "${test_dir}/output/logs/double_inverted_adapter.log"
