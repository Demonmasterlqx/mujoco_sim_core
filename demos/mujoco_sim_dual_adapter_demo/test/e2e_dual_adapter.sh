#!/usr/bin/env bash
set -euo pipefail

launcher="$1"
server_bin="$2"
wbr_bin="$3"
double_bin="$4"
server_config="$5"
wbr_config="$6"
double_config="$7"
scene="$8"
mode="$9"
ticks="${10}"
test_dir="$(mktemp -d /tmp/mujoco_sim_dual_e2e.XXXXXX)"

cleanup() {
  rm -rf -- "${test_dir}"
}
trap cleanup EXIT

DUAL_ADAPTER_GRACE_TIMEOUT_S=1 \
DUAL_ADAPTER_TERM_TIMEOUT_S=1 \
DUAL_ADAPTER_KILL_TIMEOUT_S=1 \
bash "${launcher}" \
  --server-bin "${server_bin}" \
  --wbr-bin "${wbr_bin}" \
  --double-bin "${double_bin}" \
  --server-config "${server_config}" \
  --wbr-config "${wbr_config}" \
  --double-config "${double_config}" \
  --scene "${scene}" \
  --output-dir "${test_dir}/output" \
  --server-name "dual_e2e_${BASHPID}_$(date +%s%N)" \
  --mode "${mode}" \
  --wbr-ticks "${ticks}" \
  --timeout-s 45 \
  >"${test_dir}/launcher.log" 2>&1 || {
    sed -n '1,260p' "${test_dir}/launcher.log" >&2 || true
    find "${test_dir}/output/logs" -type f -maxdepth 1 \
      -exec sh -c 'echo "===== $1 =====" >&2; sed -n "1,260p" "$1" >&2' _ {} \; \
      2>/dev/null || true
    exit 1
  }

grep -Fq "dual adapter demo completed" "${test_dir}/launcher.log"
