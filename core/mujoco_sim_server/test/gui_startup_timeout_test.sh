#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
model_path="$2"
log_path="/tmp/mujoco_sim_gui_timeout_${BASHPID}.log"

set +e
timeout 10s "${server_bin}" \
  --model "${model_path}" \
  --max-ticks 1 \
  --startup-timeout-ms 100 \
  >"${log_path}" 2>&1
status="$?"
set -e

if [[ "${status}" -ne 2 ]]; then
  echo "expected GUI server startup timeout exit 2, got ${status}" >&2
  sed -n '1,160p' "${log_path}" >&2 || true
  exit 1
fi
