#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
model_path="$2"
log_path="/tmp/mujoco_sim_gui_timeout_${BASHPID}.log"
trap 'rm -f "${log_path}"' EXIT

set +e
timeout 10s "${server_bin}" \
  --model "${model_path}" \
  --max-ticks 1 \
  --startup-timeout-ms 100 \
  --log-summary-period-ms 0 \
  >"${log_path}" 2>&1
status="$?"
set -e

if [[ "${status}" -ne 2 ]]; then
  echo "expected GUI server startup timeout exit 2, got ${status}" >&2
  sed -n '1,160p' "${log_path}" >&2 || true
  exit 1
fi

grep -Fq "event=viewer_initialized mode=gui" "${log_path}"
grep -Fq "event=server_stopped reason=startup_timeout exit_code=2" \
  "${log_path}"
if grep -Fq "event=tick_summary" "${log_path}"; then
  echo "log_summary_period_ms=0 unexpectedly emitted a summary" >&2
  exit 1
fi
