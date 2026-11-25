#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
model_path="$2"
log_path="/tmp/mujoco_sim_gui_timeout_${BASHPID}.log"
metrics_log_path="/tmp/mujoco_sim_gui_metrics_${BASHPID}.log"
trap 'rm -f "${log_path}" "${metrics_log_path}"' EXIT

set +e
timeout 10s "${server_bin}" \
  --model "${model_path}" \
  --max-ticks 1 \
  --log-summary-period-ms 0 \
  >"${log_path}" 2>&1
status="$?"
set -e

if [[ "${status}" -ne 0 ]]; then
  echo "expected GUI server max_ticks exit 0, got ${status}" >&2
  sed -n '1,160p' "${log_path}" >&2 || true
  exit 1
fi

grep -Fq "event=viewer_initialized mode=gui" "${log_path}"
grep -Fq 'event=server_stopped reason="max_ticks" exit_code=0' \
  "${log_path}"
if grep -Fq "event=server_summary" "${log_path}"; then
  echo "log_summary_period_ms=0 unexpectedly emitted a summary" >&2
  exit 1
fi

set +e
timeout 10s "${server_bin}" \
  --model "${model_path}" \
  --max-ticks 1000 \
  --viewer-sync-period-ms 5 \
  --log-summary-period-ms 50 \
  >"${metrics_log_path}" 2>&1
status="$?"
set -e

if [[ "${status}" -ne 0 ]]; then
  echo "expected GUI metrics run exit 0, got ${status}" >&2
  sed -n '1,200p' "${metrics_log_path}" >&2 || true
  exit 1
fi

grep -Eq 'event=server_summary .*viewer_capture_attempts=[1-9][0-9]* .*viewer_capture_submitted=[1-9][0-9]*' \
  "${metrics_log_path}"
grep -Eq 'event=server_summary .*viewer_sync_completed=[1-9][0-9]* .*viewer_sync_failures=0' \
  "${metrics_log_path}"
grep -Fq 'viewer_sync_period_ms=5' "${metrics_log_path}"
grep -Fq 'event=server_stopped reason="max_ticks" exit_code=0' \
  "${metrics_log_path}"
