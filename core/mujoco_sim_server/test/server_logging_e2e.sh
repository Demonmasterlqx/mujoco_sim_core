#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
probe_bin="$2"
model_path="$3"
instance_name="logging_e2e_${BASHPID}_$(date +%s%N)"
work_dir="$(mktemp -d)"
server_pid=""

cleanup() {
  if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
  fi
  rm -rf "${work_dir}"
}
trap cleanup EXIT

"${server_bin}" --headless --server-name "${instance_name}" \
  --model "${model_path}" --max-ticks 10000 \
  --log-summary-period-ms 100 --log-file "${work_dir}/file/server.log" \
  >"${work_dir}/server.out" 2>"${work_dir}/server.err" &
server_pid="$!"

sleep 0.1
timeout 8s "${probe_bin}" "${instance_name}"

set +e
wait "${server_pid}"
server_status="$?"
set -e
server_pid=""
if [[ "${server_status}" -ne 0 ]]; then
  echo "expected max_ticks exit 0, got ${server_status}" >&2
  sed -n '1,240p' "${work_dir}/server.err" >&2 || true
  exit 1
fi

if grep -Fq "event=" "${work_dir}/server.out"; then
  echo "server logger unexpectedly wrote structured logs to stdout" >&2
  sed -n '1,120p' "${work_dir}/server.out" >&2 || true
  exit 1
fi

for log in "${work_dir}/server.err" "${work_dir}/file/server.log"; do
  grep -Fq "event=server_starting" "${log}"
  grep -Fq "event=server_started" "${log}"
  if [[ "$(grep -c 'event=server_summary' "${log}")" -lt 2 ]]; then
    echo "expected periodic server summaries in ${log}" >&2
    exit 1
  fi
  positive_number='([1-9][0-9]*(\.[0-9]+)?|0\.[0-9]*[1-9][0-9]*)'
  grep -Eq "event=server_summary .*interval_steps=[1-9][0-9]* .*step_rate_hz=${positive_number} .*realtime_factor_rate=${positive_number}" \
    "${log}"
  grep -Eq 'event=server_summary .*transport_endpoints=[0-9]+ .*malformed_messages=[0-9]+' \
    "${log}"
  awk '
    /event=server_summary/ {
      interval_simulation = ""
      interval_wall = ""
      measured_rate = ""
      for (field_index = 1; field_index <= NF; ++field_index) {
        split($field_index, field, "=")
        if (field[1] == "interval_simulation_s") interval_simulation = field[2]
        if (field[1] == "interval_wall_s") interval_wall = field[2]
        if (field[1] == "realtime_factor_rate") measured_rate = field[2]
      }
      if (interval_simulation != "" && interval_wall + 0 > 0 &&
          measured_rate != "") {
        expected_rate = interval_simulation / interval_wall
        difference = measured_rate - expected_rate
        if (difference < 0) difference = -difference
        tolerance = 0.000001 * (1 + expected_rate)
        if (difference > tolerance) exit 1
        ++checked
      }
    }
    END { if (checked < 2) exit 1 }
  ' "${log}"
  grep -Fq 'event=server_stopped reason="max_ticks" exit_code=0' "${log}"
  if grep -Eq "event=(state_published|command_accepted)" "${log}"; then
    echo "info logging contains per-State or per-Command traffic" >&2
    exit 1
  fi
  if grep -Fq "event=message_send_rejected message_type=ServerStatus" \
    "${log}"; then
    echo "missing status subscribers were reported as publish failures" >&2
    exit 1
  fi
  if grep -Fq "event=stop_requested reason=external_stop" "${log}"; then
    echo "destruction appended an incorrect external_stop reason" >&2
    exit 1
  fi
done
