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
  --model "${model_path}" --max-ticks 5000 \
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
  grep -Fq "event=service_request_malformed method=RegisterAdapter" "${log}"
  grep -Fq "event=adapter_request_rejected operation=register" "${log}"
  grep -Fq "event=topic_payload_malformed message_type=AdapterCommit" "${log}"
  grep -Fq "event=topic_payload_malformed message_type=AdapterHeartbeat" "${log}"
  grep -Fq "event=tick_summary" "${log}"
  grep -Eq "malformed_messages=[1-9][0-9]*" "${log}"
  grep -Fq "event=server_stopped reason=max_ticks exit_code=0" "${log}"
  malformed_log_count="$(grep -c \
    "event=service_request_malformed method=RegisterAdapter" "${log}")"
  if ((malformed_log_count >= 20)); then
    echo "malformed service warnings were not rate limited" >&2
    exit 1
  fi
  if grep -Eq "event=(tick_published|commit_accepted)" "${log}"; then
    echo "info logging contains per-Tick or per-Commit traffic" >&2
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
