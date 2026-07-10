#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
status_bin="$2"
model_path="$3"
instance_name="status_e2e_${BASHPID}_$(date +%s%N)"
work_dir="$(mktemp -d)"
server_pid=""
watch_pid=""
probe_pid=""

cleanup() {
  if [[ -n "${probe_pid}" ]] && kill -0 "${probe_pid}" 2>/dev/null; then
    kill "${probe_pid}" 2>/dev/null || true
    wait "${probe_pid}" 2>/dev/null || true
  fi
  if [[ -n "${watch_pid}" ]] && kill -0 "${watch_pid}" 2>/dev/null; then
    kill -INT "${watch_pid}" 2>/dev/null || true
    wait "${watch_pid}" 2>/dev/null || true
  fi
  if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
  fi
  rm -rf "${work_dir}"
}
trap cleanup EXIT

start_server() {
  "${server_bin}" --headless --server-name "${instance_name}" \
    --model "${model_path}" >"${work_dir}/server.log" 2>&1 &
  server_pid="$!"
}

wait_for_pattern() {
  local file="$1" pattern="$2" attempts="${3:-100}"
  for ((i = 0; i < attempts; ++i)); do
    grep -Fq "${pattern}" "${file}" 2>/dev/null && return 0
    if [[ -n "${watch_pid}" ]] && ! kill -0 "${watch_pid}" 2>/dev/null; then
      echo "status watch exited before '${pattern}' appeared" >&2
      return 1
    fi
    sleep 0.1
  done
  echo "timed out waiting for '${pattern}' in ${file}" >&2
  return 1
}

wait_for_line_count() {
  local file="$1" minimum="$2" attempts="${3:-100}"
  for ((i = 0; i < attempts; ++i)); do
    local count
    count="$(wc -l <"${file}" 2>/dev/null || printf '0')"
    ((count >= minimum)) && return 0
    if [[ -n "${watch_pid}" ]] && ! kill -0 "${watch_pid}" 2>/dev/null; then
      echo "status watch exited before producing ${minimum} lines" >&2
      set +e
      wait "${watch_pid}"
      local watch_status="$?"
      set -e
      echo "status watch exit code: ${watch_status}" >&2
      sed -n '1,200p' "${work_dir}/watch.err" >&2 || true
      return 1
    fi
    sleep 0.1
  done
  echo "timed out waiting for ${minimum} lines in ${file}" >&2
  return 1
}

set +e
timeout --kill-after=2s 5s "${status_bin}" \
  --server-name "${instance_name}_missing" --timeout-ms 200 \
  >"${work_dir}/missing.out" 2>"${work_dir}/missing.err"
missing_status="$?"
set -e
if [[ "${missing_status}" -ne 2 ]]; then
  echo "expected missing server exit 2, got ${missing_status}" >&2
  sed -n '1,200p' "${work_dir}/missing.err" >&2 || true
  exit 1
fi

"${status_bin}" --server-name "${instance_name}_interrupt" \
  --timeout-ms 60000 >"${work_dir}/interrupt.out" \
  2>"${work_dir}/interrupt.err" &
probe_pid="$!"
sleep 0.5
kill -INT "${probe_pid}"
for ((i = 0; i < 40; ++i)); do
  kill -0 "${probe_pid}" 2>/dev/null || break
  sleep 0.05
done
if kill -0 "${probe_pid}" 2>/dev/null; then
  echo "status query did not stop promptly after SIGINT" >&2
  exit 1
fi
set +e
wait "${probe_pid}"
interrupt_status="$?"
set -e
probe_pid=""
if [[ "${interrupt_status}" -ne 0 ]]; then
  echo "expected interrupted query exit 0, got ${interrupt_status}" >&2
  sed -n '1,200p' "${work_dir}/interrupt.err" >&2 || true
  exit 1
fi

start_server
timeout 10s "${status_bin}" --server-name "${instance_name}" \
  --timeout-ms 5000 >"${work_dir}/human.out" 2>"${work_dir}/human.err"
grep -Fq "Server: ${instance_name}" "${work_dir}/human.out"
grep -Fq "hinge_pos[1]" "${work_dir}/human.out"
grep -Fq "hinge_motor[1]" "${work_dir}/human.out"

timeout 10s "${status_bin}" --server-name "${instance_name}" \
  --timeout-ms 5000 --json >"${work_dir}/single.json" \
  2>"${work_dir}/single.err"
grep -Fq "\"server_name\":\"${instance_name}\"" "${work_dir}/single.json"
grep -Fq '"resources":{' "${work_dir}/single.json"

"${status_bin}" --server-name "${instance_name}" --timeout-ms 5000 \
  --interval-ms 200 --watch --json >"${work_dir}/watch.ndjson" \
  2>"${work_dir}/watch.err" &
watch_pid="$!"
wait_for_line_count "${work_dir}/watch.ndjson" 2

kill "${server_pid}"
wait "${server_pid}" || true
server_pid=""
wait_for_pattern "${work_dir}/watch.err" "status stream is stale"

cpu_ticks_before="$(awk '{print $14 + $15}' "/proc/${watch_pid}/stat")"
sleep 1
cpu_ticks_after="$(awk '{print $14 + $15}' "/proc/${watch_pid}/stat")"
clock_ticks="$(getconf CLK_TCK)"
cpu_tick_limit="$(((clock_ticks + 3) / 4))"
if ((cpu_ticks_after - cpu_ticks_before > cpu_tick_limit)); then
  echo "stale watch consumed too much CPU: $((cpu_ticks_after - cpu_ticks_before)) ticks" >&2
  exit 1
fi

lines_before_restart="$(wc -l <"${work_dir}/watch.ndjson")"
start_server
wait_for_pattern "${work_dir}/watch.err" "status stream recovered"
wait_for_line_count "${work_dir}/watch.ndjson" "$((lines_before_restart + 2))"
tail -n 2 "${work_dir}/watch.ndjson" | grep -Fq '"resources":{'

kill -INT "${watch_pid}"
if ! wait "${watch_pid}"; then
  echo "status watch did not exit cleanly after SIGINT" >&2
  sed -n '1,200p' "${work_dir}/watch.err" >&2 || true
  exit 1
fi
watch_pid=""
