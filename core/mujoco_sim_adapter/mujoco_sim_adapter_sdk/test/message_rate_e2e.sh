#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
probe_bin="$2"
model_path="$3"
config_path="$4"
instance="message_rate_${BASHPID}_$(date +%s%N)"
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

env -u DISPLAY -u WAYLAND_DISPLAY "${server_bin}" --headless \
  --config "${config_path}" --server-name "${instance}" \
  --model "${model_path}" --max-ticks 0 \
  >"${work_dir}/server.out" 2>"${work_dir}/server.log" &
server_pid="$!"

if ! timeout 35s "${probe_bin}" "${instance}" \
    >"${work_dir}/probe.log" 2>&1; then
  sed -n '1,160p' "${work_dir}/probe.log" >&2 || true
  tail -n 240 "${work_dir}/server.log" >&2 || true
  exit 1
fi
kill -INT "${server_pid}"
for _ in $(seq 1 100); do
  kill -0 "${server_pid}" 2>/dev/null || break
  sleep 0.05
done
if kill -0 "${server_pid}" 2>/dev/null; then
  kill -TERM "${server_pid}" 2>/dev/null || true
fi
wait "${server_pid}" || true
server_pid=""

grep -Fq 'rate probe commits=10000' "${work_dir}/probe.log"

sum_field() {
  local field="$1"
  awk -v key="${field}" '
    /event=tick_summary/ {
      for (i = 1; i <= NF; ++i) {
        split($i, pair, "=")
        if (pair[1] == key) total += pair[2]
      }
    }
    END { printf "%.0f\n", total + 0 }
  ' "${work_dir}/server.log"
}

sum_active_field() {
  local field="$1"
  awk -v key="${field}" '
    /event=tick_summary/ {
      commits = 0
      value = 0
      for (i = 1; i <= NF; ++i) {
        split($i, pair, "=")
        if (pair[1] == "commits") commits = pair[2]
        if (pair[1] == key) value = pair[2]
      }
      if (commits > 0) total += value
    }
    END { printf "%.0f\n", total + 0 }
  ' "${work_dir}/server.log"
}

active_seconds="$(awk '
  /event=tick_summary/ {
    commits = 0
    window = 0
    for (i = 1; i <= NF; ++i) {
      split($i, pair, "=")
      if (pair[1] == "commits") commits = pair[2]
      if (pair[1] == "window_s") window = pair[2]
    }
    if (commits > 0) total += window
  }
  END { printf "%.0f\n", total + 0 }
' "${work_dir}/server.log")"

commits="$(sum_field commits)"
controller_heartbeats="$(sum_active_field controller_heartbeats)"
observer_heartbeats="$(sum_active_field observer_heartbeats)"
observations="$(sum_active_field observation_frames)"
status_periodic="$(sum_active_field status_periodic)"
queries="$(sum_active_field subscriber_count_queries)"
misses="$(sum_field tick_cache_misses)"
serializations="$(sum_field tick_serializations)"
hits="$(sum_field tick_cache_hits)"
retries="$(sum_field tick_retransmissions)"

echo "rate metrics active_seconds=${active_seconds} commits=${commits} controller_heartbeats=${controller_heartbeats} observer_heartbeats=${observer_heartbeats} observations=${observations} status_periodic=${status_periodic} queries=${queries} misses=${misses} serializations=${serializations} hits=${hits} retries=${retries}" >&2

((commits == 10000))
((controller_heartbeats <= 2))
((observer_heartbeats >= active_seconds * 3 && observer_heartbeats <= active_seconds * 5))
((observations >= 950 && observations <= 1050))
((status_periodic >= active_seconds * 9 && status_periodic <= active_seconds * 11))
((queries >= active_seconds * 2 && queries <= active_seconds * 4 + 10))
((misses == 10000))
((serializations == misses))
((hits >= 1 && hits <= 3))
((retries >= 1 && retries <= 3))
