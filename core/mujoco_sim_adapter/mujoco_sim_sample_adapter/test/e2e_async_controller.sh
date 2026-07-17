#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
adapter_bin="$2"
model_path="$3"
mode="${4:-headless}"

instance_name="e2e_${BASHPID}_$(date +%s%N)"
server_log="/tmp/mujoco_sim_server_${instance_name}.log"
adapter_log="/tmp/mujoco_sim_adapter_${instance_name}.log"
server_pid=""

cleanup() {
  if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
  fi
  rm -f "${server_log}" "${adapter_log}"
}
trap cleanup EXIT

server_args=(
  --server-name "${instance_name}"
  --model "${model_path}"
  --max-ticks 0
  --log-level trace
)
if [[ "${mode}" == "headless" ]]; then
  env -u DISPLAY -u WAYLAND_DISPLAY "${server_bin}" \
    "${server_args[@]}" --headless >"${server_log}" 2>&1 &
elif [[ "${mode}" == "gui" ]]; then
  "${server_bin}" "${server_args[@]}" >"${server_log}" 2>&1 &
else
  echo "unknown server mode: ${mode}" >&2
  exit 1
fi
server_pid="$!"

sleep 0.1

if ! timeout 12s "${adapter_bin}" \
    --server-name "${instance_name}" \
    --adapter-id "sample_${BASHPID}" \
    --ticks 5 \
    --command-value 0.1 \
    --service-timeout-ms 5000 \
    >"${adapter_log}" 2>&1; then
  echo "sample adapter failed" >&2
  sed -n '1,160p' "${adapter_log}" >&2 || true
  sed -n '1,160p' "${server_log}" >&2 || true
  exit 1
fi

if ! kill -0 "${server_pid}" 2>/dev/null; then
  echo "server stopped before the adapter completed" >&2
  sed -n '1,160p' "${server_log}" >&2 || true
  sed -n '1,160p' "${adapter_log}" >&2 || true
  exit 1
fi

grep -Fq "event=adapter_registered" "${server_log}"
grep -Fq "event=adapter_activated" "${server_log}"
grep -Fq "event=adapter_unregistered" "${server_log}"

kill "${server_pid}" 2>/dev/null || true
set +e
wait "${server_pid}"
server_status="$?"
set -e
server_pid=""
if [[ "${server_status}" -ne 143 ]]; then
  echo "expected server SIGTERM exit 143, got ${server_status}" >&2
  sed -n '1,160p' "${server_log}" >&2 || true
  exit 1
fi
