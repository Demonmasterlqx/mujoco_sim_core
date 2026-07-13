#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
probe_bin="$2"
model_path="$3"
instance_name="adapter_sdk_e2e_${BASHPID}_$(date +%s%N)"
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

env -u DISPLAY -u WAYLAND_DISPLAY "${server_bin}" \
  --headless \
  --server-name "${instance_name}" \
  --model "${model_path}" \
  --max-ticks 0 \
  --tick-timeout-ms 50 \
  --activation-timeout-ms 1000 \
  --log-level debug \
  >"${work_dir}/server.out" 2>"${work_dir}/server.err" &
server_pid="$!"

timeout 12s "${probe_bin}" "${instance_name}"

for event in adapter_registered adapter_activated \
             adapter_timed_out adapter_recovery_pending \
             adapter_recovered adapter_unregistered; do
  if ! grep -Fq "event=${event}" "${work_dir}/server.err"; then
    echo "server log is missing event=${event}" >&2
    sed -n '1,240p' "${work_dir}/server.err" >&2 || true
    exit 1
  fi
done

if ! kill -0 "${server_pid}" 2>/dev/null; then
  echo "adapter timeout stopped the server" >&2
  sed -n '1,240p' "${work_dir}/server.err" >&2 || true
  exit 1
fi

kill "${server_pid}" 2>/dev/null || true
wait "${server_pid}" 2>/dev/null || true
server_pid=""
