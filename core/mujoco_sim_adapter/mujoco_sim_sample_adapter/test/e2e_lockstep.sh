#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
adapter_bin="$2"
model_path="$3"

instance_name="e2e_${BASHPID}_$(date +%s%N)"
server_log="/tmp/mujoco_sim_server_${instance_name}.log"
adapter_log="/tmp/mujoco_sim_adapter_${instance_name}.log"
server_pid=""

cleanup() {
  if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

"${server_bin}" \
  --server-name "${instance_name}" \
  --model "${model_path}" \
  --max-ticks 5 \
  --startup-timeout-ms 5000 \
  --tick-timeout-ms 1000 \
  >"${server_log}" 2>&1 &
server_pid="$!"

sleep 0.5

if ! timeout 12s "${adapter_bin}" \
    --server-name "${instance_name}" \
    --controller-id "sample_${BASHPID}" \
    --ticks 5 \
    --command-value 0.1 \
    --service-timeout-ms 5000 \
    >"${adapter_log}" 2>&1; then
  echo "sample adapter failed" >&2
  sed -n '1,160p' "${adapter_log}" >&2 || true
  sed -n '1,160p' "${server_log}" >&2 || true
  exit 1
fi

if ! wait "${server_pid}"; then
  echo "server failed" >&2
  sed -n '1,160p' "${server_log}" >&2 || true
  sed -n '1,160p' "${adapter_log}" >&2 || true
  exit 1
fi
server_pid=""
