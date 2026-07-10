#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
adapter_bin="$2"
server_config="$3"
adapter_config="$4"
mode="${5:-headless}"
ticks="${6:-600}"

instance_name="wbr_e2e_${BASHPID}_$(date +%s%N)"
server_log="/tmp/mujoco_sim_wbr_server_${instance_name}.log"
adapter_log="/tmp/mujoco_sim_wbr_adapter_${instance_name}.log"
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
  --config "${server_config}"
  --server-name "${instance_name}"
  --max-ticks "${ticks}"
  --startup-timeout-ms 10000
  --tick-timeout-ms 5000
)
if [[ "${mode}" == "headless" ]]; then
  env -u DISPLAY -u WAYLAND_DISPLAY "${server_bin}" \
    "${server_args[@]}" --headless >"${server_log}" 2>&1 &
elif [[ "${mode}" == "gui" ]]; then
  "${server_bin}" "${server_args[@]}" --gui >"${server_log}" 2>&1 &
else
  echo "unknown server mode: ${mode}" >&2
  exit 1
fi
server_pid="$!"

sleep 0.75

if ! timeout 45s "${adapter_bin}" \
    --config "${adapter_config}" \
    --server-name "${instance_name}" \
    --controller-id "wbr_${BASHPID}" \
    --ticks "${ticks}" \
    --log-every-ticks 0 \
    >"${adapter_log}" 2>&1; then
  echo "WBR adapter failed" >&2
  sed -n '1,200p' "${adapter_log}" >&2 || true
  sed -n '1,200p' "${server_log}" >&2 || true
  exit 1
fi

if ! wait "${server_pid}"; then
  echo "WBR server failed" >&2
  sed -n '1,200p' "${server_log}" >&2 || true
  sed -n '1,200p' "${adapter_log}" >&2 || true
  exit 1
fi
server_pid=""

if ! grep -Eq "wbr-lqr summary ticks=${ticks} .*nonzero_ticks=[1-9][0-9]*" \
    "${adapter_log}"; then
  echo "WBR adapter summary did not prove closed-loop control" >&2
  sed -n '1,200p' "${adapter_log}" >&2 || true
  exit 1
fi

if ! grep -Eq "space_effects=[1-9][0-9]*" "${adapter_log}"; then
  echo "WBR adapter summary did not prove the Space script effect" >&2
  sed -n '1,200p' "${adapter_log}" >&2 || true
  exit 1
fi

if (( ticks > 4000 )); then
  if ! grep -Eq "e_effects=[1-9][0-9]* .*q_effects=[1-9][0-9]*" \
      "${adapter_log}"; then
    echo "WBR adapter summary did not prove the E/Q script effects" >&2
    sed -n '1,200p' "${adapter_log}" >&2 || true
    exit 1
  fi
fi
