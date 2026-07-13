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
  --max-ticks 0
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

sleep 0.1

if ! timeout 45s "${adapter_bin}" \
    --config "${adapter_config}" \
    --server-name "${instance_name}" \
    --adapter-id "wbr_${BASHPID}" \
    --ticks "${ticks}" \
    --log-every-ticks 0 \
    >"${adapter_log}" 2>&1; then
  echo "WBR adapter failed" >&2
  sed -n '1,200p' "${adapter_log}" >&2 || true
  sed -n '1,200p' "${server_log}" >&2 || true
  exit 1
fi

if ! kill -0 "${server_pid}" 2>/dev/null; then
  echo "WBR server stopped before the adapter completed" >&2
  sed -n '1,200p' "${server_log}" >&2 || true
  sed -n '1,200p' "${adapter_log}" >&2 || true
  exit 1
fi

kill "${server_pid}" 2>/dev/null || true
set +e
wait "${server_pid}"
server_status="$?"
set -e
server_pid=""
if [[ "${server_status}" -ne 143 ]]; then
  echo "expected WBR server SIGTERM exit 143, got ${server_status}" >&2
  sed -n '1,200p' "${server_log}" >&2 || true
  exit 1
fi

for event in server_starting model_loaded server_started \
             adapter_registered adapter_activated \
             adapter_unregistered; do
  if ! grep -Fq "event=${event}" "${server_log}"; then
    echo "server log is missing event=${event}" >&2
    sed -n '1,240p' "${server_log}" >&2 || true
    exit 1
  fi
done
if grep -Eq "event=(tick_published|commit_accepted)" "${server_log}"; then
  echo "info server logging contains per-Tick or per-Commit traffic" >&2
  exit 1
fi
if grep -Fq "event=commit_duplicate" "${server_log}"; then
  echo "normal idempotent Commit replay leaked into info logs" >&2
  exit 1
fi

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
