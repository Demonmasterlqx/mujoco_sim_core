#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
adapter_bin="$2"
server_config="$3"
adapter_config="$4"
mode="${5:-headless}"
ticks="${6:-600}"
input_mode="${7:-script}"
expected_adapter_result="${8:-success}"

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

adapter_args=(
  --config "${adapter_config}"
  --server-name "${instance_name}"
  --adapter-id "wbr_${BASHPID}"
  --ticks "${ticks}"
  --log-every-ticks 0
  --input-mode "${input_mode}"
)
set +e
timeout 45s "${adapter_bin}" "${adapter_args[@]}" >"${adapter_log}" 2>&1
adapter_status="$?"
set -e
if [[ "${expected_adapter_result}" == "success" && "${adapter_status}" -ne 0 ]]; then
  echo "WBR adapter failed" >&2
  sed -n '1,200p' "${adapter_log}" >&2 || true
  sed -n '1,200p' "${server_log}" >&2 || true
  exit 1
fi
if [[ "${expected_adapter_result}" == "failure" && "${adapter_status}" -ne 1 ]]; then
  echo "WBR adapter returned ${adapter_status}; expected application failure status 1" >&2
  sed -n '1,200p' "${adapter_log}" >&2 || true
  exit 1
fi
if [[ "${expected_adapter_result}" != "success" &&
      "${expected_adapter_result}" != "failure" ]]; then
  echo "unknown expected adapter result: ${expected_adapter_result}" >&2
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

for event in server_starting server_started \
             adapter_registered adapter_activated \
             adapter_unregistered; do
  if ! grep -Fq "event=${event}" "${server_log}"; then
    echo "server log is missing event=${event}" >&2
    sed -n '1,240p' "${server_log}" >&2 || true
    exit 1
  fi
done

if [[ "${expected_adapter_result}" == "failure" ]]; then
  if ! grep -Fq "terminal input disconnected" "${adapter_log}"; then
    echo "WBR adapter did not report the injected terminal disconnect" >&2
    sed -n '1,200p' "${adapter_log}" >&2 || true
    exit 1
  fi
  exit 0
fi
if grep -Eq "event=(state_published|command_accepted)" "${server_log}"; then
  echo "info server logging contains per-State or per-Command traffic" >&2
  exit 1
fi
if grep -Fq "event=command_duplicate" "${server_log}"; then
  echo "normal duplicate Command rejection leaked into info logs" >&2
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

if [[ "${input_mode}" == "cin" ]] &&
   ! grep -Eq "forward_effects=[1-9][0-9]*" "${adapter_log}"; then
  echo "WBR adapter summary did not prove the timed W direction effect" >&2
  sed -n '1,200p' "${adapter_log}" >&2 || true
  exit 1
fi

if (( ticks > 2000 )) &&
   grep -Eq '^[[:space:]]*-[[:space:]]*\{begin_tick:[[:space:]]*2000,.*e:[[:space:]]*true' \
     "${adapter_config}"; then
  if ! grep -Eq "e_effects=[1-9][0-9]*" \
      "${adapter_log}"; then
    echo "WBR adapter summary did not prove the E stand-height script effect" >&2
    sed -n '1,200p' "${adapter_log}" >&2 || true
    exit 1
  fi
fi

if (( ticks >= 5000 )); then
  if ! grep -Eq "wbr-lqr summary .* final_fsm=normal" "${adapter_log}"; then
    echo "WBR adapter did not finish the stand-up run in NORMAL" >&2
    sed -n '1,200p' "${adapter_log}" >&2 || true
    exit 1
  fi
fi
