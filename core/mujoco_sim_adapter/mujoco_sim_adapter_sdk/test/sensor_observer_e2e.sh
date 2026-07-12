#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
probe_bin="$2"
model_path="$3"
config_path="$4"
instance_name="sensor_observer_e2e_${BASHPID}_$(date +%s%N)"
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
  --config "${config_path}" \
  --server-name "${instance_name}" \
  --model "${model_path}" \
  >"${work_dir}/server.out" 2>"${work_dir}/server.err" &
server_pid="$!"

if ! timeout 40s "${probe_bin}" "${instance_name}"; then
  echo "observer/stream E2E probe failed" >&2
  sed -n '1,260p' "${work_dir}/server.err" >&2 || true
  exit 1
fi

for event in adapter_registered adapter_activated adapter_unregistered; do
  if ! grep -Fq "event=${event}" "${work_dir}/server.err"; then
    echo "server log is missing event=${event}" >&2
    sed -n '1,260p' "${work_dir}/server.err" >&2 || true
    exit 1
  fi
done
for event in controller_timed_out controller_unregistered \
             controller_local_error; do
  if ! grep -Fq "event=${event}" "${work_dir}/server.err"; then
    echo "server log is missing event=${event}" >&2
    sed -n '1,260p' "${work_dir}/server.err" >&2 || true
    exit 1
  fi
done
if ! grep -Fq "role=OBSERVER" "${work_dir}/server.err"; then
  echo "server did not log an Observer lifecycle" >&2
  sed -n '1,260p' "${work_dir}/server.err" >&2 || true
  exit 1
fi
if ! kill -0 "${server_pid}" 2>/dev/null; then
  echo "sensor or Observer activity stopped the server" >&2
  sed -n '1,260p' "${work_dir}/server.err" >&2 || true
  exit 1
fi

kill "${server_pid}" 2>/dev/null || true
wait "${server_pid}" 2>/dev/null || true
server_pid=""
