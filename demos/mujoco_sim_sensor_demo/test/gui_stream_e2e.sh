#!/usr/bin/env bash
set -euo pipefail

server_bin="$1"
camera_bin="$2"
server_config="$3"
demo_config="$4"

output_dir="$(mktemp -d /tmp/mujoco_sensor_demo_gui.XXXXXX)"
instance="sensor_gui_${BASHPID}_$(date +%s%N)"
server_log="${output_dir}/server.log"
camera_log="${output_dir}/camera.log"
server_pid=""
camera_timeout_s="${SENSOR_DEMO_GUI_CAMERA_TIMEOUT_S:-30}"
camera_kill_timeout_s="${SENSOR_DEMO_GUI_CAMERA_KILL_TIMEOUT_S:-5}"
server_grace_timeout_s="${SENSOR_DEMO_GUI_SERVER_GRACE_TIMEOUT_S:-5}"
server_term_timeout_s="${SENSOR_DEMO_GUI_SERVER_TERM_TIMEOUT_S:-3}"
server_kill_timeout_s="${SENSOR_DEMO_GUI_SERVER_KILL_TIMEOUT_S:-2}"

for timeout_variable in camera_timeout_s camera_kill_timeout_s \
                        server_grace_timeout_s server_term_timeout_s \
                        server_kill_timeout_s; do
  if ! [[ "${!timeout_variable}" =~ ^[1-9][0-9]*$ ]]; then
    echo "${timeout_variable} must be a positive integer" >&2
    exit 2
  fi
done

wait_for_server_exit() {
  local timeout_s="$1"
  local deadline=$((SECONDS + timeout_s))
  while kill -0 "${server_pid}" 2>/dev/null; do
    ((SECONDS >= deadline)) && return 1
    sleep 0.05
  done
  wait "${server_pid}" 2>/dev/null || true
  return 0
}

stop_server() {
  if [[ -z "${server_pid}" ]]; then
    return 0
  fi
  if ! kill -0 "${server_pid}" 2>/dev/null; then
    wait "${server_pid}" 2>/dev/null || true
    return 0
  fi

  kill -INT "${server_pid}" 2>/dev/null || true
  wait_for_server_exit "${server_grace_timeout_s}" && return 0
  kill -TERM "${server_pid}" 2>/dev/null || true
  wait_for_server_exit "${server_term_timeout_s}" && return 0
  kill -KILL "${server_pid}" 2>/dev/null || true
  wait_for_server_exit "${server_kill_timeout_s}"
}

cleanup() {
  local status="$?"
  trap - EXIT
  if ! stop_server; then
    echo "GUI test server could not be stopped" >&2
    status=1
  fi
  rm -rf "${output_dir}"
  exit "${status}"
}
trap cleanup EXIT

"${server_bin}" --config "${server_config}" --server-name "${instance}" \
  --max-ticks 0 --gui >"${server_log}" 2>&1 &
server_pid="$!"
deadline=$((SECONDS + 20))
while ! grep -Fq 'event=server_started' "${server_log}" 2>/dev/null; do
  if ! kill -0 "${server_pid}" 2>/dev/null || ((SECONDS >= deadline)); then
    sed -n '1,260p' "${server_log}" >&2 || true
    exit 1
  fi
  sleep 0.05
done

if ! timeout --kill-after="${camera_kill_timeout_s}s" \
    "${camera_timeout_s}s" "${camera_bin}" --config "${demo_config}" \
    --server-name "${instance}" --output-dir "${output_dir}" \
    --camera-max-samples 1 >"${camera_log}" 2>&1; then
  sed -n '1,260p' "${camera_log}" >&2 || true
  sed -n '1,260p' "${server_log}" >&2 || true
  exit 1
fi
grep -Fq 'camera summary screenshot=1' "${camera_log}"
test -s "${output_dir}/front_camera.png"
kill -0 "${server_pid}"
