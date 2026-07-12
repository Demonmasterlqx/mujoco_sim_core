#!/usr/bin/env bash
set -euo pipefail

gui_script="$(realpath "$1")"
fake_child="$(realpath "$2")"
test_root="$(mktemp -d /tmp/mujoco_sensor_gui_lifecycle.XXXXXX)"

cleanup() {
  if [[ -f "${test_root}/all_pids" ]]; then
    while read -r pid; do
      [[ -n "${pid}" ]] || continue
      kill -KILL "${pid}" 2>/dev/null || true
    done <"${test_root}/all_pids"
  fi
  rm -rf "${test_root}"
}
trap cleanup EXIT

ln -s "${fake_child}" "${test_root}/fake-server"
ln -s "${fake_child}" "${test_root}/fake-camera"

assert_no_live_children() {
  local state_dir="$1"
  local pid
  local deadline
  local failed=0
  cat "${state_dir}/pids" >>"${test_root}/all_pids"
  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    deadline=$((SECONDS + 3))
    while kill -0 "${pid}" 2>/dev/null && ((SECONDS < deadline)); do
      sleep 0.05
    done
    if kill -0 "${pid}" 2>/dev/null; then
      echo "GUI lifecycle test left process ${pid} running" >&2
      failed=1
    fi
  done <"${state_dir}/pids"
  ((failed == 0))
}

camera_state="${test_root}/camera_timeout"
mkdir -p "${camera_state}"
set +e
FAKE_SENSOR_DEMO_STATE_DIR="${camera_state}" \
FAKE_SENSOR_DEMO_CAMERA_IGNORE_SIGNALS=1 \
SENSOR_DEMO_GUI_CAMERA_TIMEOUT_S=1 \
SENSOR_DEMO_GUI_CAMERA_KILL_TIMEOUT_S=1 \
SENSOR_DEMO_GUI_SERVER_GRACE_TIMEOUT_S=1 \
SENSOR_DEMO_GUI_SERVER_TERM_TIMEOUT_S=1 \
SENSOR_DEMO_GUI_SERVER_KILL_TIMEOUT_S=1 \
timeout --kill-after=2s 12s bash "${gui_script}" \
  "${test_root}/fake-server" "${test_root}/fake-camera" \
  /dev/null /dev/null >"${test_root}/camera_timeout.log" 2>&1
camera_status="$?"
set -e
if ((camera_status == 0 || camera_status == 124 || camera_status == 137)); then
  cat "${test_root}/camera_timeout.log" >&2
  echo "uncooperative camera was not terminated by the inner timeout" >&2
  exit 1
fi
assert_no_live_children "${camera_state}"

server_state="${test_root}/server_escalation"
mkdir -p "${server_state}"
FAKE_SENSOR_DEMO_STATE_DIR="${server_state}" \
FAKE_SENSOR_DEMO_SERVER_IGNORE_SIGNALS=1 \
FAKE_SENSOR_DEMO_CAMERA_GUI_SUCCESS=1 \
SENSOR_DEMO_GUI_CAMERA_TIMEOUT_S=2 \
SENSOR_DEMO_GUI_CAMERA_KILL_TIMEOUT_S=1 \
SENSOR_DEMO_GUI_SERVER_GRACE_TIMEOUT_S=1 \
SENSOR_DEMO_GUI_SERVER_TERM_TIMEOUT_S=1 \
SENSOR_DEMO_GUI_SERVER_KILL_TIMEOUT_S=1 \
timeout --kill-after=2s 12s bash "${gui_script}" \
  "${test_root}/fake-server" "${test_root}/fake-camera" \
  /dev/null /dev/null >"${test_root}/server_escalation.log" 2>&1
assert_no_live_children "${server_state}"
