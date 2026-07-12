#!/usr/bin/env bash
set -uo pipefail

role="$(basename "$0")"
state_dir="${FAKE_SENSOR_DEMO_STATE_DIR:?FAKE_SENSOR_DEMO_STATE_DIR is required}"
mkdir -p "${state_dir}"
printf '%s\n' "${BASHPID}" >>"${state_dir}/pids"

shutdown_delay="${FAKE_SENSOR_DEMO_SHUTDOWN_DELAY_S:-0}"
shutdown() {
  trap - INT TERM
  sleep "${shutdown_delay}"
  if [[ "${role}" == "fake-server" ]]; then
    exit "${FAKE_SENSOR_DEMO_SERVER_SHUTDOWN_EXIT_CODE:-0}"
  fi
  exit 0
}

ignore_signals="${FAKE_SENSOR_DEMO_IGNORE_SIGNALS:-0}"
if [[ "${role}" == "fake-server" &&
      "${FAKE_SENSOR_DEMO_SERVER_IGNORE_SIGNALS:-0}" == "1" ]]; then
  ignore_signals=1
fi
if [[ "${role}" == "fake-camera" &&
      "${FAKE_SENSOR_DEMO_CAMERA_IGNORE_SIGNALS:-0}" == "1" ]]; then
  ignore_signals=1
fi
if [[ "${ignore_signals}" == "1" ]]; then
  trap '' INT TERM
else
  trap shutdown INT TERM
fi

case "${role}" in
  fake-server)
    if [[ "${FAKE_SENSOR_DEMO_SERVER_DEFAULT_TERM:-0}" == "1" ]]; then
      trap '' INT
      trap - TERM
    fi
    echo "event=server_started"
    while true; do
      if [[ "${FAKE_SENSOR_DEMO_SERVER_EXIT_AFTER_CONTROLLER:-0}" == "1" &&
            -e "${state_dir}/controller.done" ]]; then
        sleep "${FAKE_SENSOR_DEMO_SERVER_EXIT_DELAY_S:-0.2}"
        exit "${FAKE_SENSOR_DEMO_SERVER_EXIT_CODE:-42}"
      fi
      sleep 0.05
    done
    ;;
  fake-controller)
    if [[ "${FAKE_SENSOR_DEMO_CONTROLLER_HANG:-0}" == "1" ]]; then
      while true; do sleep 0.05; done
    fi
    sleep "${FAKE_SENSOR_DEMO_CONTROLLER_DELAY_S:-0.2}"
    touch "${state_dir}/controller.done"
    exit "${FAKE_SENSOR_DEMO_CONTROLLER_EXIT_CODE:-0}"
    ;;
  fake-state)
    echo "state observer ready"
    ;;
  fake-camera)
    if [[ "${FAKE_SENSOR_DEMO_CAMERA_GUI_SUCCESS:-0}" == "1" ]]; then
      output_dir=""
      while (($# > 0)); do
        case "$1" in
          --output-dir) output_dir="$2"; shift 2 ;;
          *) shift ;;
        esac
      done
      if [[ -z "${output_dir}" ]]; then
        echo "fake GUI camera requires --output-dir" >&2
        exit 2
      fi
      mkdir -p "${output_dir}"
      printf 'fake camera frame\n' >"${output_dir}/front_camera.png"
      echo "camera summary screenshot=1"
      exit 0
    fi
    echo "camera consumer ready"
    ;;
  fake-mapper)
    echo "mapper ready"
    ;;
  fake-truth)
    echo "truth recorder ready"
    ;;
  *)
    echo "unknown fake sensor demo role: ${role}" >&2
    exit 2
    ;;
esac

while true; do sleep 0.05; done
