#!/usr/bin/env bash
set -euo pipefail

launcher="$(realpath "$1")"
fake_child="$(realpath "$2")"
test_root="$(mktemp -d /tmp/mujoco_sensor_demo_launcher_test.XXXXXX)"

cleanup() {
  if [[ -f "${test_root}/pids" ]]; then
    while read -r pid; do
      kill -KILL "${pid}" 2>/dev/null || true
    done <"${test_root}/pids"
  fi
  rm -rf "${test_root}"
}
trap cleanup EXIT

for role in server controller state camera mapper truth; do
  ln -s "${fake_child}" "${test_root}/fake-${role}"
done

assert_children_stopped() {
  while read -r pid; do
    if kill -0 "${pid}" 2>/dev/null; then
      echo "launcher left fake child ${pid} running" >&2
      exit 1
    fi
  done <"${test_root}/pids"
}

run_launcher() {
  local output_dir="$1"
  shift
  env \
    SENSOR_DEMO_LAUNCH_STARTUP_TIMEOUT_S=2 \
    SENSOR_DEMO_LAUNCH_GRACE_TIMEOUT_S=1 \
    SENSOR_DEMO_LAUNCH_TERM_TIMEOUT_S=1 \
    SENSOR_DEMO_LAUNCH_KILL_TIMEOUT_S=1 \
    FAKE_SENSOR_DEMO_STATE_DIR="${test_root}" \
    "$@" \
    timeout --kill-after=2s 10s bash "${launcher}" \
      --server-bin "${test_root}/fake-server" \
      --controller-bin "${test_root}/fake-controller" \
      --state-bin "${test_root}/fake-state" \
      --camera-bin "${test_root}/fake-camera" \
      --mapper-bin "${test_root}/fake-mapper" \
      --truth-bin "${test_root}/fake-truth" \
      --server-config "${test_root}/server.yaml" \
      --demo-config "${test_root}/demo.yaml" \
      --server-name "fake_${BASHPID}_$(date +%s%N)" \
      --output-dir "${output_dir}" \
      --timeout-s 1 \
      --mode headless
}

ignore_output="${test_root}/ignore-output"
set +e
run_launcher "${ignore_output}" \
  FAKE_SENSOR_DEMO_IGNORE_SIGNALS=1 \
  FAKE_SENSOR_DEMO_CONTROLLER_HANG=1 \
  >"${test_root}/ignore.stdout" 2>"${test_root}/ignore.stderr"
ignore_status="$?"
set -e
if ((ignore_status == 0 || ignore_status == 124 || ignore_status == 137)); then
  echo "launcher did not bound signal-ignoring children: exit=${ignore_status}" >&2
  sed -n '1,240p' "${test_root}/ignore.stderr" >&2
  exit 1
fi
grep -Fq "sensor demo controller did not complete before timeout" \
  "${test_root}/ignore.stderr"
grep -Fq "sending SIGTERM" "${test_root}/ignore.stderr"
grep -Fq "sending SIGKILL" "${test_root}/ignore.stderr"
assert_children_stopped

rm -f "${test_root}/controller.done" "${test_root}/pids"
server_output="${test_root}/server-output"
set +e
run_launcher "${server_output}" \
  FAKE_SENSOR_DEMO_SERVER_EXIT_AFTER_CONTROLLER=1 \
  FAKE_SENSOR_DEMO_SHUTDOWN_DELAY_S=1 \
  >"${test_root}/server.stdout" 2>"${test_root}/server.stderr"
server_status="$?"
set -e
if ((server_status == 0 || server_status == 124 || server_status == 137)); then
  echo "launcher did not report the post-controller server failure: exit=${server_status}" >&2
  sed -n '1,240p' "${test_root}/server.stderr" >&2
  exit 1
fi
grep -Fq \
  "sensor demo server exited before expected termination during consumer shutdown with exit 42" \
  "${test_root}/server.stderr"
assert_children_stopped

rm -f "${test_root}/controller.done" "${test_root}/pids"
shutdown_output="${test_root}/shutdown-output"
set +e
run_launcher "${shutdown_output}" \
  FAKE_SENSOR_DEMO_SERVER_SHUTDOWN_EXIT_CODE=42 \
  >"${test_root}/shutdown.stdout" 2>"${test_root}/shutdown.stderr"
shutdown_status="$?"
set -e
if ((shutdown_status == 0 || shutdown_status == 124 ||
     shutdown_status == 137)); then
  echo "launcher ignored the expected-shutdown server status: exit=${shutdown_status}" >&2
  sed -n '1,240p' "${test_root}/shutdown.stderr" >&2
  exit 1
fi
grep -Fq "sensor demo server failed during shutdown with exit 42" \
  "${test_root}/shutdown.stderr"
assert_children_stopped

rm -f "${test_root}/controller.done" "${test_root}/pids"
signal_output="${test_root}/signal-output"
if ! run_launcher "${signal_output}" \
  FAKE_SENSOR_DEMO_SERVER_DEFAULT_TERM=1 \
  >"${test_root}/signal.stdout" 2>"${test_root}/signal.stderr"; then
  echo "launcher rejected the expected SIGTERM server status" >&2
  sed -n '1,240p' "${test_root}/signal.stderr" >&2
  exit 1
fi
grep -Fq "sensor demo completed" "${test_root}/signal.stdout"
assert_children_stopped

echo "sensor demo launcher lifecycle tests passed"
