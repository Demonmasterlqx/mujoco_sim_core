#!/usr/bin/env bash
set -uo pipefail

server_bin="mujoco-sim-server"
controller_bin="mujoco-sim-sensor-demo-controller"
state_bin="mujoco-sim-sensor-demo-state-observer"
camera_bin="mujoco-sim-sensor-demo-camera-consumer"
mapper_bin="mujoco-sim-sensor-demo-mapper"
truth_bin=""
server_config=""
demo_config=""
output_dir="sensor_demo_output"
server_name="sensor_demo_${BASHPID}_$(date +%s%N)"
mode="headless"
timeout_s=180

startup_timeout_s="${SENSOR_DEMO_LAUNCH_STARTUP_TIMEOUT_S:-20}"
grace_timeout_s="${SENSOR_DEMO_LAUNCH_GRACE_TIMEOUT_S:-10}"
term_timeout_s="${SENSOR_DEMO_LAUNCH_TERM_TIMEOUT_S:-5}"
kill_timeout_s="${SENSOR_DEMO_LAUNCH_KILL_TIMEOUT_S:-2}"

usage() {
  cat <<'EOF'
Usage: mujoco-sim-sensor-demo-launch [options]
  --server-config PATH   Server YAML (required)
  --demo-config PATH     Demo YAML (required)
  --output-dir PATH      Map, trajectory, screenshot, and logs
  --server-name NAME     Isolated eCAL server namespace
  --mode MODE            headless or gui (default: headless)
  --timeout-s SECONDS    Controller completion deadline
  --server-bin PATH      Override mujoco-sim-server executable
  --controller-bin PATH  Override controller executable
  --state-bin PATH       Override state observer executable
  --camera-bin PATH      Override camera consumer executable
  --mapper-bin PATH      Override mapper executable
  --truth-bin PATH       Optional truth recorder executable for validation
EOF
}

while (($# > 0)); do
  case "$1" in
    --server-config) server_config="$2"; shift 2 ;;
    --demo-config) demo_config="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    --server-name) server_name="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --timeout-s) timeout_s="$2"; shift 2 ;;
    --server-bin) server_bin="$2"; shift 2 ;;
    --controller-bin) controller_bin="$2"; shift 2 ;;
    --state-bin) state_bin="$2"; shift 2 ;;
    --camera-bin) camera_bin="$2"; shift 2 ;;
    --mapper-bin) mapper_bin="$2"; shift 2 ;;
    --truth-bin) truth_bin="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${server_config}" || -z "${demo_config}" ]]; then
  echo "--server-config and --demo-config are required" >&2
  usage >&2
  exit 2
fi
if [[ "${mode}" != "headless" && "${mode}" != "gui" ]]; then
  echo "--mode must be headless or gui" >&2
  exit 2
fi
if ! [[ "${timeout_s}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--timeout-s must be a positive integer" >&2
  exit 2
fi
for timeout_variable in startup_timeout_s grace_timeout_s term_timeout_s \
                        kill_timeout_s; do
  if ! [[ "${!timeout_variable}" =~ ^[1-9][0-9]*$ ]]; then
    echo "${timeout_variable} must be a positive integer" >&2
    exit 2
  fi
done

mkdir -p "${output_dir}/logs" || exit 1
declare -a pids=()
declare -A child_names=()
declare -A child_statuses=()
server_pid=""
controller_pid=""
truth_pid=""
cleanup_started=0
premature_server_status=""
shutdown_sent_int=0
shutdown_sent_term=0

show_logs() {
  local log
  for log in "${output_dir}"/logs/*.log; do
    [[ -e "${log}" ]] || continue
    echo "===== ${log} =====" >&2
    sed -n '1,240p' "${log}" >&2 || true
  done
}

register_child() {
  local pid="$1"
  local name="$2"
  pids+=("${pid}")
  child_names["${pid}"]="${name}"
}

reap_if_exited() {
  local pid="$1"
  local status=0
  if [[ -n "${child_statuses[${pid}]+recorded}" ]]; then
    return 0
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    return 1
  fi
  wait "${pid}" 2>/dev/null || status="$?"
  child_statuses["${pid}"]="${status}"
  return 0
}

signal_children() {
  local signal="$1"
  shift
  local pid
  for pid in "$@"; do
    if ! reap_if_exited "${pid}"; then
      kill "-${signal}" "${pid}" 2>/dev/null || true
    fi
  done
}

# Returns 0 when all targets exit, 1 on timeout, and 2 if a monitored server
# exits before the launcher has started its expected server shutdown.
wait_for_children() {
  local timeout="$1"
  local monitor_server="$2"
  shift 2
  local -a targets=("$@")
  local deadline=$((SECONDS + timeout))
  local all_exited
  local pid
  while true; do
    all_exited=1
    for pid in "${targets[@]}"; do
      if ! reap_if_exited "${pid}"; then
        all_exited=0
      fi
    done
    if [[ "${monitor_server}" == "yes" ]] &&
       reap_if_exited "${server_pid}"; then
      premature_server_status="${child_statuses[${server_pid}]}"
      return 2
    fi
    ((all_exited == 1)) && return 0
    ((SECONDS >= deadline)) && return 1
    sleep 0.05
  done
}

# All shutdown stages are bounded. SIGINT lets demo processes flush outputs,
# then SIGTERM and SIGKILL ensure an uncooperative child cannot hang launcher
# teardown indefinitely.
shutdown_children() {
  local description="$1"
  local monitor_server="$2"
  shift 2
  local -a targets=("$@")
  local status
  ((${#targets[@]} > 0)) || return 0

  shutdown_sent_int=1
  shutdown_sent_term=0
  signal_children INT "${targets[@]}"
  wait_for_children "${grace_timeout_s}" "${monitor_server}" \
    "${targets[@]}"
  status="$?"
  ((status == 0)) && return 0
  ((status == 2)) && return 2

  echo "${description} did not finish after SIGINT; sending SIGTERM" >&2
  shutdown_sent_term=1
  signal_children TERM "${targets[@]}"
  wait_for_children "${term_timeout_s}" "${monitor_server}" \
    "${targets[@]}"
  status="$?"
  ((status == 0)) && return 0
  ((status == 2)) && return 2

  echo "${description} did not finish after SIGTERM; sending SIGKILL" >&2
  signal_children KILL "${targets[@]}"
  wait_for_children "${kill_timeout_s}" "${monitor_server}" \
    "${targets[@]}"
  status="$?"
  ((status == 0)) && return 0
  ((status == 2)) && return 2
  echo "${description} remained alive after SIGKILL" >&2
  return 1
}

cleanup() {
  ((cleanup_started == 0)) || return
  cleanup_started=1
  trap '' INT TERM
  shutdown_children "sensor demo cleanup" no "${pids[@]}" || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for_log() {
  local pid="$1"
  local log="$2"
  local pattern="$3"
  local deadline=$((SECONDS + startup_timeout_s))
  while ((SECONDS < deadline)); do
    if grep -Fq "${pattern}" "${log}" 2>/dev/null; then
      return 0
    fi
    if reap_if_exited "${pid}"; then
      return 1
    fi
    sleep 0.05
  done
  return 1
}

server_log="${output_dir}/logs/server.log"
server_args=(--config "${server_config}" --server-name "${server_name}" --max-ticks 0)
if [[ "${mode}" == "headless" ]]; then
  env -u DISPLAY -u WAYLAND_DISPLAY "${server_bin}" \
    "${server_args[@]}" --headless >"${server_log}" 2>&1 &
else
  "${server_bin}" "${server_args[@]}" --gui >"${server_log}" 2>&1 &
fi
server_pid="$!"
register_child "${server_pid}" server
if ! wait_for_log "${server_pid}" "${server_log}" "event=server_started"; then
  echo "sensor demo server did not become ready" >&2
  show_logs
  exit 1
fi

common_args=(--config "${demo_config}" --server-name "${server_name}" --output-dir "${output_dir}")

if [[ -n "${truth_bin}" ]]; then
  truth_log="${output_dir}/logs/truth_recorder.log"
  "${truth_bin}" "${server_name}" "${output_dir}/truth.csv" \
    >"${truth_log}" 2>&1 &
  truth_pid="$!"
  register_child "${truth_pid}" truth_recorder
  if ! wait_for_log "${truth_pid}" "${truth_log}" "truth recorder ready"; then
    echo "truth recorder did not become ready" >&2
    show_logs
    exit 1
  fi
fi

state_log="${output_dir}/logs/state_observer.log"
"${state_bin}" "${common_args[@]}" >"${state_log}" 2>&1 &
state_pid="$!"
register_child "${state_pid}" state_observer

camera_log="${output_dir}/logs/camera_consumer.log"
"${camera_bin}" "${common_args[@]}" >"${camera_log}" 2>&1 &
camera_pid="$!"
register_child "${camera_pid}" camera_consumer

mapper_log="${output_dir}/logs/mapper.log"
"${mapper_bin}" "${common_args[@]}" >"${mapper_log}" 2>&1 &
mapper_pid="$!"
register_child "${mapper_pid}" mapper

if ! wait_for_log "${state_pid}" "${state_log}" "state observer ready" ||
   ! wait_for_log "${camera_pid}" "${camera_log}" "camera consumer ready" ||
   ! wait_for_log "${mapper_pid}" "${mapper_log}" "mapper ready"; then
  echo "one or more sensor demo consumers did not become ready" >&2
  show_logs
  exit 1
fi

controller_log="${output_dir}/logs/controller.log"
"${controller_bin}" "${common_args[@]}" >"${controller_log}" 2>&1 &
controller_pid="$!"
register_child "${controller_pid}" controller

deadline=$((SECONDS + timeout_s))
while ! reap_if_exited "${controller_pid}"; do
  if ((SECONDS >= deadline)); then
    echo "sensor demo controller did not complete before timeout" >&2
    show_logs
    exit 1
  fi
  dependency_pids=("${state_pid}" "${camera_pid}" "${mapper_pid}" "${server_pid}")
  if [[ -n "${truth_pid}" ]]; then dependency_pids+=("${truth_pid}"); fi
  for pid in "${dependency_pids[@]}"; do
    if reap_if_exited "${pid}"; then
      echo "sensor demo dependency ${child_names[${pid}]} exited before the controller with exit ${child_statuses[${pid}]}" >&2
      show_logs
      exit 1
    fi
  done
  sleep 0.1
done
controller_status="${child_statuses[${controller_pid}]}"
if ((controller_status != 0)); then
  echo "sensor demo controller failed with exit ${controller_status}" >&2
  show_logs
  exit 1
fi

consumer_pids=("${state_pid}" "${camera_pid}" "${mapper_pid}")
if [[ -n "${truth_pid}" ]]; then consumer_pids+=("${truth_pid}"); fi
shutdown_children "sensor demo consumers" yes "${consumer_pids[@]}"
shutdown_status="$?"
if ((shutdown_status == 2)); then
  echo "sensor demo server exited before expected termination during consumer shutdown with exit ${premature_server_status}" >&2
  show_logs
  exit 1
fi
if ((shutdown_status != 0)); then
  echo "sensor demo consumers could not be stopped" >&2
  show_logs
  exit 1
fi

client_status=0
for pid in "${consumer_pids[@]}"; do
  status="${child_statuses[${pid}]}"
  if ((status != 0)); then
    echo "sensor demo consumer ${child_names[${pid}]} failed during shutdown with exit ${status}" >&2
    client_status=1
  fi
done
if ((client_status != 0)); then
  show_logs
  exit 1
fi

if reap_if_exited "${server_pid}"; then
  echo "sensor demo server exited before expected termination with exit ${child_statuses[${server_pid}]}" >&2
  show_logs
  exit 1
fi
shutdown_children "sensor demo server" no "${server_pid}"
shutdown_status="$?"
if ((shutdown_status != 0)); then
  echo "sensor demo server could not be stopped" >&2
  show_logs
  exit 1
fi
server_status="${child_statuses[${server_pid}]}"
# A process terminated by the launcher's expected shutdown signal is a
# successful server stop. Any other non-zero status remains a server failure.
if ((server_status != 0)) &&
   ! ((server_status == 130 && shutdown_sent_int == 1)) &&
   ! ((server_status == 143 && shutdown_sent_term == 1)); then
  echo "sensor demo server failed during shutdown with exit ${server_status}" >&2
  show_logs
  exit 1
fi

cleanup_started=1
echo "sensor demo completed output=${output_dir} server=${server_name}"
