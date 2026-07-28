#!/usr/bin/env bash
set -uo pipefail

server_bin="mujoco-sim-server"
wbr_bin="mujoco-sim-wbr-lqr-adapter"
double_bin="mujoco-sim-double-inverted-control-demo"
server_config=""
wbr_config=""
double_config=""
scene=""
output_dir="dual_adapter_demo_output"
server_name="dual_adapter_${BASHPID}_$(date +%s%N)"
mode="headless"
wbr_ticks=0
timeout_s=0
startup_timeout_s="${DUAL_ADAPTER_STARTUP_TIMEOUT_S:-20}"
grace_timeout_s="${DUAL_ADAPTER_GRACE_TIMEOUT_S:-3}"
term_timeout_s="${DUAL_ADAPTER_TERM_TIMEOUT_S:-3}"
kill_timeout_s="${DUAL_ADAPTER_KILL_TIMEOUT_S:-2}"

usage() {
  cat <<'EOF'
Usage: mujoco-sim-dual-adapter-demo-launch [options]
  --server-config PATH  Server YAML
  --wbr-config PATH     WBR LQR adapter YAML
  --double-config PATH  Double inverted pendulum adapter YAML
  --scene PATH          Combined MJCF scene
  --output-dir PATH     Persistent process logs
  --server-name NAME    Isolated eCAL Server namespace
  --mode MODE           headless or gui (default: headless)
  --wbr-ticks COUNT     Stop successfully after this many WBR State samples
  --timeout-s SECONDS   Overall deadline; zero means unlimited
  --server-bin PATH     Override mujoco-sim-server executable
  --wbr-bin PATH        Override WBR adapter executable
  --double-bin PATH     Override double inverted adapter executable
EOF
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
install_prefix="$(dirname -- "${script_dir}")"
share_dir="${install_prefix}/share/mujoco_sim_dual_adapter_demo"
if [[ -d "${share_dir}" ]]; then
  server_config="${share_dir}/config/dual_adapter_server.yaml"
  wbr_config="${share_dir}/config/wbr_lqr_adapter.yaml"
  double_config="${share_dir}/config/double_inverted_adapter.yaml"
  scene="${share_dir}/mjcf/scene.xml"
fi

while (($# > 0)); do
  case "$1" in
    --server-config) server_config="$2"; shift 2 ;;
    --wbr-config) wbr_config="$2"; shift 2 ;;
    --double-config) double_config="$2"; shift 2 ;;
    --scene) scene="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    --server-name) server_name="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --wbr-ticks) wbr_ticks="$2"; shift 2 ;;
    --timeout-s) timeout_s="$2"; shift 2 ;;
    --server-bin) server_bin="$2"; shift 2 ;;
    --wbr-bin) wbr_bin="$2"; shift 2 ;;
    --double-bin) double_bin="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for required_variable in server_config wbr_config double_config scene; do
  required_path="${!required_variable}"
  if [[ -z "${required_path}" || ! -f "${required_path}" ]]; then
    echo "${required_variable} does not name a readable file: ${required_path}" >&2
    exit 2
  fi
done
if [[ "${mode}" != "headless" && "${mode}" != "gui" ]]; then
  echo "--mode must be headless or gui" >&2
  exit 2
fi
for numeric_variable in wbr_ticks timeout_s; do
  if ! [[ "${!numeric_variable}" =~ ^[0-9]+$ ]]; then
    echo "${numeric_variable} must be a nonnegative integer" >&2
    exit 2
  fi
done
for numeric_variable in startup_timeout_s grace_timeout_s term_timeout_s kill_timeout_s; do
  if ! [[ "${!numeric_variable}" =~ ^[1-9][0-9]*$ ]]; then
    echo "${numeric_variable} must be a positive integer" >&2
    exit 2
  fi
done

mkdir -p "${output_dir}/logs" || exit 1
server_log="${output_dir}/logs/server.log"
wbr_log="${output_dir}/logs/wbr_adapter.log"
double_log="${output_dir}/logs/double_inverted_adapter.log"

declare -a pids=()
declare -A child_names=()
declare -A child_statuses=()
server_pid=""
wbr_pid=""
double_pid=""
cleanup_started=0

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

wait_until_exited() {
  local pid="$1"
  local seconds="$2"
  local deadline=$((SECONDS + seconds))
  while ((SECONDS < deadline)); do
    reap_if_exited "${pid}" && return 0
    sleep 0.05
  done
  reap_if_exited "${pid}"
}

stop_child() {
  local pid="$1"
  local name="$2"
  reap_if_exited "${pid}" && return 0
  kill -INT "${pid}" 2>/dev/null || true
  wait_until_exited "${pid}" "${grace_timeout_s}" && return 0
  echo "${name} did not stop after SIGINT; sending SIGTERM" >&2
  kill -TERM "${pid}" 2>/dev/null || true
  wait_until_exited "${pid}" "${term_timeout_s}" && return 0
  echo "${name} did not stop after SIGTERM; sending SIGKILL" >&2
  kill -KILL "${pid}" 2>/dev/null || true
  if ! wait_until_exited "${pid}" "${kill_timeout_s}"; then
    echo "${name} remained alive after SIGKILL" >&2
    return 1
  fi
}

show_logs() {
  local log
  for log in "${server_log}" "${wbr_log}" "${double_log}"; do
    [[ -f "${log}" ]] || continue
    echo "===== ${log} =====" >&2
    sed -n '1,240p' "${log}" >&2 || true
  done
}

cleanup() {
  ((cleanup_started == 0)) || return
  cleanup_started=1
  trap '' INT TERM
  if [[ -n "${wbr_pid}" ]]; then stop_child "${wbr_pid}" wbr_adapter || true; fi
  if [[ -n "${double_pid}" ]]; then stop_child "${double_pid}" double_adapter || true; fi
  if [[ -n "${server_pid}" ]]; then stop_child "${server_pid}" server || true; fi
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
    grep -Fq "${pattern}" "${log}" 2>/dev/null && return 0
    reap_if_exited "${pid}" && return 1
    sleep 0.05
  done
  return 1
}

server_args=(
  --config "${server_config}"
  --server-name "${server_name}"
  --model "${scene}"
  --initial-keyframe dual_home
  --max-ticks 0
)
if [[ "${mode}" == "headless" ]]; then
  env -u DISPLAY -u WAYLAND_DISPLAY "${server_bin}" \
    "${server_args[@]}" --headless >"${server_log}" 2>&1 &
else
  "${server_bin}" "${server_args[@]}" --gui >"${server_log}" 2>&1 &
fi
server_pid="$!"
register_child "${server_pid}" server

if ! wait_for_log "${server_pid}" "${server_log}" "event=server_started"; then
  echo "dual-adapter Server did not become ready" >&2
  show_logs
  exit 1
fi

"${double_bin}" \
  --config "${double_config}" \
  --server-name "${server_name}" \
  --adapter-id double_inverted_controller \
  >"${double_log}" 2>&1 &
double_pid="$!"
register_child "${double_pid}" double_adapter

wbr_args=(
  --config "${wbr_config}"
  --server-name "${server_name}"
  --adapter-id wbr_lqr_controller
  --ticks "${wbr_ticks}"
)
if ((wbr_ticks > 0)); then wbr_args+=(--log-every-ticks 0); fi
"${wbr_bin}" "${wbr_args[@]}" >"${wbr_log}" 2>&1 &
wbr_pid="$!"
register_child "${wbr_pid}" wbr_adapter

deadline=0
if ((timeout_s > 0)); then deadline=$((SECONDS + timeout_s)); fi
while true; do
  if reap_if_exited "${server_pid}"; then
    echo "Server exited before the dual-adapter run completed with exit ${child_statuses[${server_pid}]}" >&2
    show_logs
    exit 1
  fi
  if reap_if_exited "${double_pid}"; then
    echo "double inverted adapter exited early with exit ${child_statuses[${double_pid}]}" >&2
    show_logs
    exit 1
  fi
  if reap_if_exited "${wbr_pid}"; then
    wbr_status="${child_statuses[${wbr_pid}]}"
    if ((wbr_ticks == 0 || wbr_status != 0)); then
      echo "WBR adapter exited unexpectedly with exit ${wbr_status}" >&2
      show_logs
      exit 1
    fi
    break
  fi
  if ((deadline > 0 && SECONDS >= deadline)); then
    echo "dual-adapter run exceeded --timeout-s ${timeout_s}" >&2
    show_logs
    exit 1
  fi
  sleep 0.1
done

for adapter_id in wbr_lqr_controller double_inverted_controller; do
  if ! grep -F "event=adapter_registered" "${server_log}" | grep -Fq "${adapter_id}" ||
     ! grep -F "event=adapter_activated" "${server_log}" | grep -Fq "${adapter_id}"; then
    echo "Server log does not prove registration and activation for ${adapter_id}" >&2
    show_logs
    exit 1
  fi
done
if ! grep -Fq "Command layout:" "${double_log}"; then
  echo "double inverted adapter did not validate its command layout" >&2
  show_logs
  exit 1
fi
if ! grep -Eq "wbr-lqr summary ticks=${wbr_ticks} .*nonzero_ticks=[1-9][0-9]*" \
    "${wbr_log}"; then
  echo "WBR adapter summary does not prove nonzero closed-loop control" >&2
  show_logs
  exit 1
fi
if ((wbr_ticks >= 5000)) &&
   ! grep -Eq "wbr-lqr summary .*final_fsm=normal" "${wbr_log}"; then
  echo "WBR adapter did not finish the standing run in NORMAL" >&2
  show_logs
  exit 1
fi

stop_child "${double_pid}" double_adapter || exit 1
stop_child "${server_pid}" server || exit 1
cleanup_started=1
echo "dual adapter demo completed output=${output_dir} server=${server_name} wbr_ticks=${wbr_ticks}"
