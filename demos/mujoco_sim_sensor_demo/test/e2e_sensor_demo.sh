#!/usr/bin/env bash
set -euo pipefail

launcher="$1"
server_bin="$2"
controller_bin="$3"
state_bin="$4"
camera_bin="$5"
mapper_bin="$6"
server_config="$7"
demo_config="$8"
quality_score="$9"
truth_recorder="${10}"

output_dir="$(mktemp -d /tmp/mujoco_sensor_demo_e2e.XXXXXX)"
cleanup() {
  rm -rf "${output_dir}"
}
trap cleanup EXIT

instance="sensor_e2e_${BASHPID}_$(date +%s%N)"
if ! timeout --kill-after=18s 180s bash "${launcher}" \
    --server-bin "${server_bin}" \
    --controller-bin "${controller_bin}" \
    --state-bin "${state_bin}" \
    --camera-bin "${camera_bin}" \
    --mapper-bin "${mapper_bin}" \
    --truth-bin "${truth_recorder}" \
    --server-config "${server_config}" \
    --demo-config "${demo_config}" \
    --server-name "${instance}" \
    --output-dir "${output_dir}" \
    --timeout-s 150 \
    --mode headless; then
  for log in "${output_dir}"/logs/*.log; do
    echo "===== ${log} =====" >&2
    sed -n '1,260p' "${log}" >&2 || true
  done
  exit 1
fi

grep -Eq 'controller summary ticks=[1-9][0-9]* laps=1' \
  "${output_dir}/logs/controller.log"
grep -Eq 'state observer summary frames=[1-9][0-9]*' \
  "${output_dir}/logs/state_observer.log"
grep -Eq 'camera summary screenshot=1 .*front_rgb=[1-9][0-9]* .*front_depth=[1-9][0-9]* .*front_segmentation=[1-9][0-9]* .*front_points=[1-9][0-9]*' \
  "${output_dir}/logs/camera_consumer.log"
grep -Eq 'mapper summary processed=[1-9][0-9]* accepted=[1-9][0-9]*' \
  "${output_dir}/logs/mapper.log"
grep -Eq 'truth recorder summary samples=[1-9][0-9]*' \
  "${output_dir}/logs/truth_recorder.log"

"${quality_score}" "${output_dir}"
