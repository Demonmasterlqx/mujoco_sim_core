#!/usr/bin/env bash
set -euo pipefail

source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_prefix="${1:?usage: build_matrix.sh WORKSPACE_PREFIX [WORK_DIR]}"
work_root="${2:-${TMPDIR:-/tmp}/mujoco_sim_server_matrix}"
model="${source_dir}/../mujoco_sim_core/test/testdata/simple_actuator.xml"
core_library_path="${workspace_prefix}/mujoco_sim_core/lib"

rm -rf "${work_root}"
mkdir -p "${work_root}"

build_case() {
  local name="$1"
  local gui="$2"
  local sensors="$3"
  local build_dir="${work_root}/${name}/build"
  local install_dir="${work_root}/${name}/install"

  cmake -S "${source_dir}" -B "${build_dir}" \
    -DBUILD_TESTING=OFF \
    -DMUJOCO_SIM_BUILD_GUI="${gui}" \
    -DMUJOCO_SIM_BUILD_SENSOR_STREAMS="${sensors}" \
    -DCMAKE_PREFIX_PATH="${workspace_prefix}" \
    -DCMAKE_INSTALL_PREFIX="${install_dir}"
  cmake --build "${build_dir}" --parallel 2
  cmake --install "${build_dir}"

  LD_LIBRARY_PATH="${core_library_path}:${LD_LIBRARY_PATH:-}" \
    "${install_dir}/bin/mujoco-sim-server" --help >/dev/null
  if [[ "${gui}" == "OFF" ]]; then
    LD_LIBRARY_PATH="${core_library_path}:${LD_LIBRARY_PATH:-}" \
      timeout 10s "${install_dir}/bin/mujoco-sim-server" \
      --headless --model "${model}" --max-ticks 1 --log-level off
  fi

  if [[ "${gui}" == "OFF" ]] &&
      ldd "${install_dir}/bin/mujoco-sim-server" |
        grep -Eiq 'glfw|libpng'; then
    echo "${name}: GUI-disabled binary links GUI dependencies" >&2
    return 1
  fi
  if [[ "${sensors}" == "OFF" ]] &&
      ldd "${install_dir}/bin/mujoco-sim-server" |
        grep -Eiq 'libEGL|libOSMesa|mujoco_sim_sensor'; then
    echo "${name}: sensor-disabled binary links sensor dependencies" >&2
    return 1
  fi

  cmake -S "${source_dir}/test/package_consumer" \
    -B "${work_root}/${name}/consumer" \
    -DCMAKE_PREFIX_PATH="${install_dir};${workspace_prefix}"
  cmake --build "${work_root}/${name}/consumer" --parallel 2
}

build_case gui_on_sensor_on ON ON
build_case gui_on_sensor_off ON OFF
build_case gui_off_sensor_on OFF ON
build_case gui_off_sensor_off OFF OFF
