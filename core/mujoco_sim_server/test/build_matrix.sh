#!/usr/bin/env bash
set -euo pipefail
shopt -s nullglob

source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
prefix_input="${1:?usage: build_matrix.sh PREFIX_PATH [WORK_DIR]}"
work_root="${2:-${TMPDIR:-/tmp}/mujoco_sim_server_matrix}"
model="${source_dir}/../mujoco_sim_core/test/testdata/simple_actuator.xml"

declare -a package_prefixes=()

append_prefix() {
  local candidate="$1"
  local existing
  for existing in "${package_prefixes[@]:-}"; do
    if [[ "${existing}" == "${candidate}" ]]; then
      return
    fi
  done
  package_prefixes+=("${candidate}")
}

IFS=':;' read -r -a requested_prefixes <<< "${prefix_input}"
for requested_prefix in "${requested_prefixes[@]}"; do
  [[ -n "${requested_prefix}" ]] || continue
  if [[ -d "${requested_prefix}/lib/cmake" || -d "${requested_prefix}/share" ]]; then
    append_prefix "${requested_prefix}"
  fi
  for package_prefix in "${requested_prefix}"/*; do
    if [[ -d "${package_prefix}/lib/cmake" || -d "${package_prefix}/share" ]]; then
      append_prefix "${package_prefix}"
    fi
  done
done

if ((${#package_prefixes[@]} == 0)); then
  echo "no installed package prefixes found in: ${prefix_input}" >&2
  exit 1
fi

cmake_prefix_path=""
core_library_path=""
for package_prefix in "${package_prefixes[@]}"; do
  cmake_prefix_path="${cmake_prefix_path:+${cmake_prefix_path};}${package_prefix}"
  for mujoco_library in "${package_prefix}"/lib/libmujoco.so*; do
    if [[ -e "${mujoco_library}" ]]; then
      core_library_path="${package_prefix}/lib"
      break
    fi
  done
done

if [[ -z "${core_library_path}" ]]; then
  echo "MuJoCo runtime library not found in installed prefixes" >&2
  exit 1
fi

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
    -DCMAKE_PREFIX_PATH="${cmake_prefix_path}" \
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
    -DCMAKE_PREFIX_PATH="${install_dir};${cmake_prefix_path}"
  cmake --build "${work_root}/${name}/consumer" --parallel 2
}

build_case gui_on_sensor_on ON ON
build_case gui_on_sensor_off ON OFF
build_case gui_off_sensor_on OFF ON
build_case gui_off_sensor_off OFF OFF
