#!/usr/bin/env bash
set -Eeuo pipefail

# 2026-07-09 修改原因：把 ROS2 包交叉编译入口固定在包内，保证 build/install/log 产物不污染调用者当前目录。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PKG_NAME="robobaton_4p_ros2_demo"

X5_CROSS_ROOT="${X5_CROSS_ROOT:-/root/x5/cross_compile/new}"
CROSS_ENV_SCRIPT="${CROSS_ENV_SCRIPT:-}"
TOOLCHAIN_FILE="${TOOLCHAIN_FILE:-}"
X5_USR_X5_INCLUDE_DIR="${X5_USR_X5_INCLUDE_DIR:-}"
X5_HOBOT_LIB_DIR="${X5_HOBOT_LIB_DIR:-}"

BUILD_ROOT="${BUILD_ROOT:-${PKG_DIR}/1.ros2_build}"
BUILD_BASE="${BUILD_BASE:-}"
INSTALL_BASE="${INSTALL_BASE:-}"
LOG_BASE="${LOG_BASE:-}"
ROBOBATON_LIB_DIR="${ROBOBATON_LIB_DIR:-${PKG_DIR}/lib}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-}"
CLEAN=0
EXTRA_COLCON_ARGS=()

usage() {
  cat <<'USAGE'
Usage:
  script/build_x5_ros2.sh [options] [-- extra colcon args...]

Options:
  --clean                         Remove package-local build/install/log before building.
  --cross-root <path>             X5 cross bundle root, default /root/x5/cross_compile/new.
  --cross-env <path>              Cross environment setup script, default <cross-root>/scripts/setup_x5_cross_env.sh.
  --toolchain-file <path>         CMake toolchain file, default <cross-root>/toolchain/aarch64_x5_host_toolchain.cmake.
  --robobaton-lib-dir <path>      Shared library directory used by this ROS2 package, default ./lib.
  --primary-lib-dir <path>        Backward-compatible alias for --robobaton-lib-dir.
  --build-root <path>             Parent directory for build/install/log, default ./1.ros2_build.
  --build-base <path>             Colcon build base, default <build-root>/build.
  --install-base <path>           Colcon install base, default <build-root>/install.
  --log-base <path>               Colcon log base, default <build-root>/log.
  --cmake-build-type <type>       CMake build type, default Release.
  --parallel-workers <n>          Forward to colcon --parallel-workers.
  -h, --help                      Show this help.

Environment overrides:
  X5_USR_X5_INCLUDE_DIR, X5_HOBOT_LIB_DIR, ROBOBATON_LIB_DIR,
  BUILD_ROOT, BUILD_BASE, INSTALL_BASE, LOG_BASE, CMAKE_BUILD_TYPE,
  PARALLEL_WORKERS.

  A normal merged colcon install tree is generated at ./1.ros2_build/install by default.
  Shared libraries must already be generated into ./lib by the upstream library build rules.
USAGE
}

resolve_project_path() {
  local path="$1"
  if [[ "${path}" = /* ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s\n' "${PKG_DIR}/${path}"
  fi
}

resolve_existing_dir() {
  local path="$1"
  if [[ ! -d "${path}" ]]; then
    echo "Missing directory: ${path}" >&2
    exit 2
  fi
  cd "${path}" && pwd
}

resolve_existing_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "Missing file: ${path}" >&2
    exit 2
  fi
  cd "$(dirname "${path}")" && printf '%s/%s\n' "$(pwd)" "$(basename "${path}")"
}

resolve_output_dir() {
  local path="$1"
  mkdir -p "${path}"
  cd "${path}" && pwd
}

refuse_unsafe_clean_dir() {
  local path="$1"
  if [[ "${path}" == "/" || "${path}" == "${PKG_DIR}" || "${path}" == "${SCRIPT_DIR}" ]]; then
    echo "Refusing unsafe clean path: ${path}" >&2
    exit 2
  fi
}

reset_stale_package_build_dir() {
  local package_build_dir="${BUILD_BASE}/${PKG_NAME}"
  local legacy_package_build_dir="${PKG_DIR}/build/${PKG_NAME}"

  if [[ ! -f "${package_build_dir}/CMakeCache.txt" ]]; then
    return 0
  fi

  # 2026-07-13 修改原因：从旧 build/ 迁移到 1.ros2_build/ 时，CMakeCache 以外的生成文件也会硬编码旧 build 路径。
  # 2026-07-13 修改原因：检测到旧路径残留时整包重建 build 子树，避免只删 cache 后继续复用旧 Makefile/cmake_install。
  if grep -R -q --fixed-strings "${legacy_package_build_dir}" "${package_build_dir}"; then
    refuse_unsafe_clean_dir "${package_build_dir}"
    echo "[build_x5_ros2] removing stale package build dir: ${package_build_dir}"
    rm -rf "${package_build_dir}"
  fi
}


while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean) CLEAN=1; shift ;;
    --cross-root) X5_CROSS_ROOT="$2"; shift 2 ;;
    --cross-env) CROSS_ENV_SCRIPT="$2"; shift 2 ;;
    --toolchain-file) TOOLCHAIN_FILE="$2"; shift 2 ;;
    --x5-usr-x5-include-dir) X5_USR_X5_INCLUDE_DIR="$2"; shift 2 ;;
    --x5-hobot-lib-dir) X5_HOBOT_LIB_DIR="$2"; shift 2 ;;
    --robobaton-lib-dir) ROBOBATON_LIB_DIR="$2"; shift 2 ;;
    --primary-lib-dir) ROBOBATON_LIB_DIR="$2"; shift 2 ;;
    --build-root) BUILD_ROOT="$2"; shift 2 ;;
    --install-base) INSTALL_BASE="$2"; shift 2 ;;
    --log-base) LOG_BASE="$2"; shift 2 ;;
    --build-base) BUILD_BASE="$2"; shift 2 ;;
    --cmake-build-type) CMAKE_BUILD_TYPE="$2"; shift 2 ;;
    --parallel-workers) PARALLEL_WORKERS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    --) shift; EXTRA_COLCON_ARGS+=("$@"); break ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

X5_CROSS_ROOT="$(resolve_project_path "${X5_CROSS_ROOT}")"
if [[ -z "${CROSS_ENV_SCRIPT}" ]]; then
  CROSS_ENV_SCRIPT="${X5_CROSS_ROOT}/scripts/setup_x5_cross_env.sh"
else
  CROSS_ENV_SCRIPT="$(resolve_project_path "${CROSS_ENV_SCRIPT}")"
fi
if [[ -z "${TOOLCHAIN_FILE}" ]]; then
  TOOLCHAIN_FILE="${X5_CROSS_ROOT}/toolchain/aarch64_x5_host_toolchain.cmake"
else
  TOOLCHAIN_FILE="$(resolve_project_path "${TOOLCHAIN_FILE}")"
fi
if [[ -z "${X5_USR_X5_INCLUDE_DIR}" ]]; then
  X5_USR_X5_INCLUDE_DIR="${X5_CROSS_ROOT}/sysroot/usr_x5/include"
else
  X5_USR_X5_INCLUDE_DIR="$(resolve_project_path "${X5_USR_X5_INCLUDE_DIR}")"
fi
if [[ -z "${X5_HOBOT_LIB_DIR}" ]]; then
  X5_HOBOT_LIB_DIR="${X5_CROSS_ROOT}/sysroot/usr_x5/hobot/lib"
else
  X5_HOBOT_LIB_DIR="$(resolve_project_path "${X5_HOBOT_LIB_DIR}")"
fi

BUILD_ROOT="$(resolve_project_path "${BUILD_ROOT}")"
ROBOBATON_LIB_DIR="$(resolve_project_path "${ROBOBATON_LIB_DIR}")"
BUILD_BASE="$(resolve_project_path "${BUILD_BASE:-${BUILD_ROOT}/build}")"
INSTALL_BASE="$(resolve_project_path "${INSTALL_BASE:-${BUILD_ROOT}/install}")"
LOG_BASE="$(resolve_project_path "${LOG_BASE:-${BUILD_ROOT}/log}")"

X5_CROSS_ROOT="$(resolve_existing_dir "${X5_CROSS_ROOT}")"
CROSS_ENV_SCRIPT="$(resolve_existing_file "${CROSS_ENV_SCRIPT}")"
TOOLCHAIN_FILE="$(resolve_existing_file "${TOOLCHAIN_FILE}")"
X5_USR_X5_INCLUDE_DIR="$(resolve_existing_dir "${X5_USR_X5_INCLUDE_DIR}")"
X5_HOBOT_LIB_DIR="$(resolve_existing_dir "${X5_HOBOT_LIB_DIR}")"
ROBOBATON_LIB_DIR="$(resolve_existing_dir "${ROBOBATON_LIB_DIR}")"
BUILD_BASE="$(resolve_output_dir "${BUILD_BASE}")"
INSTALL_BASE="$(resolve_output_dir "${INSTALL_BASE}")"
LOG_BASE="$(resolve_output_dir "${LOG_BASE}")"

# 2026-07-13 修改原因：SO 由上游真实产出点同步到本包 lib/；ROS2 消费侧只校验，不再主动搬运。
for required_file in \
  "${ROBOBATON_LIB_DIR}/libicm42688.so" \
  "${ROBOBATON_LIB_DIR}/libsc132.so" \
  "${X5_USR_X5_INCLUDE_DIR}/hb_mem_mgr.h" \
  "${X5_HOBOT_LIB_DIR}/libhbmem.so"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "Missing required artifact: ${required_file}" >&2
    exit 2
  fi
done

if ! command -v colcon >/dev/null 2>&1; then
  echo "Missing command: colcon" >&2
  exit 2
fi

# 2026-07-09 修改原因：必须加载目标侧 ROS/ament/pkg-config 前缀，避免 find_package 误命中宿主机 ROS。
# shellcheck source=/root/x5/cross_compile/new/scripts/setup_x5_cross_env.sh
source "${CROSS_ENV_SCRIPT}"

if [[ "${CLEAN}" -eq 1 ]]; then
  refuse_unsafe_clean_dir "${BUILD_BASE}"
  refuse_unsafe_clean_dir "${INSTALL_BASE}"
  refuse_unsafe_clean_dir "${LOG_BASE}"
  rm -rf "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
fi
mkdir -p "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
reset_stale_package_build_dir

colcon_args=(
  --log-base "${LOG_BASE}"
  build
  --base-paths "${PKG_DIR}"
  --packages-select "${PKG_NAME}"
  --build-base "${BUILD_BASE}"
  --install-base "${INSTALL_BASE}"
  --merge-install
  --cmake-force-configure
)

if [[ -n "${PARALLEL_WORKERS}" ]]; then
  colcon_args+=(--parallel-workers "${PARALLEL_WORKERS}")
fi

if [[ "${#EXTRA_COLCON_ARGS[@]}" -gt 0 ]]; then
  colcon_args+=("${EXTRA_COLCON_ARGS[@]}")
fi

colcon_args+=(
  --cmake-args
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}"
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
  -DX5_USR_X5_INCLUDE_DIR="${X5_USR_X5_INCLUDE_DIR}"
  -DX5_HOBOT_LIB_DIR="${X5_HOBOT_LIB_DIR}"
  -DROBOBATON_LIB_DIR="${ROBOBATON_LIB_DIR}"
  -DBUILD_TESTING=OFF
)

echo "[build_x5_ros2] package=${PKG_DIR}"
echo "[build_x5_ros2] install=${INSTALL_BASE}"
echo "[build_x5_ros2] toolchain=${TOOLCHAIN_FILE}"
echo "[build_x5_ros2] target_ros=${TARGET_ROS_PREFIX:-unset}"
echo "[build_x5_ros2] robobaton_lib=${ROBOBATON_LIB_DIR}"

colcon "${colcon_args[@]}"


node_path="${INSTALL_BASE}/lib/${PKG_NAME}/robobaton_sensors_node"
if [[ ! -x "${node_path}" ]]; then
  echo "Build finished but expected node is missing: ${node_path}" >&2
  exit 3
fi

if command -v file >/dev/null 2>&1; then
  file "${node_path}"
fi

echo "[build_x5_ros2] install tree generated: ${INSTALL_BASE}"
echo "[build_x5_ros2] deploy/source on target: source ${INSTALL_BASE}/setup.bash"
