#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${PACKAGE_ROOT}/../.." && pwd)"

ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
WORKSPACE_INSTALL_PREFIX="${WORKSPACE_ROOT}/install"

filter_missing_workspace_prefixes() {
  local var_name="$1"
  local value="${!var_name-}"
  local filtered=()
  local entry=""

  [[ -n "${value}" ]] || return 0

  IFS=':' read -r -a entries <<< "${value}"
  for entry in "${entries[@]}"; do
    [[ -n "${entry}" ]] || continue

    if [[ "${entry}" == "${WORKSPACE_INSTALL_PREFIX}"* && ! -e "${entry}" ]]; then
      continue
    fi

    filtered+=("${entry}")
  done

  if ((${#filtered[@]} == 0)); then
    unset "${var_name}"
  else
    printf -v "${var_name}" '%s' "$(IFS=:; echo "${filtered[*]}")"
    export "${var_name}"
  fi
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

echo "Using workspace: ${WORKSPACE_ROOT}"

filter_missing_workspace_prefixes AMENT_PREFIX_PATH
filter_missing_workspace_prefixes CMAKE_PREFIX_PATH
filter_missing_workspace_prefixes COLCON_PREFIX_PATH

set +u
source "${ROS_SETUP}"
if [[ -f "${WORKSPACE_SETUP}" ]]; then
  source "${WORKSPACE_SETUP}"
else
  echo "Workspace setup not found, skipping: ${WORKSPACE_SETUP}"
fi
set -u

cd "${WORKSPACE_ROOT}"

exec colcon build --packages-select px4adrc --symlink-install "$@"
