#!/bin/bash

set -euo pipefail

# ---- config ----
: "${ROS_DISTRO:=humble}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="${REPO_ROOT}/_ws"

echo "[test.bash] repo: ${REPO_ROOT}"
echo "[test.bash] ws:   ${WS_ROOT}"
echo "[test.bash] distro: ${ROS_DISTRO}"

# WSL で FastDDS SHM が荒れることがあるので無効化（不要なら消してOK）
export RMW_FASTRTPS_USE_SHM=0

# ---- setup ROS ----
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # setup.bash は未定義変数を参照することがあるため、nounset(-u)を一時的に外す
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
else
  echo "[test.bash] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found" >&2
  exit 1
fi


# ---- make an isolated workspace ----
rm -rf "${WS_ROOT}"
mkdir -p "${WS_ROOT}/src"

# パッケージを workspace/src に配置（git管理そのまま使う）
rsync -a --delete \
  --exclude '_ws/' \
  --exclude '.git/' \
  "${REPO_ROOT}/" "${WS_ROOT}/src/pc_util_monitor/"


cd "${WS_ROOT}"

# ---- rosdep (best effort) ----
if command -v rosdep >/dev/null 2>&1; then
  rosdep update >/dev/null 2>&1 || true
  rosdep install -y --from-paths src --ignore-src -r || true
fi

# ---- build & test ----
colcon build --symlink-install --packages-select pc_util_monitor
# install/setup.bash も未定義変数を参照することがあるので一時的に -u を外す
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u


colcon test --packages-select pc_util_monitor --event-handlers console_direct+
colcon test-result --verbose
