#!/usr/bin/env bash
# Build a ROS 2 workspace, then start three tmux panes that run launch files.
# Usage:
#   ./run_stack.sh [workspace_path]
#   ROS_DISTRO=humble ./run_stack.sh

set -Eeuo pipefail

# -------- Config --------
SESSION_NAME="ros2_stack"
WS="${1:-$(pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# -------- Checks --------
command -v tmux >/dev/null || { echo "tmux is required: sudo apt install tmux"; exit 1; }
command -v colcon >/dev/null || { echo "colcon is required: sudo apt install python3-colcon-common-extensions"; exit 1; }

if [[ ! -d "$WS/src" ]]; then
  echo "Workspace doesn't look valid (no src/): $WS"
  exit 1
fi

# -------- Build --------
echo "==> Building workspace at: $WS"
pushd "$WS" >/dev/null

set +u
[[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] && source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

colcon build
echo "==> Build complete."

WS_SETUP="$WS/install/setup.bash"
if [[ ! -f "$WS_SETUP" ]]; then
  echo "Could not find $WS_SETUP after build."
  popd >/dev/null
  exit 1
fi
popd >/dev/null

# -------- tmux session --------
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  tmux kill-session -t "$SESSION_NAME"
fi

# Helper: build a full launch command
launch_cmd () {
  local label="$1"
  local cmd="$2"
  echo "set +u; \
        source /opt/ros/$ROS_DISTRO/setup.bash >/dev/null 2>&1 || true; \
        source \"$WS_SETUP\"; \
        set -u; \
        echo \"[$label] Running: $cmd\"; \
        $cmd; \
        echo \"[$label] exited with status \$?\"; \
        exec bash"
}

# Start session with first pane (camera)
tmux new-session -d -s "$SESSION_NAME" -n "roslaunches" \
  "bash -lc '$(launch_cmd camera "ros2 launch orbbec_camera femto_mega.launch.py")'"

# Split to second pane (lidar)
tmux split-window -v -t "$SESSION_NAME:0" \
  "bash -lc '$(launch_cmd lidar "ros2 launch orbbec_lidar_ros2 single_line.launch.py")'"

# Split to third pane (visual odometry)
tmux split-window -v -t "$SESSION_NAME:0" \
  "bash -lc '$(launch_cmd visual_odom "ros2 launch mark1_launch visual_odometry.launch.xml")'"

# Split to fourth pane (slam + ekf + robot state)
tmux split-window -v -t "$SESSION_NAME:0" \
  "bash -lc '$(launch_cmd slam "ros2 launch mark1_launch imu_lidar_slam.launch.xml")'"

# Split to fifth pane (empty, but sourced)
# tmux split-window -v -t "$SESSION_NAME:0" \
#     "bash -lc 'set +u; source /opt/ros/$ROS_DISTRO/setup.bash >/dev/null 2>&1 || true; source \"$WS_SETUP\"; set -u; exec bash'"

# Arrange panes evenly
tmux select-layout -t "$SESSION_NAME:0" tiled

# Attacho
tmux attach -t "$SESSION_NAME"
