#!/bin/bash
# Build everything on Ubuntu 22.04 + ROS2 Humble (Jetson Orin / x86):
#   1. standalone driver (linux/)  -> linux/build/mid360_agras_demo
#   2. ROS2 workspace (ros2_ws/src) -> ros2_ws/install
#        livox_ros_driver2  Livox's CustomMsg / CustomPoint definitions only (MIT)
#        FAST_LIO           hku-mars/FAST_LIO ROS2 branch, unmodified (GPL-2.0)
#        mid360_agras_ros   our node (built on linux/), launch files, FAST-LIO config
# Usage: ./build_all.sh [--deps] [--no-ros]
#   --deps    apt install the build dependencies first (needs sudo)
#   --no-ros  only step 1
set -eu
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="$(nproc)"
DEPS=0; ROS=1
for a in "$@"; do case "$a" in --deps) DEPS=1 ;; --no-ros) ROS=0 ;; esac; done

if [ "$DEPS" = 1 ]; then
	sudo apt-get update
	sudo apt-get install -y build-essential cmake git libeigen3-dev libpcl-dev python3-dev
	if [ "$ROS" = 1 ]; then
		sudo apt-get install -y python3-colcon-common-extensions ros-humble-pcl-ros ros-humble-pcl-conversions \
			ros-humble-tf2-ros ros-humble-rviz2
	fi
fi

echo "=== 1/2 standalone driver ==="
cmake -S "$ROOT/linux" -B "$ROOT/linux/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/linux/build" -j"$JOBS"

[ "$ROS" = 1 ] || { echo "done (ROS2 skipped)"; exit 0; }
echo "=== 2/2 ROS2 workspace ==="
[ -f /opt/ros/humble/setup.bash ] || { echo "ROS2 Humble not found (/opt/ros/humble). Install it or use --no-ros." >&2; exit 1; }
# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u
cd "$ROOT/ros2_ws"
# on an 8 GB Orin Nano FAST-LIO can run out of memory with many parallel jobs
MAKEFLAGS="-j$(( JOBS > 4 ? 4 : JOBS ))" colcon build --symlink-install --parallel-workers 1 \
	--cmake-args -DCMAKE_BUILD_TYPE=Release
echo "done. Run: $ROOT/scripts/run.sh view   (or mapping)"
