#!/bin/bash
# Start the Agras Mid-360 stack in one terminal (no gnome-terminal needed, works over SSH).
# Usage: ./run.sh <mode>
#   demo            standalone driver, no ROS: prints pkt/s, points, IMU
#   driver          mid360_agras_node: /livox/lidar (CustomMsg), /livox/imu, /livox/points
#   view            driver + RViz2 with the raw point cloud
#   mapping         driver + FAST-LIO + RViz2
#   mapping_norviz  same without RViz (headless / over SSH)
# Ctrl-C stops everything.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-view}"

# UDP 60001 has a single owner: the web page's lidar_web.py runs as this user and exits cleanly on SIGTERM
if pgrep -u "$(id -u)" -f 'dji-radar/setup/lidar_web.py' > /dev/null; then
	echo "stopping lidar_web.py (it holds UDP 60001); the web page's lidar layer comes back after a reboot"
	pkill -TERM -u "$(id -u)" -f 'dji-radar/setup/lidar_web.py'
	sleep 1
fi
if ! ip -4 addr | grep -q "192.168.1.20/24"; then
	echo "warning: no interface has 192.168.1.20/24; run: sudo $ROOT/scripts/setup_network.sh" >&2
fi

# Unlock performance: full-circle FOV (the firmware boots with a ~65 deg azimuth gap) +
# target state 1 (motor sampling), then point the streams back at this host. Reversible,
# resets on power-cycle / reboot. Skip with UNLOCK=0 (FOV_FULL=0 still works too).
# Harmless if the lidar is unreachable.
if [ "${UNLOCK:-${FOV_FULL:-1}}" = 1 ] && command -v python3 > /dev/null; then
	python3 "$ROOT/tools/mid360_fov.py" --unlock 2>/dev/null \
		&& echo "unlock: full circle + motor sampling on" \
		|| echo "unlock: skipped (lidar not reachable yet; run tools/mid360_fov.py --unlock later)"
fi

if [ "$MODE" = demo ]; then
	exec "$ROOT/linux/build/mid360_agras_demo"
fi

[ -f "$ROOT/ros2_ws/install/setup.bash" ] || { echo "not built yet: $ROOT/scripts/build_all.sh" >&2; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u

case "$MODE" in
	driver)         exec ros2 launch mid360_agras_ros driver.launch.py ;;
	view)           exec ros2 launch mid360_agras_ros driver.launch.py rviz:=true ;;
	mapping)        exec ros2 launch mid360_agras_ros mapping.launch.py ;;
	mapping_norviz) exec ros2 launch mid360_agras_ros mapping.launch.py rviz:=false ;;
	*) echo "unknown mode: $MODE (demo|driver|view|mapping|mapping_norviz)" >&2; exit 1 ;;
esac
