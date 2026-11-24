#!/bin/bash
# "FAST-LIO 建图录屏" desktop icon: lidar driver + FAST-LIO + RViz2, recording the screen and a rosbag.
# Close the RViz window to stop: the recording and the bag are finalised, then a map is exported.
# Output: ~/fastlio_runs/<date_time>/
#   screen.mp4      screen recording (fragmented MP4: still playable if the power dies mid-walk)
#   bag/            rosbag2 of /livox/lidar /livox/imu /Odometry /cloud_registered (re-run FAST-LIO offline)
#   map.pcd         map from /cloud_registered, one point per WALK_MAP_VOXEL (default 0.1 m)
#   trajectory.csv  t, x, y, z, qx, qy, qz, qw from /Odometry
#   logs/           one log per process
# Env: WALK_FPS (15), WALK_MAP_VOXEL (0.1), WALK_LIO_VOXEL (0.15), WALK_LIO_FILTER (2), WALK_NO_UNLOCK=1, WALK_NO_BAG=1 (no bag,
#      no map), WALK_NO_DIALOG=1
set -u
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
OUT="$HOME/fastlio_runs/$(date +%Y%m%d_%H%M%S)"
LOG="$OUT/logs"
FPS="${WALK_FPS:-15}"
VOXEL="${WALK_MAP_VOXEL:-0.1}"
# FAST-LIO map resolution (same as config/fastlio_mid360_agras.yaml): replaying the 2026-11-05 stairwell walk
# 3 times each, 0.5 m lost track every time, 0.2 m once, 0.15 m never; point_filter_num 2 = every 2nd point
LIO_VOXEL="${WALK_LIO_VOXEL:-0.15}"
LIO_FILTER="${WALK_LIO_FILTER:-2}"
case "$LIO_VOXEL" in *.*) ;; *) LIO_VOXEL="$LIO_VOXEL.0" ;; esac  # ROS would read "1" as int, the node wants a double
TITLE="FAST-LIO 建图录屏"
export DISPLAY="${DISPLAY:-:0}"
[ -n "${XAUTHORITY:-}" ] || export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ] || export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
exec 9> "/tmp/fastlio_walk.$(id -u).lock"
if ! flock -n 9; then
	[ "${WALK_NO_DIALOG:-0}" = 1 ] ||
		zenity --warning --title="$TITLE" --width=400 --text="已经在运行。关掉 RViz 窗口就会结束上一次。" 2>/dev/null
	exit 0
fi
mkdir -p "$LOG"

log() { echo "$(date +%T) $*" >> "$LOG/walk.log"; }
say() { log "$1"; notify-send -t "${2:-6000}" "$TITLE" "$1" 2>/dev/null || true; }
die() { # stop everything first: the dialog waits for a click, and the driver must not keep UDP 60001 meanwhile
	log "ERROR: $1"
	stop_all
	[ "${WALK_NO_DIALOG:-0}" = 1 ] || zenity --error --title="$TITLE" --width=460 --text="$1" 2>/dev/null
	exit 1
}

declare -A PID
start() { # start NAME CMD...: own session, so the whole process tree can be signalled; log in logs/NAME.log
	local name="$1"
	shift
	# bash starts background jobs of a script with SIGINT ignored; reset it, or ffmpeg / ros2 bag
	# would not see the SIGINT that makes them finalise their files
	setsid env --default-signal=INT,QUIT "$@" > "$LOG/$name.log" 2>&1 < /dev/null 9>&- &
	PID[$name]=$!
}
stop() { # stop NAME SECONDS: SIGINT the process group, SIGKILL if it is still there after SECONDS
	local p="${PID[$1]:-}" i
	[ -n "$p" ] || return 0
	kill -INT -- "-$p" 2>/dev/null
	for ((i = 0; i < $2 * 10; i++)); do
		kill -0 "$p" 2>/dev/null || break
		sleep 0.1
	done
	if kill -0 "$p" 2>/dev/null; then
		log "$1 did not stop within $2 s, killing it"
		kill -KILL -- "-$p" 2>/dev/null
	fi
	unset "PID[$1]"
}
alive() { [ -n "${PID[$1]:-}" ] && kill -0 "${PID[$1]}" 2>/dev/null; }
WEB_STOPPED=0
WEB_CMD=""
web_note=""
restore_web() { # give UDP 60001 back to the web page's lidar_web.py (idempotent)
	[ "$WEB_STOPPED" = 1 ] || return 0
	WEB_STOPPED=0
	if sudo -n systemctl start lidar-web 2> /dev/null; then
		web_note="网页点云服务已恢复。"
	else
		# no passwordless sudo: run it as a plain user process until the next reboot (the service is enabled)
		# shellcheck disable=SC2086
		setsid $WEB_CMD > /tmp/lidar_web_after_walk.log 2>&1 < /dev/null 9>&- &
		web_note="网页点云已恢复（临时进程，重启后由 lidar-web 服务接管）。"
	fi
	log "$web_note"
}
stop_all() { stop screen 10; stop bag 20; stop rviz 5; stop fastlio 10; stop driver 10; stop inhibit 2; restore_web; }
trap stop_all EXIT
trap 'exit 130' INT TERM HUP

# ---- environment and lidar
[ -f "$ROOT/ros2_ws/install/setup.bash" ] || die "ROS2 工作区还没编译：$ROOT/scripts/build_all.sh"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
read -r W H < <(xdpyinfo 2>/dev/null | awk '/dimensions:/ {split($2, a, "x"); print a[1], a[2]}')
[ -n "${W:-}" ] || die "打不开显示器 $DISPLAY。这个脚本要在 Orin 的桌面上运行。"
ip -4 addr | grep -q 'inet 192\.168\.1\.20/24' ||
	die "没有网口是 192.168.1.20/24。\n先运行：sudo $ROOT/scripts/setup_network.sh"
say "等待激光雷达 192.168.1.10…" 4000
ok=0
for _ in $(seq 1 30); do
	ping -c 1 -W 1 192.168.1.10 > /dev/null 2>&1 && { ok=1; break; }
	sleep 1
done
[ "$ok" = 1 ] || die "30 秒内 ping 不通激光雷达 192.168.1.10。\n检查供电（约 0.35 A）和网线；刚插上的链路可能要 1 分钟。"

# UDP 60001 has a single owner: the web page's lidar_web.py runs as this user, so it can be stopped
# without sudo (it exits cleanly on SIGTERM, so systemd does not restart it)
WEB_CMD="$(pgrep -a -u "$(id -u)" -f 'dji-radar/setup/lidar_web.py' | head -n 1 | cut -d' ' -f2-)"
if [ -n "$WEB_CMD" ]; then
	log "stopping lidar_web.py (it holds UDP 60001): $WEB_CMD"
	pkill -TERM -u "$(id -u)" -f 'dji-radar/setup/lidar_web.py'
	WEB_STOPPED=1
fi
for _ in $(seq 1 30); do
	ss -Hlun 'sport = :60001' | grep -q . || break
	sleep 0.2
done
ss -Hlun 'sport = :60001' | grep -q . &&
	die "UDP 60001 被别的程序占用（lidar-web 服务或另一个驱动）。\n先运行：sudo systemctl stop lidar-web"

# keep the screen on while walking
command -v gnome-session-inhibit > /dev/null &&
	start inhibit gnome-session-inhibit --inhibit idle:suspend --reason "$TITLE" sleep infinity

# ---- unlock performance: full-circle FOV + motor sampling state, streams pointed back here.
# Same as scripts/run.sh; resets when the lidar reboots. WALK_NO_UNLOCK=1 skips it.
if [ "${WALK_NO_UNLOCK:-0}" != 1 ]; then
	if python3 "$ROOT/tools/mid360_fov.py" --unlock >> "$LOG/walk.log" 2>&1; then
		log "unlock: full circle + motor sampling on"
	else
		log "unlock: failed or lidar not reachable (see above); mapping continues with the boot FOV"
	fi
fi

# ---- driver and FAST-LIO
say "启动激光雷达驱动…" 3000
start driver ros2 run mid360_agras_ros mid360_agras_node
# the message type is given so echo waits for the topic instead of failing while it does not exist
# yet; best_effort matches both reliable and best-effort publishers
if ! timeout 30 ros2 topic echo --once --qos-reliability best_effort /livox/imu sensor_msgs/msg/Imu > /dev/null 2>&1; then
	tail -n 20 "$LOG/driver.log" >> "$LOG/walk.log"
	die "驱动 30 秒内没有发出 /livox/imu。\n日志：$LOG/driver.log"
fi
log "driver is publishing /livox/imu"
SHARE="$(ros2 pkg prefix fast_lio)/share/fast_lio"
CFG="$(ros2 pkg prefix mid360_agras_ros)/share/mid360_agras_ros/config/fastlio_mid360_agras.yaml"
# the config has map_en off (FAST-LIO would re-send its whole growing map every second) and dense on
# (/cloud_registered carries the full undistorted scan, for RViz and map.pcd); the voxel can be overridden
log "FAST-LIO: $CFG, map voxel $LIO_VOXEL m, point_filter_num $LIO_FILTER"
start fastlio ros2 run fast_lio fastlio_mapping --ros-args --params-file "$CFG" \
	-p filter_size_surf:="$LIO_VOXEL" -p filter_size_map:="$LIO_VOXEL" -p point_filter_num:="$LIO_FILTER"

# ---- RViz, sized to the screen, without the side panel, the view following the lidar
sed -e 's/Decay Time: 30/Decay Time: 20/' -e 's/Hide Left Dock: false/Hide Left Dock: true/' \
	-e 's/Target Frame: <Fixed Frame>/Target Frame: body/' \
	-e "s/^  Width: .*/  Width: $W/" -e "s/^  Height: .*/  Height: $H/" -e 's/^  X: .*/  X: 0/' -e 's/^  Y: .*/  Y: 0/' \
	"$SHARE/rviz/fastlio.rviz" > "$OUT/walk.rviz"
start rviz rviz2 -d "$OUT/walk.rviz"
WID=""
for _ in $(seq 1 60); do
	WID="$(xdotool search --onlyvisible --name 'RViz' 2> /dev/null | head -n 1)"
	[ -n "$WID" ] && break
	alive rviz || die "RViz 启动失败。\n日志：$LOG/rviz.log"
	sleep 0.5
done
[ -n "$WID" ] && xdotool windowmove "$WID" 0 0 windowsize "$WID" 100% 100% 2> /dev/null

# ---- recording
start screen ffmpeg -nostdin -loglevel warning -f x11grab -draw_mouse 1 -framerate "$FPS" -video_size "${W}x${H}" \
	-i "$DISPLAY" -vf "crop=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p \
	-g "$((FPS * 2))" -movflags +frag_keyframe+empty_moov+default_base_moof "$OUT/screen.mp4"
[ "${WALK_NO_BAG:-0}" = 1 ] || start bag ros2 bag record -o "$OUT/bag" /livox/lidar /livox/imu /Odometry /cloud_registered
say $'正在录制。先让雷达静止 3 秒（IMU 初始化），再开始走。\n结束：关掉 RViz 窗口，会自动保存。' 12000

warned=" "
while alive rviz; do
	for p in driver fastlio screen bag; do
		if [ -n "${PID[$p]:-}" ] && ! alive "$p" && [[ "$warned" != *" $p "* ]]; then
			warned+="$p "
			say "$p 意外退出了，日志：$LOG/$p.log" 10000
		fi
	done
	sleep 1
done
log "RViz closed"

# ---- finish
say "RViz 已关闭，正在保存…" 4000
stop screen 10
stop bag 20
stop fastlio 10
stop driver 10
stop inhibit 2
if [ -d "$OUT/bag" ]; then
	say "正在从数据包导出地图（走得越久越慢）…" 8000
	python3 "$ROOT/scripts/bag_to_pcd.py" "$OUT/bag" "$OUT/map.pcd" --voxel "$VOXEL" --traj "$OUT/trajectory.csv" \
		> "$LOG/export.log" 2>&1 || log "map export failed, see logs/export.log"
fi
restore_web
size() { if [ -e "$1" ]; then du -sh "$1" | cut -f1; else echo "无"; fi; }
summary="$(grep -E '^(scans|trajectory)' "$LOG/export.log" 2> /dev/null)"
log "done: screen $(size "$OUT/screen.mp4"), bag $(size "$OUT/bag"), map $(size "$OUT/map.pcd")"
[ "${WALK_NO_DIALOG:-0}" = 1 ] || zenity --info --title="$TITLE" --width=520 --timeout=180 --text="已保存到 $OUT\n\n录屏 screen.mp4：$(size "$OUT/screen.mp4")\n数据包 bag/：$(size "$OUT/bag")\n地图 map.pcd：$(size "$OUT/map.pcd")\n\n$summary\n\n$web_note" 2> /dev/null
exit 0
