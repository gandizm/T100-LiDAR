#!/bin/bash
# Give the Ethernet port wired to the Agras Mid-360 the fixed address it talks to (192.168.1.20/24).
# Creates a NetworkManager profile "livox-mid360" (never default route, so WiFi keeps the internet),
# or falls back to a temporary `ip addr` when NetworkManager is not running.
# Usage: sudo ./setup_network.sh [interface]      (default: the first wired interface)
set -eu

IFACE="${1:-}"
if [ -z "$IFACE" ]; then
	for n in /sys/class/net/*; do
		i="$(basename "$n")"
		[ -e "$n/device" ] || continue          # skip lo, bridges, veth
		[ -d "$n/wireless" ] && continue        # skip WiFi
		case "$i" in docker*|br-*|veth*|can*|usb*) continue ;; esac
		IFACE="$i"
		break
	done
fi
[ -n "$IFACE" ] || { echo "no wired interface found; pass one: sudo $0 eth0" >&2; exit 1; }
echo "interface: $IFACE"

if command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager; then
	nmcli connection delete livox-mid360 >/dev/null 2>&1 || true
	nmcli connection add type ethernet ifname "$IFACE" con-name livox-mid360 \
		ipv4.method manual ipv4.addresses 192.168.1.20/24 ipv4.never-default yes \
		ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 100 >/dev/null
	nmcli connection up livox-mid360 >/dev/null || echo "profile saved; it comes up once the link is up"
	echo "NetworkManager profile livox-mid360 = 192.168.1.20/24 on $IFACE (persists across reboots)"
else
	ip link set "$IFACE" up
	ip addr replace 192.168.1.20/24 dev "$IFACE"
	echo "temporary address 192.168.1.20/24 on $IFACE (lost on reboot)"
fi

# bigger socket buffers: 300k points/s in ~3100 packets/s
sysctl -qw net.core.rmem_max=16777216 net.core.rmem_default=8388608
printf 'net.core.rmem_max=16777216\nnet.core.rmem_default=8388608\n' > /etc/sysctl.d/90-livox-mid360.conf

for _ in $(seq 1 15); do
	if ping -c 1 -W 1 192.168.1.10 >/dev/null 2>&1; then echo "lidar 192.168.1.10 answers ping"; exit 0; fi
	sleep 2
done
echo "lidar 192.168.1.10 does not answer yet (power on? cable on the right pairs? the link can take ~1 min)"
