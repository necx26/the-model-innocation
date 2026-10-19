#!/usr/bin/env bash
# Test an already mounted Ubuntu installation without booting it. Host tool only.
set -euo pipefail
usb_root=${1:?Usage: usb-chroot-test.sh MOUNTPOINT [--as-root] command arguments...}
shift
test_uid=1000; test_gid=1000
if [[ ${1:-} == --as-root ]]; then test_uid=0; test_gid=0; shift; fi
sudo -n unshare --mount --propagation private /bin/bash -s -- "$usb_root" "$test_uid" "$test_gid" "$@" <<'SH'
set -euo pipefail
target=$1; uid=$2; gid=$3; shift 3
mount --rbind /dev "$target/dev"
mount --make-rslave "$target/dev"
mount -t proc proc "$target/proc"
mount --rbind /sys "$target/sys"
mount --make-rslave "$target/sys"
mkdir -p "$target/tmp/.X11-unix" "$target/run/systemd/resolve"
mount --bind /tmp/.X11-unix "$target/tmp/.X11-unix"
if [[ -d /run/systemd/resolve ]]; then
  mount --bind /run/systemd/resolve "$target/run/systemd/resolve"
fi
exec chroot --userspec="$uid:$gid" "$target" /usr/bin/env -i \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  HOME=/home/ros USER=ros LOGNAME=ros LANG=C.UTF-8 LC_ALL=C.UTF-8 TERM=xterm \
  DISPLAY=:1 XAUTHORITY=/home/ros/yzbot_competition/.test-Xauthority \
  LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=4 \
  "$@"
SH
