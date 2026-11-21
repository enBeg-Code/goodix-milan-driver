#!/bin/bash
# Removes the goodixmilan driver. Enrolled prints stay in /var/lib/fprint until deleted
# (fprintd-delete <user>). fprintd and libpam-fprintd are left installed.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Please run with sudo"; exit 1; }
cd "$(dirname "$0")"

if [ -e /usr/share/pam-configs/goodix-fp ]; then
  pam-auth-update --disable goodix-fp || true
  rm -f /usr/share/pam-configs/goodix-fp
fi
pam-auth-update --disable fprintd || true
make -s uninstall
udevadm control --reload
D=/sys/bus/spi/devices/spi-GXFP3200:00
if [ -e "$D/driver" ]; then echo spi-GXFP3200:00 > "$D/driver/unbind"; fi
if [ -e "$D/driver_override" ]; then echo > "$D/driver_override"; fi
systemctl restart fprintd.service || true
echo "goodixmilan driver removed."
