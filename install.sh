#!/bin/bash
# Builds and installs the goodixmilan libfprint TOD driver on Ubuntu/Debian.
#
#   sudo ./install.sh                 install the driver
#   sudo ./install.sh --pam-retries   also allow 3 fingerprint attempts in PAM (default: 1)
#
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Please run with sudo"; exit 1; }
cd "$(dirname "$0")"

PAM_RETRIES=0
[ "${1:-}" = "--pam-retries" ] && PAM_RETRIES=1

D=/sys/bus/spi/devices/spi-GXFP3200:00
if [ ! -e "$D" ]; then
  echo "No GXFP3200 SPI fingerprint sensor found on this machine (expected $D)."
  exit 1
fi

echo "== 1/5 build dependencies"
apt-get install -y -qq build-essential pkg-config libfprint-2-tod-dev fprintd libpam-fprintd >/dev/null

echo "== 2/5 build"
make -s

echo "== 3/5 install module and udev rule"
make -s install
udevadm control --reload
# bind spidev now, as the udev rule will do at every boot
if [ -e "$D/driver" ] && [ "$(basename "$(readlink "$D/driver")")" != spidev ]; then
  echo "The sensor is bound to another driver ($(basename "$(readlink "$D/driver")")), aborting."
  exit 1
fi
[ -e "$D/driver" ] || udevadm trigger --action=add "$D"
for _ in $(seq 30); do ls "$D/spidev" >/dev/null 2>&1 && break; sleep 0.1; done
ls "$D/spidev" >/dev/null 2>&1 || { echo "spidev was not bound to the sensor"; exit 1; }

echo "== 4/5 restart fprintd"
systemctl restart fprintd.service
sleep 1
if ! fprintd-list root 2>&1 | grep -q "Goodix Milan F"; then
  fprintd-list root 2>&1 | head -5
  echo "fprintd does not see the sensor; see 'journalctl -u fprintd' and the README troubleshooting section."
  exit 1
fi

echo "== 5/5 PAM"
if [ "$PAM_RETRIES" = 1 ]; then
  install -m 644 data/pam-goodix-fp /usr/share/pam-configs/goodix-fp
  pam-auth-update --disable fprintd
  pam-auth-update --enable goodix-fp
else
  pam-auth-update --enable fprintd
fi

echo
echo "Done. Enroll a finger with:  fprintd-enroll -f right-index-finger"
echo "or from GNOME Settings > Users > Fingerprint Login."
