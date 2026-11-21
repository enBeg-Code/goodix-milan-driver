# goodix-milan-driver

*[Italiano](README.it.md)*

Linux driver for the **Goodix GXFP3200** SPI fingerprint sensor (Goodix "Milan F" family,
chip ID `0x2202xx`) found in the **ASUSPRO P5440FA / ExpertBook P5440** laptops, which so
far only worked on Windows.

It is a [libfprint](https://fprint.freedesktop.org/) driver, loaded as a TOD module by the
libfprint shipped with Ubuntu, so the standard stack works unchanged: `fprintd`, GNOME
Settings (Fingerprint Login), the lock screen, GDM and `sudo` through `pam_fprintd`.

> **Status: experimental.** Works daily on the author's laptop. Tested on Ubuntu 26.04 with
> libfprint 1.95.1+tod1. Feedback from other machines is very welcome.

## Supported hardware

Check whether your laptop has the sensor:

```
ls /sys/bus/spi/devices/ | grep GXFP3200
```

If `spi-GXFP3200:00` is listed, the driver should work. The driver checks the chip ID at
startup and only accepts `0x2202xx` (108 x 88 pixel sensor). Other Goodix Milan chips are
documented in [docs/PROTOCOL.md](docs/PROTOCOL.md) but untested.

## Install (Ubuntu / Debian)

```
git clone https://github.com/enBeg-Code/goodix-milan-driver.git
cd goodix-milan-driver
sudo ./install.sh                # or: sudo ./install.sh --pam-retries
```

The script installs the build dependencies (`libfprint-2-tod-dev`, `fprintd`,
`libpam-fprintd`), builds and installs the module in the libfprint TOD directory, adds a udev
rule that binds `spidev` to the sensor at boot, restarts `fprintd` and enables fingerprint
authentication in PAM.

Ubuntu allows one fingerprint attempt before falling back to the password. With
`--pam-retries` the script installs a PAM profile that allows 3 attempts (recommended: a
small sensor rejects roughly one touch in ten).

Then enroll a finger:

```
fprintd-enroll -f right-index-finger      # 12 touches, move the finger slightly each time
fprintd-verify
```

or use *Settings → Users → Fingerprint Login*.

Distributions whose libfprint has no TOD support (e.g. Fedora, Arch) need the driver built
into libfprint itself; this is not packaged yet.

## Uninstall

```
sudo ./uninstall.sh
```

Enrolled prints stay in `/var/lib/fprint` until removed with `fprintd-delete <user>`.

## How it works

- **Protocol**: plain register reads/writes over SPI; an image is captured with a short
  command sequence and read from a FIFO in 4 KiB chunks (so the default `spidev` buffer
  size is enough). Full description in [docs/PROTOCOL.md](docs/PROTOCOL.md).
- **Images**: 108 x 88 pixels, 12 bit; the finger is the difference between a frame and an
  empty-sensor background, refreshed whenever the sensor is free.
- **Matching**: the sensor is too small for libfprint's minutiae matcher (NBIS finds 1-2
  minutiae per image), so the driver has its own matcher (`src/gm-match.c`, plain C):
  ORB-like keypoints with RANSAC geometric verification. A template is 12 images; a
  verification takes about 0.1 s.
- **Accuracy** (one person, 4 fingers, 50 images): no false accepts, about 7-12% false
  rejects per touch. More data is needed; see *Contributing*.

## Security notes

Fingerprint unlock is a convenience feature. This driver has no liveness or anti-spoofing
detection, and its accuracy has only been measured on a small dataset. Your password always
remains a valid alternative. Templates are stored by `fprintd` in `/var/lib/fprint`,
readable only by root.

## Troubleshooting

- `journalctl -u fprintd -b` shows the driver messages; for more detail run
  `sudo systemctl stop fprintd` and `sudo G_MESSAGES_DEBUG=all /usr/libexec/fprintd`.
- `ls /sys/bus/spi/devices/spi-GXFP3200:00/spidev` must list `spidev0.0` (or similar).
  If not, reload the rule: `sudo udevadm trigger --action=add /sys/bus/spi/devices/spi-GXFP3200:00`.
- Only one program at a time may talk to the sensor: stop `fprintd` before using the tools.

## Contributing

`tools/` contains Python scripts to test the sensor, collect images and evaluate the matcher
(`pip install numpy opencv-python-headless pillow`):

```
sudo systemctl stop fprintd
sudo python3 tools/gmsensor.py                 # chip ID and one frame
sudo python3 tools/collect.py right-index 20   # guided collection into ./dataset
make libgmmatch.so && python3 tools/eval_matcher.py dataset
```

Collected images are biometric data: keep them private, do not attach them to issues or pull
requests. Reports of accuracy numbers from `eval_matcher.py` are very useful.

Possible next steps: finger detection with the sensor's FDT interrupt mode instead of
polling, OTP calibration, support for other Milan chip IDs, upstreaming the driver into
libfprint.

## Credits

Developed by **enBeg**, who drove the project, provided the hardware and tested every step
on the real laptop.

The reverse engineering, the protocol analysis and most of the code were done with the help
of **[Claude](https://www.anthropic.com/claude)**, an AI assistant made by Anthropic, working
interactively with enBeg. This is stated openly so that users and reviewers know how the
project was made; as with any code, please review it before trusting it.

## Support

If this driver is useful to you, you can support the project with a donation:

[![Donate with PayPal](https://img.shields.io/badge/Donate-PayPal-blue.svg)](https://paypal.me/enbeg)

Donations are entirely voluntary and come with no obligation on either side: they do not buy
support, features or any guarantee. The driver is and will remain free.

## Legal

This project is not affiliated with, authorized or endorsed by Goodix or ASUS. "Goodix" is
used only to identify the hardware. The protocol was obtained by analysing the behaviour of
the device and of the Windows driver solely to achieve interoperability (EU Directive
2009/24/EC, art. 6). The repository contains no vendor code, firmware or binaries.

## License

[LGPL-2.1-or-later](LICENSE), the same license as libfprint.
