# Goodix Milan F (GXFP3200) SPI protocol

This document describes what is known about the fingerprint sensor found in the
ASUSPRO P5440FA (ExpertBook P5440) and how this driver talks to it. It was obtained
by observing the hardware from Linux and by analysing the ASUS Windows driver for
interoperability purposes. No vendor code is included in this repository.

## Hardware

| | |
|---|---|
| ACPI path | `\_SB.PCI0.SPI0.FPRT` |
| ACPI HID | `GXFP3200` (the BIOS returns it when its variable `FPVD == 0x22`; other units of the same model may ship an ELAN sensor, `ELAN7001`) |
| Bus | Intel LPSS SPI controller (`pxa2xx-spi`, PCI `00:1e.2`), chip select 0 |
| SPI | mode 0 (CPOL 0, CPHA 0), 8 bit words, CS active low, 4 MHz (`_CRS`); 8 MHz also works |
| Reset GPIO | `GPP_C15`, output, driven high by firmware (sensor out of reset at boot) |
| IRQ GPIO | `GPP_D16`, edge, active high (not needed: the driver polls the IRQ status register) |
| Chip ID | `0x002202a0` |
| Pixel array | 108 x 88, 12 bit, roughly 508 dpi |

Note: on the same laptop `spi1.0` is the BIOS flash (`spi-nor`). Never bind anything to it.

## Linux access

The sensor has no kernel driver. Binding the generic `spidev` driver exposes it as
`/dev/spidevN.0`:

```
echo spidev > /sys/bus/spi/devices/spi-GXFP3200:00/driver_override
echo spi-GXFP3200:00 > /sys/bus/spi/drivers/spidev/bind
```

`data/60-goodixmilan-spidev.rules` does this at boot, in the same form libfprint uses for
its ELAN SPI sensors.

## Transactions

Every line is one SPI message (chip select held low for its whole duration).

| Operation | Bytes |
|---|---|
| Set address | `F0 aH aL` |
| Read *n* bytes | set address, then `F1` followed by *n* read bytes |
| Write 16-bit register | `F0 aH aL 00 01 vH vL` (the `00 01` is the length in 16-bit words) |
| Command | single byte |

Register values are 16 bit, big endian. Reading 4 bytes at `0x0000` returns the chip ID as
two 16-bit words, low word first: bytes `02 a0 00 22` mean `0x002202a0`.

### Commands

| Byte | Meaning (from the Windows driver) |
|---|---|
| `C0` | idle |
| `C1` | finger detection (FDT) mode |
| `C2` | start an image capture |
| `C8` | sleep |
| `A1`, `CA` | used during FDT calibration and navigation modes |

### Registers used by this driver

| Address | Use |
|---|---|
| `0x0000` | chip ID (4 bytes) |
| `0x0124` | IRQ clear: write the bits to acknowledge |
| `0x0126` | IRQ status: `0x08` image ready, `0x02` FDT event, `0x10` ESD, `0x100` reset happened |
| `0x0050`, `0x0052`, `0x0054`, `0x005C`, `0x0120` | image capture configuration |
| `0x0072`, `0x0074` | scrambling seed (low, high word); always 0 on this unit |
| `0xAAAA` | image FIFO |

## Image capture

```
C0                              idle; wait 1 ms
read 0x0126 -> st; if st: write 0x0124 = st      clear pending flags (e.g. 0x100 after power-on)
write 0x0124 = 0x00FF
write 0x005C = 0x0080
write 0x0120 = 0x0408
write 0x0052 = 0x0008
write 0x0050 = 0x0501
C2                              start capture; wait 1 ms
write 0x0054 = 0x0110
poll 0x0126 until bit 0x08      about 30 ms
write 0x0124 = status
set address 0xAAAA, then read 14260 bytes with F1 (may be split in chunks, e.g. 4092 bytes,
                                each chunk being a separate `F1 + read` message)
read 0x0074, 0x0072             scrambling seed
C0
```

The whole sequence takes about 40 ms at 4 MHz. No initialisation is needed after power-on:
the sensor works with its reset defaults and the capture configuration above.

### Frame format

14260 bytes = 14256 bytes of pixels + 4 bytes of CRC.

Pixels are packed 6 bytes for 4 pixels of 12 bit:

```
p0 = b1 | (b0 & 0x0F) << 8
p1 = b3 << 4 | b0 >> 4
p2 = b2 | (b5 & 0x0F) << 8
p3 = b4 << 4 | b5 >> 4
```

Rows are 108 pixels, 88 rows.

The CRC is CRC-32/MPEG-2 (poly `0x04C11DB7`, init `0xFFFFFFFF`, no reflection, no final XOR)
over the 14256 pixel bytes, stored as two big-endian 16-bit words, low word first:
`crc >> 8, crc, crc >> 24, crc >> 16`.

The Windows driver can descramble frames by XORing them with a pseudo-random sequence seeded
by registers `0x0072`/`0x0074`. On this hardware the seed is always 0 (no scrambling), so the
driver rejects frames with a non-zero seed instead of implementing it.

### Images

An empty sensor gives a flat image with horizontal banding. The finger is obtained by
subtracting a frame from an empty-sensor background frame; ridges are then clearly visible.
Mean absolute difference: about 3.5 without a finger (noise), about 500 with a finger.

Finger presence is detected without a background frame from the "ridge energy" of the raw
frame (row means removed, band-pass filtered, standard deviation): about 31 with an empty
sensor, 52-56 with a finger.

## Matching

NBIS (the minutiae matcher used by most libfprint image drivers) finds only 1-2 minutiae on
a 108 x 88 image and cannot be used. The driver uses its own matcher, `src/gm-match.c`:
single-scale ORB-like keypoints (FAST-9, intensity-centroid orientation, steered BRIEF on a
3x upscaled image) and RANSAC verification of a similarity transform. The score is the number
of geometrically consistent correspondences; the match threshold is 12.

On 50 images of 4 fingers (one person): no false accepts (impostor maximum 7-9) and 7-12%
false rejects per attempt with a 12-image template. More data from more people is needed to
validate these numbers.

## Sensor family

The Windows driver recognises these chip IDs (`chip_id >> 8`); only `0x2202` has been tested:

| ID | Sensor type | Pixels (W x H) |
|---|---|---|
| `0x2202` | 0 | 108 x 88 |
| `0x2207` | 1 | 176 x 64 |
| `0x2208` | 2 | 176 x 54 |
| `0x220C` | 11 | 108 x 88 |
| `0x220F` | 12 | 80 x 64 |

## Other configurations seen in the Windows driver

Register sets written for other sensor modes (type 0 sensors), as `address=value`.
Not needed by this driver; listed for future work (finger detection mode, calibration).

| Mode | Registers |
|---|---|
| 0 | `0120=0482 0020=4010 0022=2001 0024=0014 005c=0100 0228=0000 022a=0000 0082=1580 0080=0001` |
| 1 | `0120=0482 0020=4010 0022=2001 0024=0014 005c=0100 0228=0000 022a=0000 0082=1380 0080=0003` |
| 2 | `00ca=0007 0020=4010 0024=0080 0056=2808 0058=0148 00ba=0001 0084=8180 0086=8c80 0088=9780 008a=b080 008c=8680 008e=8c80 0090=a080 0092=b380 0094=8480 0096=8880 0098=a080 009a=b880 0070=0000 00d0=0000` |
| 3 | `0120=0402 0020=4010 0022=2001 0024=0032 005c=0080 0228=0000 022a=0000 0082=0c80 0080=0001` |
| 4 | `0120=0408 0022=0810 005c=0080 0228=0000 022a=0000 0082=0c80 0080=0001` |
| 5 (capture) | `005c=0080 0120=0408 0052=0008 0050=0501` |
| 6 | `005c=0080 0120=0408 0052=0008 0050=0501` |
| 7 | `005c=0080 0120=0408 0052=0004 0050=0501 0054=0100` |

The Windows driver also reads a calibration value from the sensor OTP and writes it to
`0x005C` before capturing; images are good with the default `0x0080`.
