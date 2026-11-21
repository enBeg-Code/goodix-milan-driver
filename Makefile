# goodix-milan-driver: libfprint TOD module for the Goodix GXFP3200 SPI fingerprint sensor

CC       ?= gcc
CFLAGS   ?= -O2 -g
PKGS      = libfprint-2-tod-1 glib-2.0 gobject-2.0 gio-2.0 gmodule-2.0
TARGET    = libfprint-2-tod-goodixmilan.so
TOD_DIR  ?= $(shell pkg-config --variable=tod_driversdir libfprint-2-tod-1)
UDEV_DIR ?= /etc/udev/rules.d
SRC       = src/goodixmilan.c src/gm-match.c
HDR       = src/gm-match.h

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CC) $(CFLAGS) -Wall -Wextra -Wno-unused-parameter -std=gnu11 -fPIC -shared \
	    -DGOODIXMILAN_TOD -o $@ $(SRC) $$(pkg-config --cflags --libs $(PKGS)) -lm

# shared library used by tools/eval_matcher.py
libgmmatch.so: src/gm-match.c $(HDR)
	$(CC) $(CFLAGS) -std=gnu11 -fPIC -shared -o $@ src/gm-match.c -lm

install: $(TARGET)
	@test -n "$(TOD_DIR)" || { echo "libfprint-2-tod-dev not found (pkg-config libfprint-2-tod-1)"; exit 1; }
	install -D -m 644 $(TARGET) $(DESTDIR)$(TOD_DIR)/$(TARGET)
	install -D -m 644 data/60-goodixmilan-spidev.rules $(DESTDIR)$(UDEV_DIR)/60-goodixmilan-spidev.rules

uninstall:
	rm -f $(DESTDIR)$(TOD_DIR)/$(TARGET) $(DESTDIR)$(UDEV_DIR)/60-goodixmilan-spidev.rules

clean:
	rm -f $(TARGET) libgmmatch.so

.PHONY: all install uninstall clean
