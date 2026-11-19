CROSS_COMPILE ?= arm-unknown-linux-uclibcgnueabi-
CC      = $(CROSS_COMPILE)gcc
STRIP   = $(CROSS_COMPILE)strip
CFLAGS  = -static -O2 -Wall -Wextra
BUILD   = build

all: $(BUILD)/ptzctl

$(BUILD)/ptzctl: ptzctl.c
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $<
	$(STRIP) $@

# make deploy CAMERA=<camera-ip>   (or export CAMERA in your shell)
deploy: $(BUILD)/ptzctl
	@test -n "$(CAMERA)" || { echo "usage: make deploy CAMERA=<camera-ip>"; exit 1; }
	scp -O $(BUILD)/ptzctl root@$(CAMERA):/tmp/ptzctl

clean:
	rm -rf $(BUILD)

.PHONY: all deploy clean
