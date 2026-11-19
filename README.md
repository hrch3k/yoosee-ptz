# ptzctl (open)

PTZ control for Yoosee/Gwell GM8135S cameras (Yoosee YYZ100S) on OpenIPC, without the
vendor kernel modules `dsa.ko` / `gpio.ko`. Drives the 3-wire bus to the PTZ
microcontroller directly via `/sys/class/gpio` (kernel option `CONFIG_GPIO_SYSFS`).

| Signal | GPIO | SoC pin |
|---|---|---|
| DATA | 52 | GPIO_1[20] / X_GPIO_DAT[5] |
| ACK  | 53 | GPIO_1[21] / X_GPIO_DAT[6] |
| CLK  | 54 | GPIO_1[22] / X_GPIO_DAT[7] |

Protocol description: see the header of `ptzctl.c`.

## Usage
```
ptzctl [-f] [-v] COMMAND [ARGS]
```

### Options
| Option | Effect |
|---|---|
| `-f` | **force**: skip the limit check and send the raw command to the MCU (see *Limits* below) |
| `-v` | **verbose**: print every packet received from the MCU, and on failure which handshake step timed out |

### Commands
| Command | What it does |
|---|---|
| `up STEPS` / `down STEPS` | tilt by STEPS motor steps (default 30) |
| `left STEPS` / `right STEPS` | pan by STEPS motor steps (default 30) |
| `move TILT PAN` | relative move on both axes at once; positive = up / right, negative = down / left |
| `goto TILT PAN` | move to an absolute position (e.g. a preset) |
| `pos` | print the current position |
| `range` | print the travel range (the MCU measures it during its power-on sweep) |
| `reset` | MCU soft reset (command 0x80); the MCU answers with its range. It does **not** re-run the homing sweep, only a power cycle does |
| `listen [SECONDS]` | print every packet the MCU sends for SECONDS (default 10); for debugging |

Exit status is 0 on success, 1 on any error (no answer from the MCU, bus timeout, bad arguments).

Examples:
```
ptzctl pos                  # position: tilt 108  pan 330
ptzctl range                # travel range: tilt 0..215  pan 0..620
ptzctl right 50             # moving 108/330 -> 108/380
ptzctl move -20 100         # 20 steps down and 100 right
ptzctl goto 108 330         # back to the park position
```

### Coordinates
Positions are motor steps counted from the end stops: tilt 0 = fully down, pan 0 = fully left.
On a Yoosee YYZ100S the range is tilt **0..215** and pan **0..620**; after power-on the MCU
parks the camera at **108 / 330**. The motors move about 55-60 steps per second.

### Limits (endpoint clamping)
After power-on the MCU drives both axes against their mechanical stops to find the travel
range ("homing"). From then on it counts steps, but it does **not** protect the limits itself:
when asked to go past an end it stops counting at the limit while the motor keeps turning
against the stop. The position counter then no longer matches the real position, and only a
power cycle (new homing sweep) fixes that.

ptzctl therefore checks every move before sending it:

1. read the travel range (0x20) and the current position (0x40) from the MCU
2. compute the target; relative moves are turned into an absolute target
3. clamp the target to `0..range` on each axis, and say so:
   `limit: target 108/700 clamped to 108/620 (range 0..215 / 0..620)`
4. if the clamped target equals the current position, do nothing: `already at 108/620`
5. otherwise send one absolute move: `moving 108/380 -> 108/620`

If the MCU does not answer the range or position query, ptzctl refuses to move.
`-f` skips all of this and sends the command unchanged (relative commands as relative steps),
for testing only.

Only one ptzctl talks to the bus at a time (lock file `/var/lock/ptzctl.lock`).

## Toolchain
`ptzctl` is plain C with no dependencies and is linked statically, so any ARMv5 (ARM926EJ-S)
Linux cross compiler works. Set `CROSS_COMPILE` to its prefix.

Tested with:

| Toolchain | Prefix | Where to get it |
|---|---|---|
| Grain Media GM8136 SDK, GCC 4.4.0, uClibc (Buildroot 2012.02) | `arm-unknown-linux-uclibcgnueabi-` | GM8136 SDK (`toolchain_gnueabi-4.4.0_ARMv5TE`) |

Should also work with (not tested yet):

| Toolchain | Prefix | Where to get it |
|---|---|---|
| OpenIPC toolchain for GM8136 | `arm-openipc-linux-uclibcgnueabi-` | https://github.com/openipc/firmware/releases/download/toolchain/toolchain.grainmedia-gm8136.tgz |

## Build
```
export PATH=<toolchain>/bin:$PATH
make                                    # -> build/ptzctl (static, stripped)
make CROSS_COMPILE=<prefix>             # other toolchain
make deploy CAMERA=<camera-ip>          # copies to the camera as /tmp/ptzctl
```

## Test
```
rmmod dsa; rmmod hi_gpio        # if the vendor modules are loaded, unload them first
/tmp/ptzctl -v pos
/tmp/ptzctl range
/tmp/ptzctl right 50
/tmp/ptzctl goto 108 330
```
