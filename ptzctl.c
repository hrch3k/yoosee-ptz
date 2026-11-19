/*
 * ptzctl - PTZ control for Yoosee/Gwell GM8135S cameras (e.g. Yoosee YYZ100S) on OpenIPC
 *
 * Open-source replacement for the vendor modules dsa.ko + gpio.ko: talks to the PTZ
 * microcontroller directly through the kernel's GPIO sysfs interface. No kernel module,
 * no /dev/mem needed.
 *
 * Bus (3 wires, open-drain style, each line has an internal pull-up):
 *   DATA = GPIO 52 (GPIO_1[20], pin X_GPIO_DAT[5])
 *   ACK  = GPIO 53 (GPIO_1[21], pin X_GPIO_DAT[6])
 *   CLK  = GPIO 54 (GPIO_1[22], pin X_GPIO_DAT[7])
 *   pin-mux: PMU 0x90c0005c bits [31:26] = 0 selects GPIO (reset default)
 *   A line is "released" by making it an input (pull-up -> 1) and "driven" by making it an
 *   output at 0. Every step waits for the other side (handshake); timeout 2 s, as in dsa.ko.
 *
 * Camera -> MCU (packet: [0]=length incl. itself, normally 7):
 *   start:  DATA low, wait ACK low, release DATA, wait ACK high
 *   bit:    set DATA (MSB first), wait ACK high, CLK low, wait ACK low, release CLK
 *   stop:   ACK low, wait CLK low, release DATA+ACK, wait CLK high
 * MCU -> camera (only when the MCU holds DATA low = data waiting):
 *   start:  ACK low, wait DATA high, release ACK
 *   bit:    wait CLK low, sample DATA, ACK low, wait CLK high, release ACK
 *   stop:   wait ACK low, release DATA, CLK low, wait ACK high, release CLK
 *
 * Packet: [0]=7 [1]=flags [2]=flags2 [3..4]=tilt BE16 [5..6]=pan BE16
 *   [1] 0x01 up, 0x02 down, 0x04 right, 0x08 left (relative, steps in [3..6])
 *       0x20 request travel range (reply 0x20: tilt range, pan range)
 *       0x40 request position     (reply 0x40: tilt pos,   pan pos)
 *       0x80 MCU soft reset (replies with status; does NOT re-run the homing sweep)
 *   [2] 0x10 go to absolute position [3..4]=tilt, [5..6]=pan
 * Measured on a YYZ100S: range tilt 0..215, pan 0..620 steps, park after power-on 108/330.
 * The MCU clamps its position counter at the range limits but still drives the motor, so
 * steps past a limit make the counter drift. ptzctl clamps every move to 0..range (-f bypasses).
 *
 * Protocol reverse-engineered from the behaviour of the vendor driver; this is an
 * independent implementation.
 *
 * Build: arm-unknown-linux-uclibcgnueabi-gcc -static -O2 -Wall -Wextra -o ptzctl ptzctl.c
 * SPDX-License-Identifier: MIT
 */
#define _GNU_SOURCE     /* pread() on uClibc */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define GPIO_DATA   52
#define GPIO_ACK    53
#define GPIO_CLK    54

#define PMU_BASE        0x90c00000UL
#define PMU_MFPSR3      0x5c
#define MFPSR3_PTZ_MASK 0xfc000000UL    /* bits 31:26 = X_GPIO_DAT[7:5] mux, 0 = GPIO */

#define TIMEOUT_US  2000000             /* per handshake step, same as the vendor driver */
#define LOCKFILE    "/var/lock/ptzctl.lock"

#define F_UP       0x01
#define F_DOWN     0x02
#define F_RIGHT    0x04
#define F_LEFT     0x08
#define F_STATUS   0x20
#define F_POS      0x40
#define F_RESET    0x80
#define F2_GOTO    0x10

static int verbose;

/* ---------------------------------------------------------------- GPIO (sysfs) */

struct line {
    int gpio;
    const char *name;
    int dir_fd;     /* /sys/class/gpio/gpioN/direction */
    int val_fd;     /* /sys/class/gpio/gpioN/value */
    int driven;     /* 1 = we drive it low, 0 = released (input) */
};

static struct line L_DATA = { GPIO_DATA, "DATA", -1, -1, -1 };
static struct line L_ACK  = { GPIO_ACK,  "ACK",  -1, -1, -1 };
static struct line L_CLK  = { GPIO_CLK,  "CLK",  -1, -1, -1 };

static int write_str(const char *path, const char *s)
{
    int fd = open(path, O_WRONLY), ok;
    if (fd < 0)
        return -1;
    ok = write(fd, s, strlen(s)) == (ssize_t)strlen(s);
    close(fd);
    return ok ? 0 : -1;
}

static int line_open(struct line *l)
{
    char p[64], n[8];

    snprintf(p, sizeof(p), "/sys/class/gpio/gpio%d/value", l->gpio);
    if (access(p, F_OK) != 0) {
        snprintf(n, sizeof(n), "%d", l->gpio);
        if (write_str("/sys/class/gpio/export", n) < 0 && errno != EBUSY) {
            fprintf(stderr, "cannot export GPIO %d (%s): %s\n", l->gpio, l->name, strerror(errno));
            return -1;
        }
    }
    snprintf(p, sizeof(p), "/sys/class/gpio/gpio%d/direction", l->gpio);
    l->dir_fd = open(p, O_WRONLY);
    snprintf(p, sizeof(p), "/sys/class/gpio/gpio%d/value", l->gpio);
    l->val_fd = open(p, O_RDONLY);
    if (l->dir_fd < 0 || l->val_fd < 0) {
        fprintf(stderr, "cannot open GPIO %d (%s) in sysfs: %s\n", l->gpio, l->name, strerror(errno));
        return -1;
    }
    return 0;
}

/* release = input, the pull-up makes the line read 1 */
static void line_release(struct line *l)
{
    if (l->driven != 0) {
        if (write(l->dir_fd, "in", 2) != 2)
            perror("gpio release");
        l->driven = 0;
    }
}

/* drive low = output at 0 ("low" sets direction and value in one step, no glitch) */
static void line_low(struct line *l)
{
    if (l->driven != 1) {
        if (write(l->dir_fd, "low", 3) != 3)
            perror("gpio drive");
        l->driven = 1;
    }
}

static int line_get(struct line *l)
{
    char c = '1';
    if (pread(l->val_fd, &c, 1, 0) != 1)
        perror("gpio read");
    return c == '1';
}

static long long now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* wait until the line reads 'level'; 0 = ok, -1 = timeout */
static int line_wait(struct line *l, int level, const char *step)
{
    long long t0 = now_us();
    int spins = 0;

    while (line_get(l) != level) {
        if (now_us() - t0 > TIMEOUT_US) {
            if (verbose)
                fprintf(stderr, "timeout: %s (waiting for %s=%d)\n", step, l->name, level);
            return -1;
        }
        if (++spins > 50)       /* busy-poll briefly, then yield */
            usleep(20);
    }
    return 0;
}

static void bus_release_all(void)
{
    line_release(&L_DATA);
    line_release(&L_ACK);
    line_release(&L_CLK);
}

/* the three pins must be in GPIO mode; the reset default is GPIO, check and fix if needed */
static void pinmux_check(void)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    volatile uint32_t *pmu;
    uint32_t v;

    if (fd < 0)
        return;     /* no /dev/mem: rely on the reset default */
    pmu = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PMU_BASE);
    if (pmu != MAP_FAILED) {
        v = pmu[PMU_MFPSR3 / 4];
        if (v & MFPSR3_PTZ_MASK) {
            fprintf(stderr, "pin-mux 0x%08x: PTZ pins not in GPIO mode, fixing\n", (unsigned)v);
            pmu[PMU_MFPSR3 / 4] = v & ~MFPSR3_PTZ_MASK;
        }
        munmap((void *)pmu, 4096);
    }
    close(fd);
}

static int bus_open(void)
{
    pinmux_check();
    if (line_open(&L_DATA) || line_open(&L_ACK) || line_open(&L_CLK))
        return -1;
    bus_release_all();
    return 0;
}

/* ---------------------------------------------------------------- bus protocol */

static int send_byte(uint8_t b)
{
    int i;
    for (i = 0; i < 8; i++, b <<= 1) {
        if (b & 0x80)
            line_release(&L_DATA);
        else
            line_low(&L_DATA);
        if (line_wait(&L_ACK, 1, "tx bit, ACK idle"))
            return -1;
        line_low(&L_CLK);
        if (line_wait(&L_ACK, 0, "tx bit, ACK latch")) {
            line_release(&L_CLK);
            return -1;
        }
        line_release(&L_CLK);
    }
    return 0;
}

static int recv_byte(uint8_t *b)
{
    int i;
    *b = 0;
    for (i = 0; i < 8; i++) {
        if (line_wait(&L_CLK, 0, "rx bit, CLK low"))
            return -1;
        *b = (*b << 1) | line_get(&L_DATA);
        line_low(&L_ACK);
        if (line_wait(&L_CLK, 1, "rx bit, CLK high"))
            return -1;
        line_release(&L_ACK);
    }
    return 0;
}

/* send pkt[0..pkt[0]-1]; 0 = accepted by the MCU */
static int bus_send(const uint8_t *pkt)
{
    int i, len = pkt[0];

    bus_release_all();
    if (!line_get(&L_DATA) || !line_get(&L_ACK) || !line_get(&L_CLK)) {
        if (verbose)
            fprintf(stderr, "bus busy (a line is held low)\n");
        return -1;
    }
    /* start */
    line_low(&L_DATA);
    if (line_wait(&L_ACK, 0, "tx start, ACK low"))
        goto fail;
    line_release(&L_DATA);
    if (line_wait(&L_ACK, 1, "tx start, ACK high"))
        goto fail;
    /* bytes */
    for (i = 0; i < len; i++)
        if (send_byte(pkt[i]))
            goto fail;
    /* stop */
    line_low(&L_ACK);
    if (line_wait(&L_CLK, 0, "tx stop, CLK low"))
        goto fail;
    line_release(&L_DATA);
    line_release(&L_ACK);
    if (line_wait(&L_CLK, 1, "tx stop, CLK high"))
        goto fail;
    return 0;
fail:
    bus_release_all();
    return -1;
}

/* receive one packet if the MCU has one waiting; returns length, 0 = nothing, -1 = error */
static int bus_recv(uint8_t *buf)
{
    int i, len;

    memset(buf, 0, 256);
    bus_release_all();
    if (line_get(&L_DATA))
        return 0;                       /* DATA high: MCU has nothing for us */
    /* start */
    line_low(&L_ACK);
    if (line_wait(&L_DATA, 1, "rx start, DATA high"))
        goto fail;
    line_release(&L_ACK);
    /* length byte, then the rest */
    if (recv_byte(&buf[0]))
        goto fail;
    len = buf[0];
    for (i = 1; i < len; i++)
        if (recv_byte(&buf[i]))
            goto fail;
    /* stop */
    if (line_wait(&L_ACK, 0, "rx stop, ACK low"))
        goto fail;
    line_release(&L_DATA);
    line_low(&L_CLK);
    if (line_wait(&L_ACK, 1, "rx stop, ACK high"))
        goto fail;
    line_release(&L_CLK);
    return len;
fail:
    bus_release_all();
    return -1;
}

/* ---------------------------------------------------------------- commands */

static int mcu_send(uint8_t f1, uint8_t f2, uint16_t tilt, uint16_t pan)
{
    uint8_t pkt[7] = { 7, f1, f2, tilt >> 8, tilt & 0xff, pan >> 8, pan & 0xff };
    if (bus_send(pkt)) {
        fprintf(stderr, "MCU did not accept the packet (bus busy or timeout)\n");
        return -1;
    }
    return 0;
}

static void dump(const uint8_t *b, int len)
{
    int i;
    printf("rx %d bytes:", len);
    for (i = 0; i < len; i++)
        printf(" %02x", b[i]);
    printf("\n");
}

/* wait for a reply with the given flag, polling every 50 ms */
static int wait_reply(uint8_t flag, uint8_t *buf, int timeout_ms)
{
    int t, len;
    for (t = 0; t < timeout_ms; t += 50) {
        len = bus_recv(buf);
        if (len > 0) {
            if (verbose)
                dump(buf, len);
            if (buf[1] & flag)
                return len;
        }
        usleep(50000);
    }
    return 0;
}

static int force;

static int query(uint8_t flag, unsigned *tilt, unsigned *pan)
{
    uint8_t buf[256];
    if (mcu_send(flag, 0, 0, 0) < 0)
        return -1;
    if (!wait_reply(flag, buf, 2000))
        return -1;
    *tilt = buf[3] << 8 | buf[4];
    *pan  = buf[5] << 8 | buf[6];
    return 0;
}

static long clampl(long v, long lo, long hi) { return v < lo ? lo : v > hi ? hi : v; }

static int safe_goto(long tgt_t, long tgt_p)
{
    unsigned rt, rp, pt, pp;
    long ct, cp;

    if (force)
        return mcu_send(0, F2_GOTO, (uint16_t)tgt_t, (uint16_t)tgt_p);
    if (query(F_STATUS, &rt, &rp) || rt == 0 || rp == 0) {
        fprintf(stderr, "cannot read travel range from MCU - not moving (use -f to force)\n");
        return -1;
    }
    if (query(F_POS, &pt, &pp)) {
        fprintf(stderr, "cannot read position from MCU - not moving (use -f to force)\n");
        return -1;
    }
    ct = clampl(tgt_t, 0, rt);
    cp = clampl(tgt_p, 0, rp);
    if (ct != tgt_t || cp != tgt_p)
        printf("limit: target %ld/%ld clamped to %ld/%ld (range 0..%u / 0..%u)\n",
               tgt_t, tgt_p, ct, cp, rt, rp);
    if ((unsigned)ct == pt && (unsigned)cp == pp) {
        printf("already at %u/%u\n", pt, pp);
        return 0;
    }
    printf("moving %u/%u -> %ld/%ld\n", pt, pp, ct, cp);
    return mcu_send(0, F2_GOTO, (uint16_t)ct, (uint16_t)cp);
}

static int safe_rel(long dt, long dp)
{
    unsigned pt, pp;
    if (force) {
        uint8_t f = (dt > 0 ? F_UP : dt < 0 ? F_DOWN : 0) | (dp > 0 ? F_RIGHT : dp < 0 ? F_LEFT : 0);
        return mcu_send(f, 0, labs(dt) & 0xffff, labs(dp) & 0xffff);
    }
    if (query(F_POS, &pt, &pp)) {
        fprintf(stderr, "cannot read position from MCU - not moving (use -f to force)\n");
        return -1;
    }
    return safe_goto((long)pt + dt, (long)pp + dp);
}

static void usage(const char *p)
{
    printf("usage: %s [-f] [-v] COMMAND   (-f = no limit check, -v = show bus traffic)\n", p);
    printf("  %s up|down|left|right STEPS   relative move\n"
           "  %s move TILT PAN              both axes (+ = up/right, - = down/left)\n"
           "  %s goto TILT PAN              absolute position (preset)\n"
           "  %s pos                        current position\n"
           "  %s range                      travel range\n"
           "  %s reset                      MCU soft reset (0x80); power-cycle to re-home\n"
           "  %s listen [SECONDS]           print everything the MCU sends\n",
           p, p, p, p, p, p, p);
}

int main(int argc, char **argv)
{
    uint8_t buf[256];
    int ret = 0, lock;
    const char *prog = argv[0];

    while (argc > 1 && argv[1][0] == '-') {
        if (!strcmp(argv[1], "-f"))      force = 1;
        else if (!strcmp(argv[1], "-v")) verbose = 1;
        else { usage(prog); return 1; }
        argv++;
        argc--;
    }
    if (argc < 2) {
        usage(prog);
        return 1;
    }

    /* one bus user at a time */
    lock = open(LOCKFILE, O_CREAT | O_RDWR, 0644);
    if (lock >= 0)
        flock(lock, LOCK_EX);

    if (access("/dev/npc_dsa", F_OK) == 0)
        fprintf(stderr, "warning: vendor dsa.ko is loaded; unload it (rmmod dsa hi_gpio)\n");
    if (bus_open())
        return 1;

    if (!strcmp(argv[1], "up") || !strcmp(argv[1], "down") ||
        !strcmp(argv[1], "left") || !strcmp(argv[1], "right")) {
        long n = argc > 2 ? strtol(argv[2], NULL, 0) : 30;
        if (n < 0) n = -n;
        if (n > 0xffff) n = 0xffff;
        if (argv[1][0] == 'u')      ret = safe_rel(n, 0);
        else if (argv[1][0] == 'd') ret = safe_rel(-n, 0);
        else if (argv[1][0] == 'r') ret = safe_rel(0, n);
        else                        ret = safe_rel(0, -n);
    } else if (!strcmp(argv[1], "move") && argc > 3) {
        ret = safe_rel(strtol(argv[2], NULL, 0), strtol(argv[3], NULL, 0));
    } else if (!strcmp(argv[1], "goto") && argc > 3) {
        ret = safe_goto(strtol(argv[2], NULL, 0), strtol(argv[3], NULL, 0));
    } else if (!strcmp(argv[1], "range")) {
        unsigned rt, rp;
        if (query(F_STATUS, &rt, &rp) == 0)
            printf("travel range: tilt 0..%u  pan 0..%u\n", rt, rp);
        else { printf("no range reply\n"); ret = 1; }
    } else if (!strcmp(argv[1], "pos")) {
        unsigned pt, pp;
        if (query(F_POS, &pt, &pp) == 0)
            printf("position: tilt %u  pan %u\n", pt, pp);
        else { printf("no position reply\n"); ret = 1; }
    } else if (!strcmp(argv[1], "reset")) {
        if ((ret = mcu_send(F_RESET, 0, 0, 0)) == 0 && wait_reply(F_STATUS, buf, 5000))
            printf("MCU reset, travel range: tilt %u  pan %u\n",
                   buf[3] << 8 | buf[4], buf[5] << 8 | buf[6]);
    } else if (!strcmp(argv[1], "listen")) {
        int s = argc > 2 ? atoi(argv[2]) : 10, t, len;
        for (t = 0; t < s * 20; t++) {
            len = bus_recv(buf);
            if (len > 0) dump(buf, len);
            usleep(50000);
        }
    } else {
        usage(prog);
        ret = 1;
    }
    bus_release_all();
    return ret ? 1 : 0;
}
