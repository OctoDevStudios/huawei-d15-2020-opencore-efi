/*
 * mac_hw.c — macOS hardware layer for the goodix51a0 port.
 *
 * Replaces Linux spidev + gpiochip with MateBookFP.kext:
 *   - gx_mac_ioctl() emulates SPI_IOC_MESSAGE(n) (CS "high" during the
 *     transfer, like SPI_CS_HIGH on Linux) and ignores config ioctls;
 *   - gx51_* (GPIO transport): reset = pad 189, IRQ = pad 204 (read/written
 *     through the kext; the Linux line number is ignored).
 */
#include <IOKit/IOKitLib.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>

#include "../src/gx51_transport.h"

struct spi_ioc_transfer {
    uint64_t tx_buf, rx_buf;
    uint32_t len, speed_hz;
    uint16_t delay_usecs;
    uint8_t bits_per_word, cs_change, tx_nbits, rx_nbits, word_delay_usecs, pad;
};

enum { kRd = 0, kWr = 1, kPads = 2, kReset = 3, kXfer = 4, kInfo = 5 };
#define SHARED_LEN 65536u

static io_connect_t C;
static uint8_t *SH;
static uint32_t cs_idle, cs_assert;

static uint32_t bar_rd(uint32_t off)
{
    uint64_t i = off, o = 0; uint32_t n = 1;
    IOConnectCallScalarMethod(C, kRd, &i, 1, &o, &n);
    return (uint32_t)o;
}
static void bar_wr(uint32_t off, uint32_t v)
{
    uint64_t i[2] = { off, v };
    IOConnectCallScalarMethod(C, kWr, i, 2, NULL, NULL);
}

/* SSP init identical to `fpctl init` (first validated contact). */
static void hw_init(void);

/* Connect to the kext (once) + (re)configure the SSP on every call:
 * after sleep, the LPSS SPI block comes back in reset and loses SSCR0/CS_CTRL. */
int gx_mac_open(void)
{
    if (!C) {
    io_service_t s = IOServiceGetMatchingService(kIOMainPortDefault, IOServiceMatching("MateBookFP"));
    if (!s) { fprintf(stderr, "MateBookFP.kext not loaded\n"); return -1; }
    kern_return_t kr = IOServiceOpen(s, mach_task_self(), 0, &C);
    IOObjectRelease(s);
    if (kr) { fprintf(stderr, "IOServiceOpen 0x%x (root required)\n", kr); C = 0; return -1; }
    mach_vm_address_t a = 0; mach_vm_size_t sz = 0;
    if (IOConnectMapMemory64(C, 0, mach_task_self(), &a, &sz, kIOMapAnywhere)) return -1;
    SH = (uint8_t *)a;
    }
    hw_init();
    return 0;
}

static void hw_init(void)
{
    if ((bar_rd(0x204) & 7) != 7) { bar_wr(0x204, 7); usleep(1000); }
    if (!(bar_rd(0x200) & 1)) { bar_wr(0x200, 0x80010003); usleep(100); }
    bar_wr(0x00, 0);
    bar_wr(0x04, 0x1dc0);
    bar_wr(0x28, 0); bar_wr(0x30, 0);
    unsigned scr = getenv("GXFP_SCR") ? (unsigned)strtoul(getenv("GXFP_SCR"), 0, 0) : 99u;
    bar_wr(0x00, ((scr & 0xfff) << 8) | 0x7);
    bar_wr(0x00, bar_rd(0x00) | (1u << 7));
    cs_idle = (bar_rd(0x224) & ~3u & ~(3u << 8)) | 1u;
    cs_assert = cs_idle | 2u;
    bar_wr(0x224, cs_idle);
}

/* Transfer through the kext. v0.2+: CS->clock delay done in the kernel (5th argument);
 * v0.1: rejects 5 arguments -> fall back to a userspace delay.
 * Failure (sleep, controller reset) -> reconfigure the SSP and retry once. */
static int kext_v2 = -1;

static kern_return_t xfer_once(uint32_t len, long setup_us)
{
    uint64_t o = 0; uint32_t n = 1;
    if (kext_v2 != 0) {
        uint64_t in[5] = { len, cs_assert, cs_idle, 2000000, (uint64_t)setup_us };
        kern_return_t kr = IOConnectCallScalarMethod(C, kXfer, in, 5, &o, &n);
        if (kr != kIOReturnBadArgument) { kext_v2 = 1; return kr; }
        kext_v2 = 0;
    }
    if (setup_us > 0) { bar_wr(0x224, cs_assert); usleep((useconds_t)setup_us); }
    uint64_t in[4] = { len, cs_assert, cs_idle, 2000000 };
    return IOConnectCallScalarMethod(C, kXfer, in, 4, &o, &n);
}

static int spi_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    static long setup_us = -1;
    if (!C && gx_mac_open()) return -1;
    if (len == 0 || len > SHARED_LEN) { errno = EINVAL; return -1; }
    if (setup_us < 0) setup_us = getenv("GXFP_CS_SETUP_US") ? atol(getenv("GXFP_CS_SETUP_US")) : 50;
    if (tx) memcpy(SH, tx, len); else memset(SH, 0, len);
    kern_return_t kr = xfer_once(len, setup_us);
    if (kr) {
        usleep(200000);                     /* never busy-loop */
        hw_init();
        if (tx) memcpy(SH, tx, len); else memset(SH, 0, len);
        kr = xfer_once(len, setup_us);
    }
    if (kr) { errno = EIO; return -1; }
    if (rx) memcpy(rx, SH, len);
    return (int)len;
}

int gx_mac_ioctl(int fd, unsigned long req, ...)
{
    (void)fd;
    va_list ap; va_start(ap, req); void *arg = va_arg(ap, void *); va_end(ap);
    if ((req & 0xffff0000u) == 0x53000000u) {           /* SPI_IOC_MESSAGE(n) */
        unsigned n = (unsigned)(req & 0xffff), total = 0;
        struct spi_ioc_transfer *x = arg;
        for (unsigned i = 0; i < n; i++) {
            if (spi_xfer((const uint8_t *)(uintptr_t)x[i].tx_buf, (uint8_t *)(uintptr_t)x[i].rx_buf, x[i].len) < 0)
                return -1;
            total += x[i].len;
        }
        return (int)total;
    }
    return 0;                                               /* SPI_IOC_WR_*: configured at open */
}

/* ---------------- GPIO ---------------- */

static int pads(uint64_t o[4])
{
    if (!C && gx_mac_open()) return -1;
    uint32_t n = 4;
    return IOConnectCallScalarMethod(C, kPads, NULL, 0, o, &n) ? -1 : 0;
}
int gx_mac_irq_level(void) { uint64_t o[4]; if (pads(o)) return -1; return (int)((o[2] >> 1) & 1); }
int gx_mac_set_reset(int high)
{
    if (!C && gx_mac_open()) return -1;
    uint64_t i = high ? 1 : 0, o = 0; uint32_t n = 1;
    return IOConnectCallScalarMethod(C, kReset, &i, 1, &o, &n) ? -1 : 0;
}

static int sleep_us(long us) { struct timespec t = { us / 1000000L, (us % 1000000L) * 1000L }; while (nanosleep(&t, &t) < 0) if (errno != EINTR) return -1; return 0; }

int gx51_open_irq_gpio48(void) { return gx_mac_open() ? -1 : 48; }

static int wait_level(unsigned timeout_ms, int want_high)
{
    if (timeout_ms == 0) return -1;
    for (unsigned t = 0; t < timeout_ms; t++) {
        int v = gx_mac_irq_level();
        if (v < 0) return -1;
        if (v == !!want_high) return 0;
        sleep_us(1000);
    }
    errno = ETIMEDOUT;
    return -1;
}
int gx51_wait_irq_gpio48(int fd, unsigned int ms) { (void)fd; return wait_level(ms, 1); }
int gx51_wait_irq_gpio48_low(int fd, unsigned int ms) { (void)fd; return wait_level(ms, 0); }

int gx51_reset_gpio264(void)
{
    if (gx_mac_set_reset(1)) return -1;
    sleep_us(300000);
    if (gx_mac_set_reset(0)) return -1;
    sleep_us(600000);
    return 0;
}

/* ---------------- framing (copy of gx51_transport.c, pure) ---------------- */

int gx51_make_outer(uint8_t type, size_t len, struct gx51_outer *out)
{
    if (!out || len == 0 || len > GX51_MAX_FRAME) return -1;
    out->type = type; out->len_lo = (uint8_t)(len & 0xff); out->len_hi = (uint8_t)(len >> 8);
    out->checksum = (uint8_t)(out->type + out->len_lo + out->len_hi);
    return 0;
}
uint8_t gx51_body_checksum(const uint8_t *d, size_t n) { uint8_t s = 0; while (n--) s += *d++; return (uint8_t)(0xAAu - s); }
size_t gx51_read_chunk_size(size_t r) { return r > GX51_SPI_READ_CHUNK_MAX ? GX51_SPI_READ_CHUNK_MAX : r; }
