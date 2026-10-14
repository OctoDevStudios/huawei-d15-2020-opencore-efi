/*
 * fpctl — userspace debug tool for MateBookFP.kext (GXFP51A0 on LPSS SPI1).
 *
 *   fpctl info                 kext / PCI state, SSP + LPSS registers, pads
 *   fpctl init [scr]           configure SSP: mode 0, 8 bits, SCR (default 99 ≈ 1 MHz)
 *   fpctl reset <0|1>          pad 189: 1 = MCU held in reset, 0 = MCU running
 *   fpctl irq                  level of pad 204
 *   fpctl xfer <cshigh 0|1> <hex...> [+N]   full-duplex transfer (+N zero bytes)
 *   fpctl probe                cases A–E of cshigh_repro.py (first contact)
 *   fpctl rd <off> | wr <off> <val>          raw BAR0 access (< 0x400)
 */
#include <IOKit/IOKitLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

enum { kRd = 0, kWr = 1, kPads = 2, kReset = 3, kXfer = 4, kInfo = 5 };

static io_connect_t C;
static uint8_t *SH;          /* 64 KB shared buffer */

static uint64_t call(uint32_t sel, const uint64_t *in, uint32_t nin, uint64_t *out, uint32_t nout, kern_return_t *kr)
{
    uint64_t o[4] = {0};
    uint32_t n = nout;
    kern_return_t k = IOConnectCallScalarMethod(C, sel, in, nin, out ? out : o, &n);
    if (kr) *kr = k;
    else if (k) fprintf(stderr, "  [!] sel %u -> 0x%x\n", sel, k);
    return out ? out[0] : o[0];
}
static uint32_t rd(uint32_t off) { uint64_t i = off, o = 0; call(kRd, &i, 1, &o, 1, NULL); return (uint32_t)o; }
static void wr(uint32_t off, uint32_t v) { uint64_t i[2] = { off, v }; call(kWr, i, 2, NULL, 0, NULL); }

static void pads(void)
{
    uint64_t o[4] = {0};
    call(kPads, NULL, 0, o, 4, NULL);
    printf("pad189 reset DW0=0x%08llx DW1=0x%08llx  TX=%llu  -> %s\n", o[0], o[1], o[0] & 1,
           (o[0] & 1) ? "MCU IN RESET" : "MCU running");
    printf("pad204 irq   DW0=0x%08llx DW1=0x%08llx  RX=%llu\n", o[2], o[3], (o[2] >> 1) & 1);
}
static int irq(void) { uint64_t o[4] = {0}; call(kPads, NULL, 0, o, 4, NULL); return (int)((o[2] >> 1) & 1); }
static void reset(int lvl) { uint64_t i = lvl, o = 0; call(kReset, &i, 1, &o, 1, NULL); }

static uint32_t cs_base(void) { return (rd(0x224) & ~3u & ~(3u << 8)) | 1u; /* SW mode, CS0 */ }

/* CS "high during transfer" (Linux SPI_CS_HIGH) or classic active-low */
static int xfer(const uint8_t *tx, uint32_t len, int cshigh, uint8_t *rx)
{
    uint32_t b = cs_base();
    uint32_t assert_v = cshigh ? (b | 2) : b, idle_v = cshigh ? b : (b | 2);
    memcpy(SH, tx, len);
    uint64_t in[4] = { len, assert_v, idle_v, 500000 }, o = 0;
    kern_return_t kr;
    call(kXfer, in, 4, &o, 1, &kr);
    if (rx) memcpy(rx, SH, len);
    if (kr) fprintf(stderr, "  [!] xfer %u -> 0x%x (done %llu)\n", len, kr, o);
    return kr ? -1 : 0;
}

static void dump(const uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i += 16) {
        int same = 1;
        for (uint32_t j = i; j < n; j++) if (p[j] != p[i]) { same = 0; break; }
        if (i && same) { printf("     ...   (%02x up to 0x%04x)\n", p[i], n - 1); return; }
        printf("     %04x ", i);
        for (uint32_t j = i; j < i + 16 && j < n; j++) printf(" %02x", p[j]);
        printf("  ");
        for (uint32_t j = i; j < i + 16 && j < n; j++) putchar(p[j] >= 32 && p[j] < 127 ? p[j] : '.');
        putchar('\n');
    }
}

static void init_ssp(uint32_t scr)
{
    printf("before: CLK=0x%08x RESETS=0x%08x SSCR0=0x%08x SSCR1=0x%08x CS=0x%08x\n",
           rd(0x200), rd(0x204), rd(0x00), rd(0x04), rd(0x224));
    if ((rd(0x204) & 7) != 7) { wr(0x204, 7); usleep(1000); }           /* take LPSS out of reset */
    if (!(rd(0x200) & 1)) { wr(0x200, 0x80010003); usleep(100); }        /* gate ON, M/N = 1/1 */
    wr(0x00, 0);                                                          /* SSE off */
    wr(0x04, 0x1dc0);                                                     /* mode 0, FIFO thresholds (Linux) */
    wr(0x28, 0); wr(0x30, 0);                                             /* SSTO, SSPSP */
    wr(0x00, ((scr & 0xfff) << 8) | 0x7);                                 /* 8 bits, Motorola, SCR */
    wr(0x00, rd(0x00) | (1u << 7));                                       /* SSE on */
    wr(0x224, cs_base());                                                 /* CS software mode, idle low */
    printf("after:  CLK=0x%08x RESETS=0x%08x SSCR0=0x%08x SSCR1=0x%08x SSSR=0x%08x CS=0x%08x\n",
           rd(0x200), rd(0x204), rd(0x00), rd(0x04), rd(0x08), rd(0x224));
}

static void frame(const uint8_t *body, uint32_t n, int cshigh)
{
    uint8_t h[4] = { 0xa0, (uint8_t)n, (uint8_t)(n >> 8), 0 };
    h[3] = (uint8_t)(h[0] + h[1] + h[2]);
    xfer(h, 4, cshigh, NULL); usleep(2000);
    xfer(body, n, cshigh, NULL); usleep(20000);
}

static void run_case(const char *tag, int rst, int cshigh, int ncmd, const int *cmds)
{
    static const uint8_t NOP[] = { 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88 };
    static const uint8_t EVK[] = { 0xa8, 0x03, 0x00, 0x00, 0x00, 0xff };
    static const uint8_t CHIP[] = { 0x82, 0x06, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x1e };
    reset(1); usleep(300000);
    int irq_rst = irq();
    reset(rst); usleep(600000);
    printf("\n== %s  reset=%d irq(reset)=%d irq(before)=%d cshigh=%d\n", tag, rst, irq_rst, irq(), cshigh);
    for (int i = 0; i < ncmd; i++) {
        if (cmds[i] == 0) frame(NOP, sizeof NOP, cshigh);
        if (cmds[i] == 1) frame(EVK, sizeof EVK, cshigh);
        if (cmds[i] == 2) frame(CHIP, sizeof CHIP, cshigh);
    }
    uint8_t z[256] = {0}, rx[256];
    xfer(z, 256, cshigh, rx);
    int odd = 0;
    for (int i = 0; i < 256; i++) if (rx[i] != 0 && rx[i] != 0xff) odd++;
    printf("   irq after=%d  non-idle bytes: %d\n", irq(), odd);
    if (odd) dump(rx, 256);
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: see the header of fpctl.c\n"); return 1; }
    io_service_t s = IOServiceGetMatchingService(kIOMainPortDefault, IOServiceMatching("MateBookFP"));
    if (!s) { fprintf(stderr, "MateBookFP.kext not loaded\n"); return 1; }
    kern_return_t kr = IOServiceOpen(s, mach_task_self(), 0, &C);
    IOObjectRelease(s);
    if (kr) { fprintf(stderr, "IOServiceOpen 0x%x (root ?)\n", kr); return 1; }
    mach_vm_address_t a = 0; mach_vm_size_t sz = 0;
    if (IOConnectMapMemory64(C, 0, mach_task_self(), &a, &sz, kIOMapAnywhere)) { fprintf(stderr, "map failed\n"); return 1; }
    SH = (uint8_t *)a;

    const char *cmd = argv[1];
    if (!strcmp(cmd, "info")) {
        uint64_t o[4] = {0};
        call(kInfo, NULL, 0, o, 4, NULL);
        printf("BAR0 phys=0x%llx len=0x%llx  PCI id=0x%08llx  PMCSR=0x%llx\n", o[0], o[1], o[2], o[3]);
        printf("SSCR0=0x%08x SSCR1=0x%08x SSSR=0x%08x SSTO=0x%08x SSPSP=0x%08x\n", rd(0), rd(4), rd(8), rd(0x28), rd(0x30));
        printf("LPSS CLK=0x%08x RESETS=0x%08x SSP_REG=0x%08x CS_CTRL=0x%08x CAPS=0x%08x\n",
               rd(0x200), rd(0x204), rd(0x220), rd(0x224), rd(0x2fc));
        pads();
    } else if (!strcmp(cmd, "init")) {
        init_ssp(argc > 2 ? (uint32_t)strtoul(argv[2], 0, 0) : 99);
    } else if (!strcmp(cmd, "reset") && argc > 2) {
        reset(atoi(argv[2])); pads();
    } else if (!strcmp(cmd, "irq")) {
        printf("%d\n", irq());
    } else if (!strcmp(cmd, "rd") && argc > 2) {
        printf("0x%08x\n", rd((uint32_t)strtoul(argv[2], 0, 0)));
    } else if (!strcmp(cmd, "wr") && argc > 3) {
        wr((uint32_t)strtoul(argv[2], 0, 0), (uint32_t)strtoul(argv[3], 0, 0));
    } else if (!strcmp(cmd, "xfer") && argc > 3) {
        static uint8_t tx[65536], rx[65536];
        uint32_t n = 0;
        for (int i = 3; i < argc; i++) {
            if (argv[i][0] == '+') { uint32_t k = atoi(argv[i] + 1); memset(tx + n, 0, k); n += k; continue; }
            for (const char *p = argv[i]; p[0] && p[1]; p += 2) { unsigned v; sscanf(p, "%2x", &v); tx[n++] = (uint8_t)v; }
        }
        if (!xfer(tx, n, atoi(argv[2]), rx)) dump(rx, n);
    } else if (!strcmp(cmd, "probe")) {
        init_ssp(99);
        const int A[] = { 0, 1, 2 }, E[] = { 1 };
        run_case("A  CS_HIGH, reset LOW, NOP+EVK+CHIPID", 0, 1, 3, A);
        run_case("B  CS low,  reset LOW, NOP+EVK+CHIPID", 0, 0, 3, A);
        run_case("C  CS_HIGH, reset HIGH, NOP+EVK+CHIPID", 1, 1, 3, A);
        run_case("D  CS_HIGH, reset LOW, no command", 0, 1, 0, A);
        run_case("E  CS_HIGH, reset LOW, EVK only", 0, 1, 1, E);
        run_case("A' repeat of A", 0, 1, 3, A);
        reset(1); usleep(300000); reset(0); usleep(300000);
        printf("\nreset left LOW (MCU running), irq=%d\n", irq());
        pads();
    } else { fprintf(stderr, "unknown command\n"); return 1; }
    return 0;
}
