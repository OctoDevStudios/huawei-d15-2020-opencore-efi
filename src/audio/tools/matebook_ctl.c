#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <IOKit/IOKitLib.h>

/* Must match MateBookAudioDevice.hpp selectors */
enum {
    kSel_StartPlayback  = 0,
    kSel_StopPlayback   = 1,
    kSel_GetPosition    = 2,
    kSel_UpdateSPIB     = 3,
    kSel_StartCapture   = 4,
    kSel_StopCapture    = 5,
    kSel_GetCapPosition = 6,
    kSel_TriggerInitDSP = 7,
    kSel_ReadBAR0_8     = 8,
    kSel_ReadBAR0_16    = 9,
    kSel_ReadBAR0_32    = 10,
    kSel_WriteBAR0_8    = 11,
    kSel_WriteBAR0_16   = 12,
    kSel_WriteBAR0_32   = 13,
    kSel_ReadBAR4_8     = 14,
    kSel_ReadBAR4_16    = 15,
    kSel_ReadBAR4_32    = 16,
    kSel_WriteBAR4_8    = 17,
    kSel_WriteBAR4_16   = 18,
    kSel_WriteBAR4_32   = 19,
    kSel_ReadBAR4_64    = 20,
    kSel_WriteBAR4_64   = 21,
    kSel_ReadGPIO32     = 22,
    kSel_WriteGPIO32    = 23,
    kSel_I2CWriteReg    = 24,
    kSel_I2CReadReg     = 25,
    kSel_GetPhysicalAddrs = 26,
    kSel_ForceResetDSP    = 27,
    kSel_InitDSPFromStaging = 28,
    kSel_GetFwPhys        = 29,
    kSel_SetFwStagingLen  = 30
};

enum {
    kMemPlayback = 0,
    kMemCapture  = 1,
    kMemFlags    = 2,
    kMemFirmware = 3
};

#define FW_STAGING_SIZE  (600u * 1024u)

/* SOF IPC mailbox / SSP (ABI 3.x) — mirrors dma_capture helpers */
#define OUTBOX              0x82000u
#define IPC_HIPCTDR         0xC0u
#define IPC_HIPCTDA         0xC4u
#define IPC_HIPCIDR         0xD0u
#define IPC_HIPCIDA         0xD4u
#define IPC_HIPCCTL         0xE8u
#define IPC_BUSY_DONE       (1u << 31)
#define DAI_SSP             1u
/* sof_ipc_dai_ssp_params.clks_control. The Linux topology for this board
 * carries no SSP vendor tokens, so every one of clks_control / quirks /
 * mclk_id / frame_pulse_width / bclk_delay is 0. The old (1u << 8) here was a
 * reserved bit, not MCLK_AON (which is BIT(4)). */
#define SOF_SSP_CLKS_CONTROL  0u

/* Amp enables, from the Ubuntu pinctrl dump on this machine with sound
 * confirmed audible: gpio-559 "speakers-enable" = pin 40 = GPP_B15 and
 * gpio-560 "headphone-enable" = pin 41 = GPP_B16. GPP_B is pins 25..50, which
 * /proc/iomem puts in community 0 (PCR port 0x6E). padcfg = PADBAR(0xA50) +
 * pin*16. The old C1 0xB50/0xB60 were pads 16/17 of community 1, i.e.
 * GPP_D16/D17 = ISH_UART0_CTSB and DMIC_CLK_1.
 *
 * PADCFG0 bit 0 (GPIOTXSTATE) 1 = amp ON: Linux goes 0x44000200 -> 0x44000201
 * when the internal speakers come up, and drives BOTH pads together (speakers
 * = both 1, headphones = both 0, since the headphone output uses the codec's
 * own charge pump).
 *
 * High 16 bits = community for the kext. */
#define GPIO_COMM0       0x00000000u
#define GPIO_COMM1       0x00010000u
#define GPIO_SPK_PADCFG  (GPIO_COMM0 | 0x0CD0u)   /* pin 40, GPP_B15 */
#define GPIO_HP_PADCFG   (GPIO_COMM0 | 0x0CE0u)   /* pin 41, GPP_B16 */
#define CAP_BUFFER_SIZE  131072u  /* 2ch S32 × 16384 frames */
#define CAP_CHANNELS     2
#define ROM_STATUS       0x80000u
#define ROM_STATUS_EX    0x80004u
#define FSR_FW_ENTERED   0x5u

/* ES8336 one-shot I2C map — must match ES8336Codec.hpp */
#define ES8316_RESET           0x00
#define ES8316_RESET_CSM_ON    0x80
#define ES8316_CLKMGR_CLKSW    0x01
#define ES8316_CLKMGR_CLKSEL   0x02
#define ES8316_CLKMGR_ADCOSR   0x03
#define ES8316_CLKMGR_ADCDIV1  0x04
#define ES8316_CLKMGR_ADCDIV2  0x05
#define ES8316_CLKMGR_DACDIV1  0x06
#define ES8316_CLKMGR_DACDIV2  0x07
#define ES8316_CLKMGR_CPDIV    0x08
#define ES8316_SERDATA1        0x09
#define ES8316_SERDATA_ADC     0x0A
#define ES8316_SERDATA_DAC     0x0B
#define ES8316_SYS_VMIDSEL     0x0C
#define ES8316_SYS_PDN         0x0D
#define ES8316_SYS_LP1         0x0E
#define ES8316_SYS_LP2         0x0F
#define ES8316_SYS_REF         0x12
#define ES8316_HPMIX_SEL       0x13
#define ES8316_HPMIX_SWITCH    0x14
#define ES8316_HPMIX_PDN       0x15
#define ES8316_HPMIX_VOL       0x16
#define ES8316_CPHP_OUTEN      0x17
#define ES8316_CPHP_ICAL_VOL   0x18
#define ES8316_CPHP_PDN1       0x19
#define ES8316_CPHP_PDN2       0x1A
#define ES8316_CPHP_LDOCTL     0x1B
#define ES8316_DAC_PDN         0x2F
#define ES8316_DAC_SET1        0x30
#define ES8316_DAC_SET2        0x31
#define ES8316_DAC_SET3        0x32
#define ES8316_DAC_VOLL        0x33
#define ES8316_DAC_VOLR        0x34
#define ES8316_GPIO_DEBOUNCE   0x4E

static void print_cfprop(io_service_t svc, CFStringRef key, const char *label) {
    CFTypeRef prop = IORegistryEntryCreateCFProperty(svc, key, kCFAllocatorDefault, 0);
    if (!prop) return;
    char buf[256] = {0};
    if (CFGetTypeID(prop) == CFStringGetTypeID()) {
        CFStringGetCString((CFStringRef)prop, buf, sizeof(buf), kCFStringEncodingUTF8);
        printf("  %s: %s\n", label, buf);
    } else if (CFGetTypeID(prop) == CFDataGetTypeID()) {
        CFDataRef d = (CFDataRef)prop;
        const UInt8 *bytes = CFDataGetBytePtr(d);
        CFIndex n = CFDataGetLength(d);
        if (bytes && n > 0 && n < (CFIndex)sizeof(buf)) {
            memcpy(buf, bytes, (size_t)n);
            buf[n < 255 ? n : 255] = 0;
            printf("  %s: %s\n", label, buf);
        }
    }
    CFRelease(prop);
}

static void dump_hdas_props(void) {
    CFMutableDictionaryRef hdasMatch = IOServiceNameMatching("HDAS");
    io_service_t hdas = IOServiceGetMatchingService(kIOMainPortDefault, hdasMatch);
    if (!hdas) {
        printf("[-] HDAS not found\n");
        return;
    }
    printf("[*] HDAS registry props:\n");
    print_cfprop(hdas, CFSTR("MateBookAudioProbed"), "Probed");
    print_cfprop(hdas, CFSTR("MateBookAudioStarted"), "Started");
    print_cfprop(hdas, CFSTR("MateBookAudio_Step"), "Step");
    print_cfprop(hdas, CFSTR("MateBookAudioDSPStatus"), "DSPStatus");
    print_cfprop(hdas, CFSTR("MateBookAudioCodec"), "Codec");
    print_cfprop(hdas, CFSTR("MateBookAudio_DSPFail"), "DSPFail");
    print_cfprop(hdas, CFSTR("MateBookAudio_ROMStatus"), "ROMStatus");
    IOObjectRelease(hdas);
}

static void dump_device_props(io_service_t service) {
    printf("[*] MateBookAudioDevice props:\n");
    print_cfprop(service, CFSTR("Status"), "Status");
    print_cfprop(service, CFSTR("DSPStatus"), "DSPStatus");
    print_cfprop(service, CFSTR("Codec"), "Codec");
    print_cfprop(service, CFSTR("CodecStatus"), "CodecStatus");
    print_cfprop(service, CFSTR("FW-Version"), "FW-Version");
    print_cfprop(service, CFSTR("Topology"), "Topology");
}

static uint32_t amp_padcfg(io_connect_t c, uint32_t pin);

static kern_return_t gpio_rd(io_connect_t c, uint32_t off, uint32_t *val) {
    uint64_t in = off, out = 0;
    uint32_t outCnt = 1;
    kern_return_t kr = IOConnectCallScalarMethod(c, kSel_ReadGPIO32, &in, 1, &out, &outCnt);
    if (kr == KERN_SUCCESS) *val = (uint32_t)out;
    return kr;
}

static kern_return_t gpio_wr(io_connect_t c, uint32_t off, uint32_t val) {
    uint64_t in[2] = { off, val };
    return IOConnectCallScalarMethod(c, kSel_WriteGPIO32, in, 2, NULL, NULL);
}

static int i2c_write_reg(io_connect_t c, uint8_t reg, uint8_t val) {
    uint64_t in[2] = { reg, val };
    kern_return_t k = IOConnectCallScalarMethod(c, kSel_I2CWriteReg, in, 2, NULL, NULL);
    if (k != KERN_SUCCESS) {
        printf("  [-] I2C[0x%02x]=0x%02x failed 0x%x\n", reg, val, k);
        return -1;
    }
    return 0;
}

static uint8_t i2c_read_reg(io_connect_t c, uint8_t reg) {
    uint64_t in = reg, out = 0;
    uint32_t outCnt = 1;
    if (IOConnectCallScalarMethod(c, kSel_I2CReadReg, &in, 1, &out, &outCnt) != KERN_SUCCESS)
        return 0xff;
    return (uint8_t)out;
}

/* Canonical 19.2 MHz / S16 slave (AUDIO_HUAWEI validated SERDATA 0x06 + HPMIX 0xBB). */
static void apply_codec_v19_slave(io_connect_t c) {
    /* SYS_PDN 0x0d: DAPM bits are inverted. 0x3F = all analog OFF (Linux idle).
     * Linux paplay headphones = 0x06 (DAC Vref+Bias+Analog ON; ADC bits off).
     * This was the silence root cause — playtone never cleared 0x3F. */
    i2c_write_reg(c, 0x0d, 0x06);                 /* ES8316_SYS_PDN */
    i2c_write_reg(c, 0x0e, 0x00);                 /* SYS_LP1 */
    i2c_write_reg(c, 0x0f, 0x00);                 /* SYS_LP2 */
    i2c_write_reg(c, ES8316_CLKMGR_CLKSW, 0x7F); /* keep DAC/CP clocks; Linux uses 0x77 */
    i2c_write_reg(c, ES8316_CLKMGR_CLKSEL, 0x00);
    i2c_write_reg(c, ES8316_CLKMGR_ADCDIV1, 0x01);
    i2c_write_reg(c, ES8316_CLKMGR_ADCDIV2, 0x90);
    i2c_write_reg(c, ES8316_CLKMGR_DACDIV1, 0x01);
    i2c_write_reg(c, ES8316_CLKMGR_DACDIV2, 0x90);
    i2c_write_reg(c, ES8316_SERDATA1, 0x06);      /* bclk_div=6 slave S16 */
    i2c_write_reg(c, ES8316_SERDATA_ADC, 0x0C);   /* I2S 16-bit */
    i2c_write_reg(c, ES8316_SERDATA_DAC, 0x0C);
    i2c_write_reg(c, 0x15, 0x00);                 /* HPMIX_PDN — Linux play 0x00 */
    i2c_write_reg(c, ES8316_HPMIX_SWITCH, 0x88);
    i2c_write_reg(c, ES8316_HPMIX_VOL, 0x00);     /* −12 dB (Linux playing) */
    i2c_write_reg(c, ES8316_CPHP_ICAL_VOL, 0x00);
    i2c_write_reg(c, ES8316_CPHP_OUTEN, 0x66);
    i2c_write_reg(c, ES8316_DAC_PDN, 0x00);
    i2c_write_reg(c, ES8316_DAC_VOLL, 0x17);
    i2c_write_reg(c, ES8316_DAC_VOLR, 0x17);
    i2c_write_reg(c, ES8316_DAC_SET1, 0x10);
}

/*
 * Byte-for-byte replica of the Linux FULL3 "paplay headphones" i2cdump, which
 * is the only register set proven to make this board audible. 24-bit word
 * length (SERDATA_DAC LEN_24 = 0x00) to match tplg slot width 25.
 * Volumes match FULL3 playing-speakers: HPMIX 0x00 (−12 dB) + DAC 0x17
 * (~−11.5 dB). Our old 0xBB/0x00 was ~24 dB hotter and hard-clipped the amps.
 */
/* Always 0 dB HPMIX/DAC — speakers validated loud; HP attenuate was sticky and
 * left speakers almost silent after route flips. GPIO alone selects SPK vs HP. */
static void apply_codec_linux_s24(io_connect_t c) {
    i2c_write_reg(c, 0x0d, 0x06);                 /* SYS_PDN analog on */
    i2c_write_reg(c, 0x0e, 0x00);
    i2c_write_reg(c, 0x0f, 0x00);
    i2c_write_reg(c, ES8316_CLKMGR_CLKSW, 0x77);
    i2c_write_reg(c, ES8316_CLKMGR_CLKSEL, 0x00);
    i2c_write_reg(c, ES8316_CLKMGR_ADCDIV1, 0x11);
    i2c_write_reg(c, ES8316_CLKMGR_ADCDIV2, 0x90);
    i2c_write_reg(c, ES8316_CLKMGR_DACDIV1, 0x11);
    i2c_write_reg(c, ES8316_CLKMGR_DACDIV2, 0x90);
    i2c_write_reg(c, ES8316_SERDATA1, 0x04);
    i2c_write_reg(c, ES8316_SERDATA_ADC, 0x40);
    i2c_write_reg(c, ES8316_SERDATA_DAC, 0x00);
    i2c_write_reg(c, 0x15, 0x00);
    i2c_write_reg(c, ES8316_HPMIX_SWITCH, 0x88);
    i2c_write_reg(c, ES8316_HPMIX_VOL, 0xBB);      /* 0 dB */
    i2c_write_reg(c, ES8316_CPHP_ICAL_VOL, 0x00);
    i2c_write_reg(c, ES8316_CPHP_OUTEN, 0x66);
    i2c_write_reg(c, ES8316_DAC_PDN, 0x00);
    i2c_write_reg(c, ES8316_DAC_VOLL, 0x00);       /* 0 dB */
    i2c_write_reg(c, ES8316_DAC_VOLR, 0x00);
    i2c_write_reg(c, ES8316_DAC_SET1, 0x10);
}

static void apply_linux_play_codec(io_connect_t c) {
    apply_codec_linux_s24(c);
}

static void apply_linux_play_codec_route(io_connect_t c, int speakers) {
    (void)speakers;
    apply_codec_linux_s24(c);
}

static void apply_codec_linux_s24_spk(io_connect_t c) {
    apply_codec_linux_s24(c);
}

static void dump_play_snapshot(io_connect_t c) {
    static const uint8_t regs[] = {
        0x00, 0x01, 0x04, 0x05, 0x06, 0x07, 0x09, 0x0a, 0x0b, 0x0c,
        0x0d, 0x14, 0x15, 0x16, 0x17, 0x18, 0x2f, 0x30, 0x33, 0x34, 0x4f
    };
    uint32_t hp = 0, spk = 0;
    gpio_rd(c, amp_padcfg(c, 41), &hp);
    gpio_rd(c, amp_padcfg(c, 40), &spk);
    printf("==== dump-play ====\n");
    printf("GPIO HP pin41=0x%08x (TX=%u) SPK pin40=0x%08x (TX=%u)  [1=amp ON]\n",
           hp, hp & 1u, spk, spk & 1u);
    printf("Target speakers: both TX=1 | 0x0d=06 0x16=BB 0x30=10\n");
    for (size_t i = 0; i < sizeof(regs); i++) {
        uint8_t v = i2c_read_reg(c, regs[i]);
        printf("I2C[0x%02x] = 0x%02x\n", regs[i], v);
    }
}

static uint32_t bar4_rd32(io_connect_t c, uint32_t off) {
    uint64_t in = off, out = 0;
    uint32_t outCnt = 1;
    if (IOConnectCallScalarMethod(c, kSel_ReadBAR4_32, &in, 1, &out, &outCnt) != KERN_SUCCESS)
        return 0;
    return (uint32_t)out;
}

static uint32_t bar0_rd32(io_connect_t c, uint32_t off) {
    uint64_t in = off, out = 0;
    uint32_t outCnt = 1;
    if (IOConnectCallScalarMethod(c, kSel_ReadBAR0_32, &in, 1, &out, &outCnt) != KERN_SUCCESS)
        return 0;
    return (uint32_t)out;
}

static uint16_t bar0_rd16(io_connect_t c, uint32_t off) {
    uint64_t in = off, out = 0;
    uint32_t outCnt = 1;
    if (IOConnectCallScalarMethod(c, kSel_ReadBAR0_16, &in, 1, &out, &outCnt) != KERN_SUCCESS)
        return 0;
    return (uint16_t)out;
}

static double now_sec(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}

/*
 * HDA/DSP stream probe. Decisive question: does the DSP drain the host stream?
 * 48 kHz S16 stereo must advance LPIB at 192000 bytes/s. Much slower ⇒ the DSP
 * host copier is not pulling (pipeline not really running).
 */
static void probe_dma(io_connect_t c, double window) {
    uint16_t gcap = bar0_rd16(c, 0x00);
    int niss = (gcap >> 8) & 0xF, noss = (gcap >> 12) & 0xF;
    printf("GCAP=0x%04x NISS=%d NOSS=%d INTCTL=0x%08x\n",
           gcap, niss, noss, bar0_rd32(c, 0x20));

    uint32_t ppctl_off = 0, spib_off = 0;
    uint32_t off = bar0_rd32(c, 0x14) & 0xFFFFu;
    for (int i = 0; i < 8 && off; i++) {
        uint32_t v = bar0_rd32(c, off);
        uint32_t cid = (v >> 16) & 0xFFFu;
        printf("  cap@0x%04x id=%u\n", off, cid);
        if (cid == 3) ppctl_off = off + 4;
        if (cid == 4) spib_off = off;
        off = v & 0xFFFFu;
    }
    if (ppctl_off)
        printf("PPCTL@0x%04x = 0x%08x (GPROCEN=%u PIE=%u)\n", ppctl_off,
               bar0_rd32(c, ppctl_off), (bar0_rd32(c, ppctl_off) >> 30) & 1u,
               (bar0_rd32(c, ppctl_off) >> 31) & 1u);
    printf("DPLBASE=0x%08x DPUBASE=0x%08x%s\n", bar0_rd32(c, 0x70), bar0_rd32(c, 0x74),
           ((bar0_rd32(c, 0x70) & ~1u) == 0 && (bar0_rd32(c, 0x70) & 1u))
               ? "  <-- WARNING: position buffer at phys 0" : "");
    if (spib_off) printf("SPIB cap@0x%04x ctl=0x%08x\n", spib_off, bar0_rd32(c, spib_off + 4));

    int nsd = niss + noss;
    if (nsd <= 0 || nsd > 16) nsd = 16;
    int running[16], nrun = 0;
    for (int i = 0; i < nsd; i++) {
        uint32_t base = 0x80u + (uint32_t)i * 0x20u;
        uint32_t ctl = bar0_rd32(c, base);
        int run = (int)((ctl >> 1) & 1u);
        printf("  SD%-2d base=0x%03x CTL=0x%06x STS=0x%02x RUN=%d STRM=%u "
               "LPIB=0x%08x CBL=0x%08x LVI=%u FMT=0x%04x\n",
               i, base, ctl & 0xFFFFFFu, (ctl >> 24) & 0xFFu, run,
               (ctl >> 20) & 0xFu, bar0_rd32(c, base + 4), bar0_rd32(c, base + 8),
               bar0_rd16(c, base + 0x0C), bar0_rd16(c, base + 0x12));
        if (run && nrun < 16) running[nrun++] = i;
    }
    if (!nrun) { printf("[-] no stream has RUN=1 — start playback first\n"); return; }

    for (int k = 0; k < nrun; k++) {
        uint32_t base = 0x80u + (uint32_t)running[k] * 0x20u;
        uint32_t cbl = bar0_rd32(c, base + 8);
        if (!cbl) cbl = 0x10000u;
        /* Window must stay under one CBL wrap or the delta is ambiguous:
         * at 192000 B/s a 64 KB ring wraps every 341 ms. */
        double w = window;
        double maxw = (double)cbl / 192000.0 * 0.7;
        if (w > maxw) w = maxw;
        uint32_t a = bar0_rd32(c, base + 4);
        double t0 = now_sec();
        usleep((useconds_t)(w * 1e6));
        uint32_t b = bar0_rd32(c, base + 4);
        double dt = now_sec() - t0;
        uint32_t delta = (b >= a) ? (b - a) : (cbl - a + b);
        double rate = (double)delta / dt;
        printf("SD%-2d LPIB %u -> %u  delta=%u in %.3fs = %.0f B/s "
               "(%.1f%% of 192000 expected) %s\n",
               running[k], a, b, delta, dt, rate, 100.0 * rate / 192000.0,
               (rate > 172000.0 && rate < 212000.0) ? "OK (DSP drains at 48 kHz)"
                                                    : "<-- ANOMALY");
    }

    /* SOF SW-reg window: FW publishes its own stream positions here. All-zero
     * while the host DMA runs ⇒ the DSP side never started. */
    printf("SRAM win0 (SW regs @0x80000):");
    for (uint32_t i = 0; i < 16; i++) {
        if ((i % 8) == 0) printf("\n  +0x%03x:", i * 4);
        printf(" %08x", bar4_rd32(c, 0x80000u + i * 4));
    }
    printf("\n");
}

static void bar4_wr32(io_connect_t c, uint32_t off, uint32_t val) {
    uint64_t in[2] = { off, val };
    IOConnectCallScalarMethod(c, kSel_WriteBAR4_32, in, 2, NULL, NULL);
}

static void drain_hipct(io_connect_t c) {
    for (int i = 0; i < 64; i++) {
        uint32_t hipct = bar4_rd32(c, IPC_HIPCTDR);
        if (!(hipct & IPC_BUSY_DONE)) break;
        bar4_wr32(c, IPC_HIPCTDA, IPC_BUSY_DONE);
        bar4_wr32(c, IPC_HIPCTDR, hipct);
    }
}

static int send_ipc(io_connect_t c, const void *msg, size_t len, uint32_t *replyErr) {
    drain_hipct(c);
    for (int t = 0; t < 500; t++) {
        if (!(bar4_rd32(c, IPC_HIPCIDR) & IPC_BUSY_DONE)) break;
        drain_hipct(c);
        usleep(1000);
    }
    const uint8_t *p = (const uint8_t *)msg;
    for (size_t i = 0; i < len; i += 4) {
        uint32_t w = 0;
        memcpy(&w, p + i, (len - i >= 4) ? 4 : (len - i));
        bar4_wr32(c, OUTBOX + (uint32_t)i, w);
    }
    bar4_wr32(c, IPC_HIPCIDA, bar4_rd32(c, IPC_HIPCIDA) | IPC_BUSY_DONE);
    bar4_wr32(c, IPC_HIPCCTL, bar4_rd32(c, IPC_HIPCCTL) | 0x3u);
    bar4_wr32(c, IPC_HIPCIDR, IPC_BUSY_DONE);
    for (int t = 0; t < 1000; t++) {
        if (bar4_rd32(c, IPC_HIPCIDA) & IPC_BUSY_DONE) {
            uint32_t err = bar4_rd32(c, OUTBOX + 8);
            bar4_wr32(c, IPC_HIPCIDA, bar4_rd32(c, IPC_HIPCIDA) | IPC_BUSY_DONE);
            drain_hipct(c);
            if (replyErr) *replyErr = err;
            return 0;
        }
        drain_hipct(c);
        usleep(500);
    }
    if (replyErr) *replyErr = 0xffffffffu;
    return -1;
}

/* sof_ipc_dai_config: hdr 8 | type 8 | dai_index 12 | format 16 (u16) |
 * group_id 18 | flags 19 | reserved[8] 20..51 | union ssp @52.
 * sof_ipc_dai_ssp_params: hdr.size 0 | reserved1 4 | mclk_id 6 | mclk_rate 8 |
 * fsync_rate 12 | bclk_rate 16 | tdm_slots 20 | rx_slots 24 | tx_slots 28 |
 * sample_valid_bits 32 | tdm_slot_width 36 | mclk_direction 40 |
 * frame_pulse_width 44 | tdm_per_slot_padding 46 | clks_control 48 |
 * quirks 52 | bclk_delay 56  => sizeof 60.
 * fmt: 0x4001 = I2S|CBC_CFC (codec slave), 0x0001 = I2S|CBP_CFP (codec master).
 * mclk_direction 1 = MCLK is an input to the codec, i.e. SSP drives it
 * (tplg SND_SOC_TPLG_MCLK_CI). */
static void ssp0_fill_dai_msg_ex(uint8_t m[216], uint8_t flags, uint16_t fmt,
                                 uint32_t mclk, uint32_t bclk, uint32_t bits, uint16_t slot_w) {
    memset(m, 0, 216);
    *(uint32_t *)(m + 0) = 216;
    *(uint32_t *)(m + 4) = 0x80010000u; /* DAI_CONFIG */
    *(uint32_t *)(m + 8) = DAI_SSP;
    *(uint32_t *)(m + 12) = 0; /* ssp0 */
    *(uint16_t *)(m + 16) = fmt;
    m[18] = 0;
    m[19] = flags;
    uint8_t *ssp = m + 52;
    *(uint32_t *)(ssp + 0) = 60;  /* sizeof(sof_ipc_dai_ssp_params) */
    *(uint32_t *)(ssp + 8) = mclk;
    *(uint32_t *)(ssp + 12) = 48000u;
    *(uint32_t *)(ssp + 16) = bclk;
    *(uint32_t *)(ssp + 20) = 2;
    *(uint32_t *)(ssp + 24) = 3;
    *(uint32_t *)(ssp + 28) = 3;
    *(uint32_t *)(ssp + 32) = bits;
    *(uint16_t *)(ssp + 36) = slot_w;
    *(uint32_t *)(ssp + 40) = (fmt == 0x0001) ? 0u : 1u;
    *(uint32_t *)(ssp + 48) = SOF_SSP_CLKS_CONTROL;
}

/* Defaults = sof-cml-es8336-dmic2ch-ssp0.tplg hw_config (the config Linux
 * actually runs on this board): MCLK 24 MHz, BCLK 2.4 MHz, 2 slots of 25 bits,
 * 24 valid bits. We had been sending 19.2 MHz / 1.536 MHz / 16 bits. */
#define SSP_GOLD_MCLK   24000000u
#define SSP_GOLD_BCLK   2400000u
#define SSP_GOLD_BITS   24u
#define SSP_GOLD_SLOTW  25u

static void ssp0_fill_dai_msg(uint8_t m[216], uint8_t flags, uint16_t fmt) {
    ssp0_fill_dai_msg_ex(m, flags, fmt, SSP_GOLD_MCLK, SSP_GOLD_BCLK,
                         SSP_GOLD_BITS, SSP_GOLD_SLOTW);
}

static int ssp0_dai_config_flags_ex(io_connect_t c, uint8_t flags, uint16_t fmt,
                                    uint32_t mclk, uint32_t bclk, uint32_t bits, uint16_t slot_w,
                                    const char *tag) {
    uint8_t m[216];
    ssp0_fill_dai_msg_ex(m, flags, fmt, mclk, bclk, bits, slot_w);
    uint32_t err = 0;
    if (send_ipc(c, m, sizeof(m), &err) != 0 || err != 0) {
        printf("[-] SSP0 DAI_CONFIG %s failed replyErr=0x%x\n", tag, err);
        return -1;
    }
    printf("[+] SSP0 DAI_CONFIG %s ok (fmt=0x%04x mclk=%u bclk=%u bits=%u)\n",
           tag, fmt, mclk, bclk, bits);
    return 0;
}

static int ssp0_dai_config_flags(io_connect_t c, uint8_t flags, uint16_t fmt, const char *tag) {
    return ssp0_dai_config_flags_ex(c, flags, fmt, SSP_GOLD_MCLK, SSP_GOLD_BCLK,
                                    SSP_GOLD_BITS, SSP_GOLD_SLOTW, tag);
}

static int ipc_trig(io_connect_t c, uint32_t cmd, uint32_t comp_id, const char *tag) {
    uint32_t m[3] = { 12, cmd, comp_id };
    uint32_t err = 0;
    if (send_ipc(c, m, sizeof(m), &err) != 0 || err != 0) {
        printf("[-] %s comp=%u replyErr=0x%x\n", tag, comp_id, err);
        return -1;
    }
    printf("[+] %s comp=%u ok\n", tag, comp_id);
    return 0;
}

static int ssp0_dai_config_hw_params(io_connect_t c) {
    return ssp0_dai_config_flags(c, 0x01, 0x4001, "HW_PARAMS");
}

static int ssp0_dai_config_hw_free(io_connect_t c) {
    return ssp0_dai_config_flags(c, 0x02, 0x4001, "HW_FREE");
}

/* Community-0 PADBAR (usually 0xA50, read from the community window). */
static uint32_t c0_padbar(io_connect_t c) {
    uint32_t v = 0;
    if (gpio_rd(c, GPIO_COMM0 | 0x0Cu, &v) == KERN_SUCCESS) {
        v &= 0xFFFF;
        if (v >= 0x100u && v < 0x8000u) return v;
    }
    return 0x0A50u;
}

static uint32_t amp_padcfg(io_connect_t c, uint32_t pin) {
    return GPIO_COMM0 | (c0_padbar(c) + pin * 16u);
}

/* Drive GPP_B15/B16 TXSTATE. `padcfg_off` is either the encoded token
 * GPIO_SPK/HP_PADCFG or an already-resolved MMIO offset. */
static kern_return_t gpio_set_level(io_connect_t c, uint32_t padcfg_off, int high) {
    uint32_t local = padcfg_off & 0xFFFFu;
    uint32_t pin;
    if (local == 0x0CD0u || local == 40u) pin = 40;
    else if (local == 0x0CE0u || local == 41u) pin = 41;
    else {
        uint32_t spk = amp_padcfg(c, 40) & 0xFFFFu;
        uint32_t hp  = amp_padcfg(c, 41) & 0xFFFFu;
        if (local == spk) pin = 40;
        else if (local == hp) pin = 41;
        else return KERN_INVALID_ARGUMENT;
    }
    uint32_t off = amp_padcfg(c, pin);

    uint32_t old = 0;
    kern_return_t kr = gpio_rd(c, off, &old);
    if (kr != KERN_SUCCESS) return kr;
    uint32_t val = (old & ~1u) | (high ? 1u : 0u);
    kr = gpio_wr(c, off, val);
    uint32_t verify = 0;
    gpio_rd(c, off, &verify);
    printf("  C0 pin=%u @0x%x 0x%08x -> wr 0x%08x rd 0x%08x (TX=%d TXDIS=%u) kr=0x%x\n",
           pin, off & 0xFFFF, old, val, verify, high ? 1 : 0, (verify >> 8) & 1u, kr);
    return kr;
}

/*
 * Map capture shared memory if not already mapped for this connection.
 * Returns 0 on success; leaves addr and size set. Caller must unmap.
 */
static int map_capture_if_needed(io_connect_t connect, mach_vm_address_t *addr,
                                 mach_vm_size_t *size) {
    if (*addr != 0 && *size >= CAP_BUFFER_SIZE) return 0;
    mach_vm_address_t a = 0;
    mach_vm_size_t s = 0;
    kern_return_t kr = IOConnectMapMemory64(connect, kMemCapture, mach_task_self(),
                                            &a, &s, kIOMapAnywhere);
    if (kr != KERN_SUCCESS || a == 0) {
        printf("[-] IOConnectMapMemory(type=1 capture) failed: 0x%x\n", kr);
        return -1;
    }
    *addr = a;
    *size = s;
    printf("[+] capture DMA mapped addr=0x%llx size=%llu\n",
           (unsigned long long)a, (unsigned long long)s);
    return 0;
}

static void usage(const char *argv0) {
    fprintf(stderr,
        "Usage: %s <command> [args]\n"
        "  status                         Show IORegistry / connection status\n"
        "  init [force]                   TriggerInitDSP embedded; force => reset then init\n"
        "  resetdsp                       ForceResetDSP (power-gate + clear fail props)\n"
        "  loadfw <path.ri> [--boot]      Stage RI to mem type 3 + SetFwStagingLen;\n"
        "                                 --boot also calls InitDSPFromStaging\n"
        "  init-staging [len]             InitDSPFromStaging (selector 28)\n"
        "  fwphys                         Print FW staging physical address\n"
        "  phys                           Print play/cap/flags/fw phys addrs\n"
        "  mapcap                         Map capture DMA (IOConnectMapMemory type 1)\n"
        "  mute-speakers                  GPIO HP+SPK LOW + DAC soft-mute (night default)\n"
        "  unmute-speakers                GPP_B15+B16 TX=1 (amps ON), DAC unmute\n"
        "  unmute-headphones              GPP_B15+B16 TX=0 (amps OFF), DAC unmute\n"
        "  wait-fsr [timeout_ms]          Poll BAR4 ROM_STATUS until FSR=5 (default 15000)\n"
        "  es8336-config                  One-shot I2C configure; amps+DAC stay muted\n"
        "  rdbar0 <8|16|32> <off>         Read BAR0\n"
        "  wrbar0 <8|16|32> <off> <val>   Write BAR0\n"
        "  rdbar4 <8|16|32|64> <off>      Read BAR4 (64 -> lo hi)\n"
        "  wrbar4 <8|16|32|64> <off> <val> [hi]\n"
        "  rdgpio <off>                   Read GPIO Community0 dword\n"
        "  wrgpio <off> <val>             Write GPIO Community0 dword\n"
        "  rdi2c <reg>                    Read ES8336 register\n"
        "  wri2c <reg> <val>              Write ES8336 register\n"
        "  start|stop|pos                 Playback helpers\n"
        "  ssp0-dai-config                SSP0 DAI_CONFIG HW_PARAMS (start clocks)\n"
        "  ssp0-dai-free                  SSP0 DAI_CONFIG HW_FREE (stop clocks)\n"
        "  ssp0-reclock                   HW_FREE then HW_PARAMS (force clock restart)\n"
        "  codec-master-tone [freq] [sec] Codec I2S master 0x86 + SSP CBP_CFP + tone\n"
        "  bclk-inv-tone [freq] [sec]     SERDATA1 BCLK_INV + slave SSP + tone\n"
        "  playtone [freq] [sec] [spk|hp] Sine + SSP0 clocks + amp route\n"
        "                                 optional spk|hp forces route (ignore jack)\n"
        "  alltone [sec]                  Brute-force clock/codec variants (LISTEN)\n"
        "  dump-play                      I2C key regs + GPIO pads (vs Linux gold)\n"
        "  calibrate [spk|hp]             Diff live regs vs Linux FULL3 targets\n"
        "  probe-dma [sec]                HDA streams + LPIB rate vs 192000 B/s\n"
        "  hot-csm                        Reset CSM while Playing (needs clocks)\n"
        "  capstart|capstop|cappos        Capture helpers (maps DMA if missing)\n",
        argv0);
}

static int parse_u64(const char *s, uint64_t *out) {
    char *end = NULL;
    errno = 0;
    unsigned long long v = strtoull(s, &end, 0);
    if (errno || !end || *end != '\0') return -1;
    *out = (uint64_t)v;
    return 0;
}

static int open_client(io_service_t *outSvc, io_connect_t *outConn) {
    CFMutableDictionaryRef matching = IOServiceMatching("MateBookAudioDevice");
    io_service_t service = IOServiceGetMatchingService(kIOMainPortDefault, matching);
    if (!service) return -1;
    io_connect_t connect = 0;
    kern_return_t kr = IOServiceOpen(service, mach_task_self(), 0, &connect);
    if (kr != KERN_SUCCESS) {
        IOObjectRelease(service);
        return -2;
    }
    *outSvc = service;
    *outConn = connect;
    return 0;
}

static int width_to_sel_rd(int bar, int width) {
    if (bar == 0) {
        if (width == 8) return kSel_ReadBAR0_8;
        if (width == 16) return kSel_ReadBAR0_16;
        if (width == 32) return kSel_ReadBAR0_32;
    } else {
        if (width == 8) return kSel_ReadBAR4_8;
        if (width == 16) return kSel_ReadBAR4_16;
        if (width == 32) return kSel_ReadBAR4_32;
        if (width == 64) return kSel_ReadBAR4_64;
    }
    return -1;
}

static int width_to_sel_wr(int bar, int width) {
    if (bar == 0) {
        if (width == 8) return kSel_WriteBAR0_8;
        if (width == 16) return kSel_WriteBAR0_16;
        if (width == 32) return kSel_WriteBAR0_32;
    } else {
        if (width == 8) return kSel_WriteBAR4_8;
        if (width == 16) return kSel_WriteBAR4_16;
        if (width == 32) return kSel_WriteBAR4_32;
        if (width == 64) return kSel_WriteBAR4_64;
    }
    return -1;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "status") == 0) {
        dump_hdas_props();
        io_service_t service = 0;
        io_connect_t connect = 0;
        int oc = open_client(&service, &connect);
        if (oc == -1) {
            printf("[-] MateBookAudioDevice NOT in IORegistry\n");
            return 1;
        }
        if (oc == -2) {
            printf("[+] MateBookAudioDevice present but UserClient open failed\n");
            dump_device_props(service);
            IOObjectRelease(service);
            return 1;
        }
        printf("[+] MateBookAudioDevice + UserClient open (svc=0x%x conn=0x%x)\n",
               service, connect);
        dump_device_props(service);
        IOServiceClose(connect);
        IOObjectRelease(service);
        return 0;
    }

    io_service_t service = 0;
    io_connect_t connect = 0;
    int oc = open_client(&service, &connect);
    if (oc != 0) {
        printf("[-] Cannot open MateBookAudioDevice UserClient (code %d)\n", oc);
        dump_hdas_props();
        return 1;
    }

    kern_return_t kr = KERN_SUCCESS;
    int rc = 0;
    mach_vm_address_t capMapAddr = 0;
    mach_vm_size_t capMapSize = 0;

    if (strcmp(cmd, "init") == 0) {
        int doForce = 0;
        if (argc >= 3) {
            uint64_t tmp = 0;
            if (parse_u64(argv[2], &tmp) == 0 && tmp) doForce = 1;
            else if (strcmp(argv[2], "force") == 0) doForce = 1;
        }
        if (doForce) {
            printf("[*] ForceResetDSP before init...\n");
            kr = IOConnectCallScalarMethod(connect, kSel_ForceResetDSP, NULL, 0, NULL, NULL);
            printf("[%c] resetdsp -> 0x%x\n", kr == KERN_SUCCESS ? '+' : '-', kr);
        }
        printf("[*] TriggerInitDSP...\n");
        kr = IOConnectCallScalarMethod(connect, kSel_TriggerInitDSP, NULL, 0, NULL, NULL);
        printf("[%c] initDSP -> 0x%x\n", kr == KERN_SUCCESS ? '+' : '-', kr);
        dump_device_props(service);
        dump_hdas_props();
        rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "resetdsp") == 0) {
        printf("[*] ForceResetDSP...\n");
        kr = IOConnectCallScalarMethod(connect, kSel_ForceResetDSP, NULL, 0, NULL, NULL);
        printf("[%c] resetdsp -> 0x%x\n", kr == KERN_SUCCESS ? '+' : '-', kr);
        dump_device_props(service);
        dump_hdas_props();
        rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "init-staging") == 0) {
        uint64_t stagedLen = 0;
        uint32_t inCnt = 0;
        if (argc >= 3) {
            if (parse_u64(argv[2], &stagedLen) != 0) {
                fprintf(stderr, "bad len\n"); rc = 1; goto done;
            }
            inCnt = 1;
        }
        printf("[*] InitDSPFromStaging(len=%llu)...\n", (unsigned long long)stagedLen);
        kr = IOConnectCallScalarMethod(connect, kSel_InitDSPFromStaging,
                                       inCnt ? &stagedLen : NULL, inCnt, NULL, NULL);
        printf("[%c] init-staging -> 0x%x\n", kr == KERN_SUCCESS ? '+' : '-', kr);
        dump_device_props(service);
        dump_hdas_props();
        rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "fwphys") == 0) {
        uint64_t out = 0;
        uint32_t outCnt = 1;
        kr = IOConnectCallScalarMethod(connect, kSel_GetFwPhys, NULL, 0, &out, &outCnt);
        if (kr == KERN_SUCCESS)
            printf("fw_staging_phys = 0x%llx\n", (unsigned long long)out);
        else {
            printf("[-] fwphys failed: 0x%x (needs rebuilt kext)\n", kr);
            rc = 1;
        }
    } else if (strcmp(cmd, "loadfw") == 0) {
        if (argc < 3) { usage(argv[0]); rc = 1; goto done; }
        const char *path = argv[2];
        int doBoot = 0;
        for (int ai = 3; ai < argc; ai++) {
            if (strcmp(argv[ai], "--boot") == 0) doBoot = 1;
        }
        int fd = open(path, O_RDONLY);
        if (fd < 0) {
            printf("[-] open(%s): %s\n", path, strerror(errno));
            rc = 1; goto done;
        }
        struct stat st;
        if (fstat(fd, &st) != 0 || st.st_size <= 0) {
            printf("[-] bad firmware file size\n");
            close(fd); rc = 1; goto done;
        }
        if ((size_t)st.st_size > FW_STAGING_SIZE) {
            printf("[-] firmware %lld bytes exceeds staging %u\n",
                   (long long)st.st_size, FW_STAGING_SIZE);
            close(fd); rc = 1; goto done;
        }
        void *fileMap = mmap(NULL, (size_t)st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
        close(fd);
        if (fileMap == MAP_FAILED) {
            printf("[-] mmap firmware: %s\n", strerror(errno));
            rc = 1; goto done;
        }

        mach_vm_address_t fwAddr = 0;
        mach_vm_size_t fwSize = 0;
        kr = IOConnectMapMemory64(connect, kMemFirmware, mach_task_self(),
                                  &fwAddr, &fwSize, kIOMapAnywhere);
        if (kr != KERN_SUCCESS || fwAddr == 0) {
            printf("[-] IOConnectMapMemory(type=3 firmware) failed: 0x%x\n", kr);
            munmap(fileMap, (size_t)st.st_size);
            rc = 1; goto done;
        }
        if (fwSize < (mach_vm_size_t)st.st_size) {
            printf("[-] staging map too small (%llu < %lld)\n",
                   (unsigned long long)fwSize, (long long)st.st_size);
            IOConnectUnmapMemory64(connect, kMemFirmware, mach_task_self(), fwAddr);
            munmap(fileMap, (size_t)st.st_size);
            rc = 1; goto done;
        }

        memset((void *)(uintptr_t)fwAddr, 0, (size_t)fwSize);
        memcpy((void *)(uintptr_t)fwAddr, fileMap, (size_t)st.st_size);
        munmap(fileMap, (size_t)st.st_size);

        uint32_t sig = *(uint32_t *)(uintptr_t)fwAddr;
        printf("[+] staged %lld bytes into mem type 3 (sig=0x%08x map=%llu)\n",
               (long long)st.st_size, sig, (unsigned long long)fwSize);

        uint64_t stagedLen = (uint64_t)st.st_size;
        kr = IOConnectCallScalarMethod(connect, kSel_SetFwStagingLen,
                                       &stagedLen, 1, NULL, NULL);
        if (kr != KERN_SUCCESS) {
            /* Fallback for older kext without selector 30: boot path sets len via 28 */
            printf("[!] SetFwStagingLen failed 0x%x — falling back to InitDSPFromStaging\n", kr);
            doBoot = 1;
        } else {
            printf("[+] fwStagingLen=%llu\n", (unsigned long long)stagedLen);
        }

        if (doBoot) {
            kr = IOConnectCallScalarMethod(connect, kSel_InitDSPFromStaging,
                                           &stagedLen, 1, NULL, NULL);
            printf("[%c] InitDSPFromStaging(len=%llu) -> 0x%x\n",
                   kr == KERN_SUCCESS ? '+' : '-', (unsigned long long)stagedLen, kr);
            dump_device_props(service);
            dump_hdas_props();
        } else {
            printf("[*] staged only — run: %s init-staging\n", argv[0]);
        }
        IOConnectUnmapMemory64(connect, kMemFirmware, mach_task_self(), fwAddr);
        rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "phys") == 0) {
        uint64_t out[5] = {0};
        uint32_t outCnt = 5;
        kr = IOConnectCallScalarMethod(connect, kSel_GetPhysicalAddrs, NULL, 0, out, &outCnt);
        if (kr == KERN_SUCCESS) {
            printf("playback_phys = 0x%llx\n", (unsigned long long)out[0]);
            printf("capture_phys  = 0x%llx\n", (unsigned long long)out[1]);
            printf("flags_phys    = 0x%llx\n", (unsigned long long)out[2]);
            printf("fw_phys       = 0x%llx\n", (unsigned long long)out[3]);
        } else {
            printf("[-] phys failed: 0x%x\n", kr);
            rc = 1;
        }
    } else if (strcmp(cmd, "mapcap") == 0) {
        rc = (map_capture_if_needed(connect, &capMapAddr, &capMapSize) == 0) ? 0 : 1;
        if (rc == 0) {
            /* Keep mapping briefly so callers can observe; then unmap before exit. */
            usleep(10000);
        }
    } else if (strcmp(cmd, "wait-fsr") == 0) {
        /* Poll BAR4 ROM_STATUS until FSR_FW_ENTERED; exit 0 only on success. */
        int timeout_ms = 15000;
        if (argc >= 3) {
            uint64_t t = 0;
            if (parse_u64(argv[2], &t) != 0 || t < 100 || t > 120000) {
                fprintf(stderr, "bad timeout_ms (100..120000)\n");
                rc = 1;
                goto done;
            }
            timeout_ms = (int)t;
        }
        printf("[*] wait-fsr timeout=%d ms (expect FSR=%u, CapChannels=%d)\n",
               timeout_ms, FSR_FW_ENTERED, CAP_CHANNELS);
        int entered = 0, cse_halt = 0, waited = 0;
        uint32_t rom = 0, ex = 0;
        for (waited = 0; waited <= timeout_ms; waited += 100) {
            uint64_t in = ROM_STATUS, out = 0;
            uint32_t n = 1;
            kr = IOConnectCallScalarMethod(connect, kSel_ReadBAR4_32, &in, 1, &out, &n);
            if (kr != KERN_SUCCESS) {
                fprintf(stderr, "[-] rdbar4 ROM_STATUS failed: 0x%x\n", kr);
                rc = 1;
                goto done;
            }
            rom = (uint32_t)out;
            in = ROM_STATUS_EX; out = 0; n = 1;
            IOConnectCallScalarMethod(connect, kSel_ReadBAR4_32, &in, 1, &out, &n);
            ex = (uint32_t)out;
            uint32_t state = rom & 0xFFFFFFu;
            if (state == FSR_FW_ENTERED) {
                entered = 1;
                printf("[+] wait-fsr: FSR_FW_ENTERED after %d ms\n", waited);
                break;
            }
            if (waited == 0 || (waited % 1000) == 0) {
                printf("[*] wait-fsr %d/%d ms ROM=0x%08x state=%u EX=0x%x\n",
                       waited, timeout_ms, rom, state, ex);
            }
            if ((rom & 0x80000000U) && (state == 0x12u)) {
                cse_halt = 1;
                fprintf(stderr,
                        "[-] CSE halt ROM=0x%08x EX=0x%x — community FW; "
                        "need EFI intel-signed kext (post-reboot)\n", rom, ex);
                break;
            }
            usleep(100000);
        }
        printf("ROM_STATUS=0x%08x\nROM_STATUS_EX=0x%08x\nFSR_STATE=%u\n"
               "FSR_FW_ENTERED=%d\nFSR_CSE_HALT=%d\nFSR_WAIT_MS=%d\nCAP_CHANNELS=%d\n",
               rom, ex, rom & 0xFFFFFF, entered ? 1 : 0, cse_halt, waited, CAP_CHANNELS);
        if (!entered) {
            fprintf(stderr,
                    "[-] wait-fsr failed — run: matebook_ctl resetdsp && matebook_ctl init\n"
                    "    (or sof_boot / loadfw --boot) after reboot with intel-signed FW\n");
        }
        rc = entered ? 0 : 1;
    } else if (strcmp(cmd, "calibrate") == 0) {
        /* Diff live codec+GPIO against Linux FULL3 playing targets. */
        int want_spk = 1;
        if (argc >= 3) {
            if (strcmp(argv[2], "hp") == 0 || strcmp(argv[2], "headphones") == 0)
                want_spk = 0;
            else if (strcmp(argv[2], "spk") == 0 || strcmp(argv[2], "speakers") == 0)
                want_spk = 1;
        }
        struct { uint8_t reg; uint8_t spk; uint8_t hp; const char *name; } gold[] = {
            { 0x01, 0x77, 0x77, "CLKSW" },
            { 0x06, 0x11, 0x11, "DACDIV1" },
            { 0x07, 0x90, 0x90, "DACDIV2" },
            { 0x09, 0x04, 0x04, "SERDATA1" },
            { 0x0a, 0x40, 0x40, "SERDATA_ADC" },
            { 0x0b, 0x00, 0x00, "SERDATA_DAC" },
            { 0x0d, 0x06, 0x06, "SYS_PDN" },
            { 0x14, 0x88, 0x88, "HPMIX_SWITCH" },
            { 0x16, 0xbb, 0xbb, "HPMIX_VOL" }, /* 0 dB both — HP attenuate poisoned SPK */
            { 0x17, 0x66, 0x66, "CPHP_OUTEN" },
            { 0x18, 0x00, 0x00, "CPHP_ICAL" },
            { 0x2f, 0x00, 0x00, "DAC_PDN" },
            { 0x30, 0x10, 0x10, "DAC_SET1" },
            { 0x33, 0x00, 0x00, "DAC_VOLL" },
            { 0x34, 0x00, 0x00, "DAC_VOLR" },
        };
        uint32_t hp = 0, spk = 0;
        gpio_rd(connect, amp_padcfg(connect, 41), &hp);
        gpio_rd(connect, amp_padcfg(connect, 40), &spk);
        printf("==== calibrate %s (Linux FULL3 + macOS SPK loudness) ====\n",
               want_spk ? "SPEAKERS" : "HEADPHONES");
        printf("GPIO pin40 TX=%u pin41 TX=%u  expect %s\n",
               spk & 1u, hp & 1u, want_spk ? "both 1" : "both 0");
        int mismatches = 0;
        if (want_spk) {
            if ((spk & 1u) != 1 || (hp & 1u) != 1) { printf("  GPIO MISMATCH\n"); mismatches++; }
        } else {
            if ((spk & 1u) != 0 || (hp & 1u) != 0) { printf("  GPIO MISMATCH\n"); mismatches++; }
        }
        for (size_t i = 0; i < sizeof(gold)/sizeof(gold[0]); i++) {
            uint8_t live = i2c_read_reg(connect, gold[i].reg);
            uint8_t exp = want_spk ? gold[i].spk : gold[i].hp;
            int ok = (live == exp);
            if (!ok) mismatches++;
            printf("  %s I2C[0x%02x] live=0x%02x expect=0x%02x %s\n",
                   gold[i].name, gold[i].reg, live, exp, ok ? "OK" : "DIFF");
        }
        printf("CALIBRATE_%s mismatches=%d\n", want_spk ? "SPK" : "HP", mismatches);
        rc = mismatches ? 1 : 0;
    } else if (strcmp(cmd, "mute-speakers") == 0) {
        /* sof_es8336 active-low: physical HIGH = deassert/OFF. Then DAC soft-mute. */
        printf("[*] mute-speakers: C0 GPP_B15+B16 TX=0 (amps OFF), DAC soft-mute\n");
        kern_return_t k1 = gpio_set_level(connect, GPIO_SPK_PADCFG, 0);
        kern_return_t k2 = gpio_set_level(connect, GPIO_HP_PADCFG, 0);
        int dac = i2c_write_reg(connect, ES8316_DAC_SET1, 0x20);
        uint32_t hp = 0, spk = 0;
        gpio_rd(connect, amp_padcfg(connect, 41), &hp);
        gpio_rd(connect, amp_padcfg(connect, 40), &spk);
        /* Locked pads stay 0x44000300 — treat write success + DAC as OK. */
        rc = (k1 == KERN_SUCCESS && k2 == KERN_SUCCESS && dac == 0) ? 0 : 1;
        printf("MUTE_OK=%d CAP_CHANNELS=%d GPIO_HP=0x%08x GPIO_SPK=0x%08x DAC_MUTE=1\n",
               rc == 0 ? 1 : 0, CAP_CHANNELS, hp, spk);
        printf("[%c] mute-speakers\n", rc == 0 ? '+' : '-');
    } else if (strcmp(cmd, "unmute-speakers") == 0) {
        printf("[*] unmute-speakers: C0 GPP_B15+B16 TX=1 (amps ON), Linux play codec\n");
        kern_return_t k1 = gpio_set_level(connect, GPIO_SPK_PADCFG, 1);
        kern_return_t k2 = gpio_set_level(connect, GPIO_HP_PADCFG, 1);
        apply_linux_play_codec(connect);
        usleep(70000); /* 70ms anti-pop like sof_es8336 pcm_pop_work */
        gpio_set_level(connect, GPIO_SPK_PADCFG, 1);
        gpio_set_level(connect, GPIO_HP_PADCFG, 1);
        uint32_t hp = 0, spk = 0;
        gpio_rd(connect, amp_padcfg(connect, 41), &hp);
        gpio_rd(connect, amp_padcfg(connect, 40), &spk);
        int dac = i2c_read_reg(connect, ES8316_DAC_SET1);
        rc = (k1 == KERN_SUCCESS && k2 == KERN_SUCCESS && dac == 0x10) ? 0 : 1;
        printf("UNMUTE_OK=%d GPIO_HP=0x%08x GPIO_SPK=0x%08x DAC_SET1=0x%02x (pads may be TX-locked)\n",
               rc == 0 ? 1 : 0, hp, spk, dac);
        printf("[%c] unmute-speakers\n", rc == 0 ? '+' : '-');
    } else if (strcmp(cmd, "unmute-headphones") == 0) {
        printf("[*] unmute-headphones: C0 GPP_B15+B16 TX=0, same 0 dB codec as speakers\n");
        kern_return_t k1 = gpio_set_level(connect, GPIO_HP_PADCFG, 0);
        kern_return_t k2 = gpio_set_level(connect, GPIO_SPK_PADCFG, 0);
        apply_linux_play_codec(connect);
        usleep(70000);
        gpio_set_level(connect, GPIO_HP_PADCFG, 0);
        gpio_set_level(connect, GPIO_SPK_PADCFG, 0);
        uint32_t hp = 0, spk = 0;
        gpio_rd(connect, amp_padcfg(connect, 41), &hp);
        gpio_rd(connect, amp_padcfg(connect, 40), &spk);
        int dac = i2c_read_reg(connect, ES8316_DAC_SET1);
        rc = (k1 == KERN_SUCCESS && k2 == KERN_SUCCESS && dac == 0x10) ? 0 : 1;
        printf("UNMUTE_OK=%d GPIO_HP=0x%08x GPIO_SPK=0x%08x DAC_SET1=0x%02x (pads may be TX-locked)\n",
               rc == 0 ? 1 : 0, hp, spk, dac);
        printf("[%c] unmute-headphones\n", rc == 0 ? '+' : '-');
    } else if (strcmp(cmd, "es8336-config") == 0) {
        /* One-shot I2C sequence mirroring ES8336Codec::configure; ends muted */
        printf("[*] es8336-config: one-shot I2C (Linux-aligned clocks/SERDATA; DAC soft-mute)\n");
        int fails = 0;
        fails += i2c_write_reg(connect, ES8316_RESET, 0x3F) < 0;
        usleep(10000);
        fails += i2c_write_reg(connect, ES8316_RESET, ES8316_RESET_CSM_ON) < 0;
        usleep(35000);
        fails += i2c_write_reg(connect, ES8316_SYS_VMIDSEL, 0xFF) < 0;
        fails += i2c_write_reg(connect, ES8316_CLKMGR_ADCOSR, 0x32) < 0;
        fails += i2c_write_reg(connect, ES8316_CLKMGR_CLKSW, 0x7F) < 0;
        fails += i2c_write_reg(connect, ES8316_CLKMGR_CLKSEL, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_CLKMGR_ADCDIV1, 0x11) < 0;
        fails += i2c_write_reg(connect, ES8316_CLKMGR_ADCDIV2, 0x90) < 0;
        fails += i2c_write_reg(connect, ES8316_CLKMGR_DACDIV1, 0x11) < 0;
        fails += i2c_write_reg(connect, ES8316_CLKMGR_DACDIV2, 0x90) < 0;
        fails += i2c_write_reg(connect, ES8316_CLKMGR_CPDIV, 0x00) < 0;
        /* Linux live SERDATA */
        fails += i2c_write_reg(connect, ES8316_SERDATA1, 0x04) < 0;
        fails += i2c_write_reg(connect, ES8316_SERDATA_ADC, 0x40) < 0;
        fails += i2c_write_reg(connect, ES8316_SERDATA_DAC, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_SYS_PDN, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_SYS_LP1, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_SYS_LP2, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_SYS_REF, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_DAC_PDN, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_DAC_SET1, 0x20) < 0; /* soft-mute */
        fails += i2c_write_reg(connect, ES8316_DAC_SET2, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_DAC_SET3, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_DAC_VOLL, 0x17) < 0;
        fails += i2c_write_reg(connect, ES8316_DAC_VOLR, 0x17) < 0;
        fails += i2c_write_reg(connect, ES8316_HPMIX_SEL, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_HPMIX_SWITCH, 0x88) < 0;
        fails += i2c_write_reg(connect, ES8316_HPMIX_PDN, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_HPMIX_VOL, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_CPHP_PDN1, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_CPHP_PDN2, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_CPHP_LDOCTL, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_CPHP_ICAL_VOL, 0x00) < 0;
        fails += i2c_write_reg(connect, ES8316_CPHP_OUTEN, 0x66) < 0;
        fails += i2c_write_reg(connect, ES8316_GPIO_DEBOUNCE, 0x02) < 0;
        /* DAC soft-mute only — pad TX is PADCFGLOCK sticky; don't require pad level. */
        uint32_t hp = 0, spk = 0;
        gpio_rd(connect, amp_padcfg(connect, 41), &hp);
        gpio_rd(connect, amp_padcfg(connect, 40), &spk);
        int mute_ok = (fails == 0);
        printf("I2C_FAILS=%d MUTE_OK=%d CAP_CHANNELS=%d GPIO_HP=0x%08x GPIO_SPK=0x%08x\n",
               fails, mute_ok ? 1 : 0, CAP_CHANNELS, hp, spk);
        rc = mute_ok ? 0 : 1;
        printf("[%c] es8336-config\n", rc == 0 ? '+' : '-');
    } else if (strcmp(cmd, "start") == 0) {
        kr = IOConnectCallScalarMethod(connect, kSel_StartPlayback, NULL, 0, NULL, NULL);
        printf("start -> 0x%x\n", kr); rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "ssp0-dai-config") == 0) {
        rc = ssp0_dai_config_hw_params(connect);
    } else if (strcmp(cmd, "ssp0-dai-free") == 0) {
        rc = ssp0_dai_config_hw_free(connect);
    } else if (strcmp(cmd, "ssp0-reclock") == 0) {
        rc = ssp0_dai_config_hw_free(connect);
        if (rc == 0) rc = ssp0_dai_config_hw_params(connect);
    } else if (strcmp(cmd, "codec-master-tone") == 0 || strcmp(cmd, "bclk-inv-tone") == 0) {
        double freq = (argc >= 3) ? atof(argv[2]) : 440.0;
        int hold_sec = (argc >= 4) ? atoi(argv[3]) : 5;
        if (hold_sec < 1) hold_sec = 1;
        if (hold_sec > 30) hold_sec = 30;
        int master = (strcmp(cmd, "codec-master-tone") == 0);
        mach_vm_address_t pbAddr = 0;
        mach_vm_size_t pbSize = 0;
        kr = IOConnectMapMemory64(connect, kMemPlayback, mach_task_self(), &pbAddr, &pbSize, kIOMapAnywhere);
        if (kr != KERN_SUCCESS || pbAddr == 0) {
            fprintf(stderr, "[-] map playback failed\n");
            rc = 1;
        } else {
            int32_t *pcm = (int32_t *)(uintptr_t)pbAddr;
            uint32_t nframes = (uint32_t)(pbSize / 8);   /* S32 stereo LJ24 */
            if (nframes > 16384) nframes = 16384;
            for (uint32_t f = 0; f < nframes; f++) {
                int32_t s = (int32_t)(1500000000.0 * sin(2.0 * M_PI * freq * (double)f / 48000.0));
                pcm[f * 2 + 0] = s; pcm[f * 2 + 1] = s;
            }
            IOConnectCallScalarMethod(connect, kSel_StopPlayback, NULL, 0, NULL, NULL);
            usleep(50000);
            /* Amp + analog path first */
            gpio_set_level(connect, GPIO_HP_PADCFG, 1);
            gpio_set_level(connect, GPIO_SPK_PADCFG, 1);
            i2c_write_reg(connect, 0x01, 0x7F);
            i2c_write_reg(connect, 0x0C, 0xFF);
            i2c_write_reg(connect, 0x06, 0x01);
            i2c_write_reg(connect, 0x07, 0x90);
            if (master) {
                i2c_write_reg(connect, 0x09, 0x86); /* MASTER | bclk_div=6 */
                printf("[*] codec MASTER SERDATA1=0x86 + SSP fmt=0x0001 (CBP_CFP)\n");
            } else {
                i2c_write_reg(connect, 0x09, 0x26); /* BCLK_INV | bclk_div=6 */
                printf("[*] codec SLAVE BCLK_INV SERDATA1=0x26 + SSP fmt=0x4001\n");
            }
            i2c_write_reg(connect, 0x0A, 0x0C);
            i2c_write_reg(connect, 0x0B, 0x0C);
            i2c_write_reg(connect, 0x14, 0x88);
            i2c_write_reg(connect, 0x16, 0x00);
            i2c_write_reg(connect, 0x17, 0x66);
            i2c_write_reg(connect, 0x18, 0x00);
            i2c_write_reg(connect, 0x2F, 0x00);
            i2c_write_reg(connect, 0x30, 0x00);
            kr = IOConnectCallScalarMethod(connect, kSel_StartPlayback, NULL, 0, NULL, NULL);
            if (kr != KERN_SUCCESS) {
                printf("[-] StartPlayback 0x%x\n", kr);
                rc = 1;
            } else {
                uint16_t fmt = master ? 0x0001 : 0x4001;
                (void)ssp0_dai_config_flags(connect, 0x02, fmt, "HW_FREE");
                rc = ssp0_dai_config_flags(connect, 0x01, fmt, "HW_PARAMS");
                /* Hot CSM while clocks should run */
                i2c_write_reg(connect, 0x00, 0x3F);
                usleep(10000);
                i2c_write_reg(connect, 0x00, 0x80);
                usleep(35000);
                i2c_write_reg(connect, 0x01, 0x7F);
                i2c_write_reg(connect, 0x09, master ? 0x86 : 0x26);
                i2c_write_reg(connect, 0x0C, 0xFF);
                i2c_write_reg(connect, 0x14, 0x88);
                i2c_write_reg(connect, 0x16, 0x00);
                i2c_write_reg(connect, 0x17, 0x66);
                i2c_write_reg(connect, 0x18, 0x00);
                i2c_write_reg(connect, 0x30, 0x00);
                printf("[+] %s %.0f Hz %ds — LISTEN headphones now (SERDATA1=0x%02x)\n",
                       cmd, freq, hold_sec, i2c_read_reg(connect, 0x09));
                int steps = hold_sec * 5;
                for (int t = 0; t < steps; t++) {
                    for (uint32_t f = 0; f < nframes; f++) {
                        int32_t s = (int32_t)(1500000000.0 * sin(2.0 * M_PI * freq * (double)f / 48000.0));
                        pcm[f * 2 + 0] = s; pcm[f * 2 + 1] = s;
                    }
                    if ((t % 5) == 0) {
                        uint64_t pos = 0; uint32_t oc = 1;
                        IOConnectCallScalarMethod(connect, kSel_GetPosition, NULL, 0, &pos, &oc);
                        printf("    t=%ds pos=%llu\n", t / 5, (unsigned long long)pos);
                    }
                    usleep(200000);
                }
                /* Restore slave default */
                i2c_write_reg(connect, 0x09, 0x06);
                IOConnectCallScalarMethod(connect, kSel_StopPlayback, NULL, 0, NULL, NULL);
                (void)ssp0_dai_config_hw_params(connect);
            }
            IOConnectUnmapMemory64(connect, kMemPlayback, mach_task_self(), pbAddr);
        }
    } else if (strcmp(cmd, "hot-csm") == 0) {
        /* Relock codec state machine while BCLK/FS should already be pulsing. */
        printf("[*] hot-csm: RESET 0x3F -> CSM_ON 0x80 + restore analog\n");
        i2c_write_reg(connect, 0x00, 0x3F);
        usleep(10000);
        i2c_write_reg(connect, 0x00, 0x80);
        usleep(35000);
        i2c_write_reg(connect, 0x01, 0x7F);
        apply_linux_play_codec(connect);
        printf("[+] hot-csm done (0x00=%02x 0x01=%02x 0x30=%02x)\n",
               i2c_read_reg(connect, 0x00), i2c_read_reg(connect, 0x01),
               i2c_read_reg(connect, 0x30));
        rc = 0;
    } else if (strcmp(cmd, "playtone") == 0) {
        /* playtone [freq_hz] [hold_sec] [spk|hp]
         * Keep playback DMA mapped and rewrite sine every 200ms — CoreAudio/HAL
         * otherwise overwrites the shared buffer with silence after we unmap.
         * Optional 3rd route arg forces speakers/headphones (ignore jack detect). */
        double freq = (argc >= 3) ? atof(argv[2]) : 750.0;
        int hold_sec = (argc >= 4) ? atoi(argv[3]) : 5;
        int force_route = 0; /* 0=auto 1=spk 2=hp */
        if (argc >= 5) {
            if (strcmp(argv[4], "spk") == 0 || strcmp(argv[4], "speakers") == 0)
                force_route = 1;
            else if (strcmp(argv[4], "hp") == 0 || strcmp(argv[4], "headphones") == 0)
                force_route = 2;
        }
        if (hold_sec < 1) hold_sec = 1;
        if (hold_sec > 60) hold_sec = 60;
        mach_vm_address_t pbAddr = 0;
        mach_vm_size_t pbSize = 0;
        kr = IOConnectMapMemory64(connect, kMemPlayback, mach_task_self(), &pbAddr, &pbSize, kIOMapAnywhere);
        if (kr != KERN_SUCCESS || pbAddr == 0 || pbSize < 65536) {
            fprintf(stderr, "[-] map playback failed: kr=0x%x size=%llu\n", kr, (unsigned long long)pbSize);
            rc = 1;
        } else {
            int32_t *pcm = (int32_t *)(uintptr_t)pbAddr;
            uint32_t nframes = (uint32_t)(pbSize / 8);   /* S32 stereo full-scale (Linux FE) */
            if (nframes > 16384) nframes = 16384;
            for (uint32_t f = 0; f < nframes; f++) {
                int32_t s = (int32_t)(1500000000.0 * sin(2.0 * M_PI * freq * (double)f / 48000.0));
                pcm[f * 2 + 0] = s;
                pcm[f * 2 + 1] = s;
            }
            /* Do NOT call DAI_CONFIG here — StartPlayback already does
             * HW_PARAMS + TRIG host. A pre-call just races CoreAudio and, if
             * StartPlayback then times out mid-TRIG, the mailbox stays wedged. */
            kr = IOConnectCallScalarMethod(connect, kSel_StartPlayback, NULL, 0, NULL, NULL);
            if (kr != KERN_SUCCESS) {
                printf("[-] StartPlayback 0x%x — resetdsp+init then retry once\n", kr);
                IOConnectCallScalarMethod(connect, kSel_ForceResetDSP, NULL, 0, NULL, NULL);
                usleep(500000);
                IOConnectCallScalarMethod(connect, kSel_TriggerInitDSP, NULL, 0, NULL, NULL);
                usleep(800000);
                kr = IOConnectCallScalarMethod(connect, kSel_StartPlayback, NULL, 0, NULL, NULL);
            }
            if (kr != KERN_SUCCESS) {
                printf("[-] StartPlayback failed: 0x%x\n", kr);
                rc = 1;
            } else {
                /* Re-assert clocks after TRIG_START (FW may gate until running). */
                (void)ssp0_dai_config_hw_params(connect);
                usleep(70000);
                uint64_t flag = 0; uint32_t oc = 1; uint64_t in = 0x4F;
                IOConnectCallScalarMethod(connect, kSel_I2CReadReg, &in, 1, &flag, &oc);
                bool hp_in = (flag != 0) && ((flag & 0x04) == 0); /* flag==0 ⇒ not HP */
                if (force_route == 1) hp_in = false;
                if (force_route == 2) hp_in = true;
                if (hp_in) {
                    gpio_set_level(connect, GPIO_HP_PADCFG, 0);  /* headphones: both amps off */
                    gpio_set_level(connect, GPIO_SPK_PADCFG, 0);
                } else {
                    gpio_set_level(connect, GPIO_SPK_PADCFG, 1); /* speakers: both amps on */
                    gpio_set_level(connect, GPIO_HP_PADCFG, 1);
                }
                apply_linux_play_codec_route(connect, hp_in ? 0 : 1);

                uint32_t hp = 0, spk = 0;
                gpio_rd(connect, amp_padcfg(connect, 41), &hp);
                gpio_rd(connect, amp_padcfg(connect, 40), &spk);
                printf("[+] playtone %.1f Hz (kr=0x%x) — listen %ds, sine re-injected (anti-HAL-silence)\n",
                       freq, kr, hold_sec);
                printf("    Route: %s | SPK pin40 TX=%u HP pin41 TX=%u (1=ON) FLAG=0x%02llx jack=%s%s\n",
                       hp_in ? "HEADPHONES" : "INTERNAL SPEAKERS",
                       spk & 1u, hp & 1u,
                       (unsigned long long)(flag & 0xff),
                       (flag != 0 && (flag & 0x04) == 0) ? "IN" : "OUT",
                       force_route ? " (forced)" : "");
                printf("    Note: HAL/afplay also feeds DMA — prefer: afplay /tmp/matebook_tone750.wav\n");
                int steps = hold_sec * 5;
                for (int t = 0; t < steps; t++) {
                    for (uint32_t f = 0; f < nframes; f++) {
                        int32_t s = (int32_t)(1500000000.0 * sin(2.0 * M_PI * freq * (double)f / 48000.0));
                        pcm[f * 2 + 0] = s;
                        pcm[f * 2 + 1] = s;
                    }
                    /* HAL can nudge I2C — keep analog powered every tick */
                    if ((t % 5) == 0) {
                        i2c_write_reg(connect, 0x0d, 0x06);
                        i2c_write_reg(connect, 0x15, 0x00);
                        i2c_write_reg(connect, 0x17, 0x66);
                        i2c_write_reg(connect, ES8316_DAC_SET1, 0x10);
                        uint64_t pos = 0; uint32_t outCnt = 1;
                        IOConnectCallScalarMethod(connect, kSel_GetPosition, NULL, 0, &pos, &outCnt);
                        int peak = 0;
                        for (uint32_t i = 0; i < nframes * 2; i++) {
                            int v = pcm[i]; if (v < 0) v = -v; if (v > peak) peak = v;
                        }
                        printf("    t=%ds pos=%llu peak=%d pdn=0x%02x\n", t / 5,
                               (unsigned long long)pos, peak, i2c_read_reg(connect, 0x0d));
                    }
                    usleep(200000);
                }
                /* Always leave speakers armed if jack out — HP playtone used to
                 * leave TX=0 + (previously) quiet vols → speakers “dead”. */
                {
                    uint64_t flag2 = 0; uint32_t oc2 = 1; uint64_t in2 = 0x4F;
                    IOConnectCallScalarMethod(connect, kSel_I2CReadReg, &in2, 1, &flag2, &oc2);
                    bool jack_in = (flag2 != 0) && ((flag2 & 0x04) == 0);
                    if (!jack_in) {
                        gpio_set_level(connect, GPIO_SPK_PADCFG, 1);
                        gpio_set_level(connect, GPIO_HP_PADCFG, 1);
                        apply_linux_play_codec(connect);
                        printf("[*] playtone exit: restored SPEAKERS (amps TX=1, 0 dB)\n");
                    }
                }
                IOConnectUnmapMemory64(connect, kMemPlayback, mach_task_self(), pbAddr);
                rc = (kr == KERN_SUCCESS) ? 0 : 1;
            }
        }
    } else if (strcmp(cmd, "dump-play") == 0) {
        dump_play_snapshot(connect);
        rc = 0;
    } else if (strcmp(cmd, "probe-dma") == 0) {
        double w = (argc >= 3) ? atof(argv[2]) : 1.0;
        if (w < 0.2) w = 0.2;
        probe_dma(connect, w);
        rc = 0;
    } else if (strcmp(cmd, "alltone") == 0) {
        /*
         * Silence root-cause matrix — cycle configs. User listens each phase.
         * Causes ruled in/out by ears:
         *  A v19_slave   — SERDATA 0x06 (doc gold) + 19.2M
         *  B v19_linux   — SERDATA dump bytes + 19.2M
         *  C v19_master  — codec I2S master + SSP CBP_CFP
         *  D v24_s16     — MCLK 24M BCLK 1.536M S16
         *  E v24_tplg    — MCLK 24M BCLK 2.4M bits24 slot25 (DMA still S16 — probe only)
         * Also TRIG_START host(0)+DAI(4)+sched(5).
         */
        int hold = (argc >= 3) ? atoi(argv[2]) : 6;
        if (hold < 3) hold = 3;
        if (hold > 15) hold = 15;
        mach_vm_address_t pbAddr = 0;
        mach_vm_size_t pbSize = 0;
        kr = IOConnectMapMemory64(connect, kMemPlayback, mach_task_self(), &pbAddr, &pbSize, kIOMapAnywhere);
        if (kr != KERN_SUCCESS || pbAddr == 0) {
            fprintf(stderr, "[-] map playback failed\n");
            rc = 1;
        } else {
            int32_t *pcm = (int32_t *)(uintptr_t)pbAddr;
            uint32_t nframes = (uint32_t)(pbSize / 8);   /* S32 stereo LJ24 */
            if (nframes > 16384) nframes = 16384;
            double freq = 1000.0;

            struct {
                const char *name;
                void (*codec)(io_connect_t);
                uint16_t fmt;
                uint32_t mclk, bclk, bits;
                uint16_t slot;
            } phases[] = {
                /* A = exact tplg hw_config + exact Linux codec bytes. */
                { "A_tplg_gold_s24", apply_codec_linux_s24_spk, 0x4001, 24000000, 2400000, 24, 25 },
                { "B_s24_mclk19", apply_codec_linux_s24_spk, 0x4001, 19200000, 2400000, 24, 25 },
                { "C_s16_mclk24", apply_codec_v19_slave, 0x4001, 24000000, 1536000, 16, 16 },
                { "D_s16_mclk19_old", apply_codec_v19_slave, 0x4001, 19200000, 1536000, 16, 16 },
                { "E_codec_master_s24", apply_codec_linux_s24_spk, 0x0001, 24000000, 2400000, 24, 25 },
            };

            printf("======== ALLTONE — %d phases × %ds — LISTEN EACH ========\n",
                   (int)(sizeof(phases)/sizeof(phases[0])), hold);
            printf("Amps: both TX stuck LOW (PADCFGLOCKTX=0x01fff9fb) = both ON (active-low).\n");
            printf("SSP BAR4@0x10000 = 0xFFFFFFFF (SOF-managed; host cannot peek clocks).\n");
            printf("DMA SD7 RUN was observed; silence ⇒ I2S/MCLK to codec or analog still dead.\n\n");

            for (size_t pi = 0; pi < sizeof(phases)/sizeof(phases[0]); pi++) {
                printf("\n******** PHASE %s — LISTEN %ds @ %.0f Hz ********\n",
                       phases[pi].name, hold, freq);
                IOConnectCallScalarMethod(connect, kSel_StopPlayback, NULL, 0, NULL, NULL);
                usleep(80000);
                (void)ssp0_dai_config_flags_ex(connect, 0x02, phases[pi].fmt,
                    phases[pi].mclk, phases[pi].bclk, phases[pi].bits, phases[pi].slot, "HW_FREE");
                phases[pi].codec(connect);
                if (strcmp(phases[pi].name, "E_codec_master_s24") == 0)
                    i2c_write_reg(connect, ES8316_SERDATA1, 0x84);
                gpio_set_level(connect, GPIO_SPK_PADCFG, 1);
                gpio_set_level(connect, GPIO_HP_PADCFG, 1); /* both amps on */
                usleep(70000);
                for (uint32_t f = 0; f < nframes; f++) {
                    int32_t s = (int32_t)(1500000000.0 * sin(2.0 * M_PI * freq * (double)f / 48000.0));
                    pcm[f * 2] = s; pcm[f * 2 + 1] = s;
                }
                kr = IOConnectCallScalarMethod(connect, kSel_StartPlayback, NULL, 0, NULL, NULL);
                printf("    StartPlayback kr=0x%x\n", kr);
                if (kr != KERN_SUCCESS) {
                    printf("    [-] StartPlayback failed — skip TRIG (mailbox may be wedged)\n");
                } else {
                    (void)ssp0_dai_config_flags_ex(connect, 0x01, phases[pi].fmt,
                        phases[pi].mclk, phases[pi].bclk, phases[pi].bits, phases[pi].slot, "HW_PARAMS");
                    /* No extra TRIG_START here. StartPlayback already triggers the
                     * host component, and TRIG on the DAI (4) or scheduler (5)
                     * comp ids is rejected and wedges the IPC mailbox for every
                     * later phase (replyErr 0xffffffff on all of B..E). */
                }
                phases[pi].codec(connect);
                if (strcmp(phases[pi].name, "E_codec_master_s24") == 0)
                    i2c_write_reg(connect, ES8316_SERDATA1, 0x84);
                printf("    I2C 0x01=%02x 0x06=%02x 0x07=%02x 0x09=%02x 0x0B=%02x 0x16=%02x 0x30=%02x\n",
                       i2c_read_reg(connect, 0x01), i2c_read_reg(connect, 0x06),
                       i2c_read_reg(connect, 0x07), i2c_read_reg(connect, 0x09),
                       i2c_read_reg(connect, 0x0b), i2c_read_reg(connect, 0x16),
                       i2c_read_reg(connect, 0x30));
                for (int t = 0; t < hold; t++) {
                    for (uint32_t f = 0; f < nframes; f++) {
                        int32_t s = (int32_t)(1500000000.0 * sin(2.0 * M_PI * freq * (double)f / 48000.0));
                        pcm[f * 2] = s; pcm[f * 2 + 1] = s;
                    }
                    printf("    ... %s t=%ds\n", phases[pi].name, t);
                    sleep(1);
                }
            }
            IOConnectCallScalarMethod(connect, kSel_StopPlayback, NULL, 0, NULL, NULL);
            IOConnectUnmapMemory64(connect, kMemPlayback, mach_task_self(), pbAddr);
            printf("\n======== ALLTONE DONE — which phase (A/B/C/D/E) had sound? ========\n");
            rc = 0;
        }
    } else if (strcmp(cmd, "stop") == 0) {
        kr = IOConnectCallScalarMethod(connect, kSel_StopPlayback, NULL, 0, NULL, NULL);
        printf("stop -> 0x%x\n", kr); rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "pos") == 0) {
        uint64_t out = 0; uint32_t outCnt = 1;
        kr = IOConnectCallScalarMethod(connect, kSel_GetPosition, NULL, 0, &out, &outCnt);
        printf("pos = %llu (kr=0x%x)\n", (unsigned long long)out, kr);
        rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "capstart") == 0) {
        if (map_capture_if_needed(connect, &capMapAddr, &capMapSize) != 0) {
            rc = 1;
        } else {
            kr = IOConnectCallScalarMethod(connect, kSel_StartCapture, NULL, 0, NULL, NULL);
            printf("capstart -> 0x%x\n", kr); rc = (kr == KERN_SUCCESS) ? 0 : 1;
        }
    } else if (strcmp(cmd, "capstop") == 0) {
        kr = IOConnectCallScalarMethod(connect, kSel_StopCapture, NULL, 0, NULL, NULL);
        printf("capstop -> 0x%x\n", kr); rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "cappos") == 0) {
        if (map_capture_if_needed(connect, &capMapAddr, &capMapSize) != 0) {
            rc = 1;
        } else {
            uint64_t out = 0; uint32_t outCnt = 1;
            kr = IOConnectCallScalarMethod(connect, kSel_GetCapPosition, NULL, 0, &out, &outCnt);
            printf("cappos = %llu (kr=0x%x)\n", (unsigned long long)out, kr);
            rc = (kr == KERN_SUCCESS) ? 0 : 1;
        }
    } else if (strcmp(cmd, "rdbar0") == 0 || strcmp(cmd, "rdbar4") == 0) {
        if (argc < 4) { usage(argv[0]); rc = 1; goto done; }
        int bar = (cmd[5] == '0') ? 0 : 4;
        int width = atoi(argv[2]);
        uint64_t off = 0;
        if (parse_u64(argv[3], &off) != 0) { fprintf(stderr, "bad offset\n"); rc = 1; goto done; }
        int sel = width_to_sel_rd(bar, width);
        if (sel < 0) { fprintf(stderr, "bad width\n"); rc = 1; goto done; }
        uint64_t in = off;
        uint64_t out[2] = {0};
        uint32_t outCnt = (width == 64) ? 2 : 1;
        kr = IOConnectCallScalarMethod(connect, sel, &in, 1, out, &outCnt);
        if (kr != KERN_SUCCESS) {
            printf("[-] read failed: 0x%x\n", kr); rc = 1;
        } else if (width == 64) {
            printf("BAR%d[0x%llx] = 0x%08llx%08llx\n", bar, (unsigned long long)off,
                   (unsigned long long)out[1], (unsigned long long)out[0]);
        } else {
            printf("BAR%d[0x%llx] = 0x%llx\n", bar, (unsigned long long)off,
                   (unsigned long long)out[0]);
        }
    } else if (strcmp(cmd, "wrbar0") == 0 || strcmp(cmd, "wrbar4") == 0) {
        if (argc < 5) { usage(argv[0]); rc = 1; goto done; }
        int bar = (cmd[5] == '0') ? 0 : 4;
        int width = atoi(argv[2]);
        uint64_t off = 0, val = 0, hi = 0;
        if (parse_u64(argv[3], &off) != 0 || parse_u64(argv[4], &val) != 0) {
            fprintf(stderr, "bad args\n"); rc = 1; goto done;
        }
        int sel = width_to_sel_wr(bar, width);
        if (sel < 0) { fprintf(stderr, "bad width\n"); rc = 1; goto done; }
        uint64_t in[3];
        uint32_t inCnt;
        if (width == 64) {
            if (argc < 6 || parse_u64(argv[5], &hi) != 0) {
                /* allow single 64-bit value: split lo/hi */
                in[0] = off;
                in[1] = (uint32_t)(val & 0xFFFFFFFFULL);
                in[2] = (uint32_t)(val >> 32);
            } else {
                in[0] = off; in[1] = val; in[2] = hi;
            }
            inCnt = 3;
        } else {
            in[0] = off; in[1] = val; inCnt = 2;
        }
        kr = IOConnectCallScalarMethod(connect, sel, in, inCnt, NULL, NULL);
        printf("[%c] write BAR%d[0x%llx] width=%d -> 0x%x\n",
               kr == KERN_SUCCESS ? '+' : '-', bar, (unsigned long long)off, width, kr);
        rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "rdgpio") == 0) {
        if (argc < 3) { usage(argv[0]); rc = 1; goto done; }
        uint64_t off = 0;
        if (parse_u64(argv[2], &off) != 0) { fprintf(stderr, "bad offset\n"); rc = 1; goto done; }
        uint64_t out = 0; uint32_t outCnt = 1;
        kr = IOConnectCallScalarMethod(connect, kSel_ReadGPIO32, &off, 1, &out, &outCnt);
        if (kr == KERN_SUCCESS)
            printf("GPIO[0x%llx] = 0x%08llx\n", (unsigned long long)off, (unsigned long long)out);
        else { printf("[-] rdgpio failed: 0x%x\n", kr); rc = 1; }
    } else if (strcmp(cmd, "wrgpio") == 0) {
        if (argc < 4) { usage(argv[0]); rc = 1; goto done; }
        uint64_t in[2];
        if (parse_u64(argv[2], &in[0]) != 0 || parse_u64(argv[3], &in[1]) != 0) {
            fprintf(stderr, "bad args\n"); rc = 1; goto done;
        }
        kr = IOConnectCallScalarMethod(connect, kSel_WriteGPIO32, in, 2, NULL, NULL);
        printf("[%c] wrgpio -> 0x%x\n", kr == KERN_SUCCESS ? '+' : '-', kr);
        rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else if (strcmp(cmd, "rdi2c") == 0) {
        if (argc < 3) { usage(argv[0]); rc = 1; goto done; }
        uint64_t reg = 0;
        if (parse_u64(argv[2], &reg) != 0) { fprintf(stderr, "bad reg\n"); rc = 1; goto done; }
        uint64_t out = 0; uint32_t outCnt = 1;
        kr = IOConnectCallScalarMethod(connect, kSel_I2CReadReg, &reg, 1, &out, &outCnt);
        if (kr == KERN_SUCCESS)
            printf("I2C[0x%02llx] = 0x%02llx\n", (unsigned long long)reg, (unsigned long long)out);
        else { printf("[-] rdi2c failed: 0x%x\n", kr); rc = 1; }
    } else if (strcmp(cmd, "wri2c") == 0) {
        if (argc < 4) { usage(argv[0]); rc = 1; goto done; }
        uint64_t in[2];
        if (parse_u64(argv[2], &in[0]) != 0 || parse_u64(argv[3], &in[1]) != 0) {
            fprintf(stderr, "bad args\n"); rc = 1; goto done;
        }
        kr = IOConnectCallScalarMethod(connect, kSel_I2CWriteReg, in, 2, NULL, NULL);
        printf("[%c] wri2c -> 0x%x\n", kr == KERN_SUCCESS ? '+' : '-', kr);
        rc = (kr == KERN_SUCCESS) ? 0 : 1;
    } else {
        usage(argv[0]);
        rc = 1;
    }

done:
    if (capMapAddr != 0) {
        IOConnectUnmapMemory64(connect, kMemCapture, mach_task_self(), capMapAddr);
        capMapAddr = 0;
    }
    IOServiceClose(connect);
    IOObjectRelease(service);
    return rc;
}
