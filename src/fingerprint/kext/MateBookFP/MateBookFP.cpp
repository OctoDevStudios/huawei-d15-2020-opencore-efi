/*
 * MateBookFP — minimal macOS bridge for the Goodix GXFP51A0 fingerprint sensor (SPI).
 *
 * Hardware (BOHB-WAX9, same as HONOR BBR-WAX9 / szlukabence's notes):
 *   Intel LPSS SPI #1  PCI 00:1e.3 [8086:02ab]  BAR0: SSP @0x000, LPSS private @0x200
 *   reset  = pin 189 (UART0_RXD), active-HIGH, padcfg 0xFD6A0680 (LOW = MCU running)
 *   irq    = pin 204 (UART2_CTSB), level, padcfg 0xFD6A0770
 *
 * The kext contains NO sensor logic: it exposes to root
 *   - 32-bit read/write in BAR0 (offsets < 0x400),
 *   - reading both pads, writing only the TXSTATE bit of the reset pad,
 *   - one full-duplex PIO SPI transfer (CS driven through CS_CTRL 0x224),
 *   - a 64 KB shared buffer for the transfer data.
 * Everything else (SSP init, protocol, TLS) lives in userspace.
 */
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <kern/clock.h>
#include <mach/kmod.h>

/* kmod_info is required for OpenCore injection (KextFindKmodAddress) */
extern "C" kern_return_t MateBookFP_start(kmod_info_t *, void *) { return KERN_SUCCESS; }
extern "C" kern_return_t MateBookFP_stop(kmod_info_t *, void *) { return KERN_SUCCESS; }
KMOD_EXPLICIT_DECL(com.hackintosh.MateBookFP, "0.2.0", MateBookFP_start, MateBookFP_stop)

#define FP_GPIO_COMM_PHYS   0xFD6A0000ULL
#define FP_GPIO_COMM_LEN    0x1000
#define FP_PAD_RESET        0x680
#define FP_PAD_IRQ          0x770
#define FP_BAR_LIMIT        0x400
#define FP_SHARED_LEN       65536

#define SSCR0   0x00
#define SSSR    0x08
#define SSDR    0x10
#define CS_CTRL 0x224
#define SSSR_TNF (1u << 2)
#define SSSR_RNE (1u << 3)
#define SSSR_BSY (1u << 4)

enum {
    kSel_ReadBar   = 0,   /* in: off            out: val */
    kSel_WriteBar  = 1,   /* in: off, val */
    kSel_ReadPads  = 2,   /* out: reset_dw0, reset_dw1, irq_dw0, irq_dw1 */
    kSel_SetReset  = 3,   /* in: level (0/1)    out: dw0 relu */
    kSel_Transfer  = 4,   /* in: len, csAssert, csIdle, timeoutUs[, csSetupUs]  out: bytes done */
    kSel_Info      = 5,   /* out: barPhys, wakeGeneration, vendorDevice, pmcsr */
    kSel_Count
};

class MateBookFP : public IOService {
    OSDeclareDefaultStructors(MateBookFP)
public:
    IOService *probe(IOService *provider, SInt32 *score) override {
        provider->setProperty("MateBookFP-stage", "probe");
        return IOService::probe(provider, score);
    }
    bool start(IOService *provider) override;
    void stop(IOService *provider) override;

    IOPCIDevice              *pci = nullptr;
    IOMemoryMap              *barMap = nullptr;
    IOMemoryMap              *gpioMap = nullptr;
    volatile UInt8           *bar = nullptr;
    volatile UInt8           *gpio = nullptr;
    IOBufferMemoryDescriptor *shared = nullptr;
    IOLock                   *lock = nullptr;
    UInt32                    pmcsr = 0xffffffff;

    UInt32 rd(UInt32 o) { return *(volatile UInt32 *)(bar + o); }
    void   wr(UInt32 o, UInt32 v) { *(volatile UInt32 *)(bar + o) = v; }
    UInt32 grd(UInt32 o) { return *(volatile UInt32 *)(gpio + o); }
    void   gwr(UInt32 o, UInt32 v) { *(volatile UInt32 *)(gpio + o) = v; }

    UInt8                     pmOff = 0;
    volatile bool             asleep = false;
    volatile UInt32           wakeGen = 0;

    IOReturn transfer(UInt32 len, UInt32 csAssert, UInt32 csIdle, UInt32 timeoutUs,
                      UInt32 setupUs, UInt32 *done);
    IOReturn setPowerState(unsigned long state, IOService *dev) override;
    void     forceD0();
    bool     alive() { UInt32 v = rd(SSSR); return v != 0xffffffffu; }
};

OSDefineMetaClassAndStructors(MateBookFP, IOService)

bool MateBookFP::start(IOService *provider)
{
    provider->setProperty("MateBookFP-stage", "start");
    if (!IOService::start(provider)) return false;
    pci = OSDynamicCast(IOPCIDevice, provider);
    if (!pci) return false;
    pci->retain();

    /* D0 + MMIO, no bus mastering (PIO only) */
    pci->findPCICapability(kIOPCIPowerManagementCapability, &pmOff);
    forceD0();

    barMap = pci->mapDeviceMemoryWithIndex(0);
    if (!barMap) { IOLog("MateBookFP: BAR0 map failed\n"); provider->setProperty("MateBookFP-stage", "fail-bar0"); return false; }
    bar = (volatile UInt8 *)barMap->getVirtualAddress();

    IODeviceMemory *gm = IODeviceMemory::withRange(FP_GPIO_COMM_PHYS, FP_GPIO_COMM_LEN);
    if (gm) { gpioMap = gm->map(kIOMapInhibitCache); gm->release(); }
    if (gpioMap) gpio = (volatile UInt8 *)gpioMap->getVirtualAddress();

    shared = IOBufferMemoryDescriptor::withOptions(kIODirectionInOut | kIOMemoryKernelUserShared,
                                                   FP_SHARED_LEN, page_size);
    lock = IOLockAlloc();
    if (!shared || !lock) { provider->setProperty("MateBookFP-stage", "fail-alloc"); return false; }

    IOLog("MateBookFP: up BAR0=%#llx SSCR0=%#x CS_CTRL=%#x pad189=%#x pad204=%#x pmcsr=%#x\n",
          barMap->getPhysicalAddress(), rd(SSCR0), rd(CS_CTRL),
          gpio ? grd(FP_PAD_RESET) : 0, gpio ? grd(FP_PAD_IRQ) : 0, pmcsr);
    /* Power management: no transfers while asleep, D0 restored on wake. */
    static IOPMPowerState states[2] = {
        { kIOPMPowerStateVersion1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { kIOPMPowerStateVersion1, kIOPMPowerOn, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0 },
    };
    PMinit();
    provider->joinPMtree(this);
    registerPowerDriver(this, states, 2);

    setProperty("FP-Bridge", "GXFP51A0 via LPSS SPI1");
    provider->setProperty("MateBookFP-stage", "running");
    registerService();
    return true;
}

void MateBookFP::forceD0()
{
    if (pmOff) {
        UInt16 cs = pci->configRead16(pmOff + 4);
        if (cs & 3) { pci->configWrite16(pmOff + 4, cs & ~3); IOSleep(20); }
        pmcsr = pci->configRead16(pmOff + 4);
    }
    pci->setMemoryEnable(true);
}

IOReturn MateBookFP::setPowerState(unsigned long state, IOService *)
{
    if (state == 0) {
        IOLockLock(lock);            /* wait for any in-flight transfer */
        asleep = true;
        IOLockUnlock(lock);
    } else if (asleep) {
        IOLockLock(lock);
        forceD0();
        asleep = false;
        wakeGen++;                   /* userspace must reconfigure the SSP */
        IOLockUnlock(lock);
    }
    return kIOPMAckImplied;
}

void MateBookFP::stop(IOService *provider)
{
    PMstop();
    OSSafeReleaseNULL(gpioMap);
    OSSafeReleaseNULL(barMap);
    OSSafeReleaseNULL(shared);
    if (lock) { IOLockFree(lock); lock = nullptr; }
    OSSafeReleaseNULL(pci);
    IOService::stop(provider);
}

/* Full-duplex PIO: TX from the shared buffer, RX written in place. At most 16 bytes
 * in flight so the RX FIFO never overflows (depth 64 on LPSS). */
IOReturn MateBookFP::transfer(UInt32 len, UInt32 csAssert, UInt32 csIdle, UInt32 timeoutUs,
                              UInt32 setupUs, UInt32 *done)
{
    *done = 0;
    if (len == 0 || len > FP_SHARED_LEN) return kIOReturnBadArgument;
    if (asleep) return kIOReturnNotReady;
    /* LPSS block powered off / in reset (0xffffffff) or SSP not enabled: refuse right
     * away. v0.1.0 looped forever here after sleep (100 % CPU). */
    UInt32 st = rd(SSSR);
    if (st == 0xffffffffu || !(rd(SSCR0) & (1u << 7))) return kIOReturnNotReady;
    UInt8 *buf = (UInt8 *)shared->getBytesNoCopy();
    UInt32 tx = 0, rx = 0;
    uint64_t deadline;
    clock_interval_to_deadline(timeoutUs ? timeoutUs : 2000000, kMicrosecondScale, &deadline);

    for (int n = 0; n < 256 && (rd(SSSR) & SSSR_RNE); n++) (void)rd(SSDR);   /* drain leftover RX (bounded) */
    wr(CS_CTRL, csAssert);
    if (setupUs) IODelay(setupUs > 1000 ? 1000 : setupUs);   /* the STM32 arms its SPI on the CS edge */
    while (rx < len) {
        while (tx < len && (tx - rx) < 16 && (rd(SSSR) & SSSR_TNF)) wr(SSDR, buf[tx++]);
        while (rx < len && (rd(SSSR) & SSSR_RNE)) buf[rx++] = (UInt8)rd(SSDR);
        if (mach_absolute_time() > deadline) break;
    }
    while (rd(SSSR) & SSSR_BSY) { if (mach_absolute_time() > deadline) break; }
    wr(CS_CTRL, csIdle);
    *done = rx;
    return rx == len ? kIOReturnSuccess : kIOReturnTimeout;
}

/* ------------------------------------------------------------------ */

class MateBookFPUserClient : public IOUserClient {
    OSDeclareDefaultStructors(MateBookFPUserClient)
    MateBookFP *dev = nullptr;
public:
    bool initWithTask(task_t t, void *sec, UInt32 type) override {
        if (clientHasPrivilege(sec, kIOClientPrivilegeAdministrator) != kIOReturnSuccess) return false;
        return IOUserClient::initWithTask(t, sec, type);
    }
    bool start(IOService *p) override {
        dev = OSDynamicCast(MateBookFP, p);
        return dev && IOUserClient::start(p);
    }
    IOReturn clientClose() override { terminate(); return kIOReturnSuccess; }
    IOReturn clientMemoryForType(UInt32 type, IOOptionBits *opts, IOMemoryDescriptor **mem) override {
        if (type != 0 || !dev->shared) return kIOReturnBadArgument;
        dev->shared->retain(); *mem = dev->shared; *opts = 0;
        return kIOReturnSuccess;
    }
    IOReturn externalMethod(uint32_t sel, IOExternalMethodArguments *a,
                            IOExternalMethodDispatch *, OSObject *, void *) override;
};

OSDefineMetaClassAndStructors(MateBookFPUserClient, IOUserClient)

IOReturn MateBookFPUserClient::externalMethod(uint32_t sel, IOExternalMethodArguments *a,
                                              IOExternalMethodDispatch *, OSObject *, void *)
{
    static const UInt32 nIn[kSel_Count]  = { 1, 2, 0, 1, 4, 0 };
    static const UInt32 nOut[kSel_Count] = { 1, 0, 4, 1, 1, 4 };
    if (sel >= kSel_Count || a->scalarOutputCount < nOut[sel] ||
        !(a->scalarInputCount == nIn[sel] || (sel == kSel_Transfer && a->scalarInputCount == 5)))
        return kIOReturnBadArgument;
    const uint64_t *in = a->scalarInput;
    uint64_t *out = a->scalarOutput;
    IOReturn r = kIOReturnSuccess;

    IOLockLock(dev->lock);
    switch (sel) {
    case kSel_ReadBar:
        if (dev->asleep) { r = kIOReturnNotReady; break; }
        if (in[0] >= FP_BAR_LIMIT || (in[0] & 3)) { r = kIOReturnBadArgument; break; }
        out[0] = dev->rd((UInt32)in[0]);
        break;
    case kSel_WriteBar:
        if (dev->asleep) { r = kIOReturnNotReady; break; }
        if (in[0] >= FP_BAR_LIMIT || (in[0] & 3)) { r = kIOReturnBadArgument; break; }
        dev->wr((UInt32)in[0], (UInt32)in[1]);
        break;
    case kSel_ReadPads:
        if (!dev->gpio) { r = kIOReturnNoDevice; break; }
        out[0] = dev->grd(FP_PAD_RESET); out[1] = dev->grd(FP_PAD_RESET + 4);
        out[2] = dev->grd(FP_PAD_IRQ);   out[3] = dev->grd(FP_PAD_IRQ + 4);
        break;
    case kSel_SetReset: {
        if (!dev->gpio) { r = kIOReturnNoDevice; break; }
        UInt32 v = dev->grd(FP_PAD_RESET);
        /* safety check: pad in GPIO mode (PMODE=0) with output enabled (TXDIS=0) */
        if (((v >> 10) & 7) != 0 || (v & (1u << 8))) { r = kIOReturnNotPermitted; break; }
        dev->gwr(FP_PAD_RESET, (v & ~1u) | (in[0] ? 1u : 0u));
        out[0] = dev->grd(FP_PAD_RESET);
        break;
    }
    case kSel_Transfer: {
        UInt32 done = 0;
        r = dev->transfer((UInt32)in[0], (UInt32)in[1], (UInt32)in[2], (UInt32)in[3],
                          a->scalarInputCount == 5 ? (UInt32)in[4] : 0, &done);
        out[0] = done;
        break;
    }
    case kSel_Info:
        out[0] = dev->barMap->getPhysicalAddress();
        out[1] = dev->wakeGen;
        out[2] = dev->pci->configRead32(0);
        out[3] = dev->pmcsr;
        break;
    }
    IOLockUnlock(dev->lock);
    return r;
}
