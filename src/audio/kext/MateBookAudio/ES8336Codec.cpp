#include "ES8336Codec.hpp"
#include <IOKit/IOLib.h>

// Register helper macros
static inline UInt32 rd32(volatile UInt8 *b, UInt32 o) { return *(volatile UInt32*)(b + o); }
static inline void   wr32(volatile UInt8 *b, UInt32 o, UInt32 v) { *(volatile UInt32*)(b + o) = v; }

ES8336Codec::ES8336Codec()
    : voodooNub(nullptr), gpioMap(nullptr), gpioBase(nullptr),
      gpioAmpMap(nullptr), gpioAmpBase(nullptr) {
    statusMsg[0] = '\0';
}

ES8336Codec::~ES8336Codec() {
    free();
}

void ES8336Codec::free() {
    if (voodooNub) {
        voodooNub->release();
        voodooNub = nullptr;
    }
    if (gpioMap) {
        gpioMap->release();
        gpioMap = nullptr;
    }
    gpioBase = nullptr;
    if (gpioAmpMap) {
        gpioAmpMap->release();
        gpioAmpMap = nullptr;
    }
    gpioAmpBase = nullptr;
}

bool ES8336Codec::init() {
    headsetMic = false;
    // 1. Locate VoodooI2CDeviceNub for ESSX8336
    if (initVoodooNub()) {
        snprintf(statusMsg, sizeof(statusMsg), "Active via VoodooI2CDeviceNub");
        IOLog("MateBookAudio: ES8336 initialized via VoodooI2CDeviceNub\n");
    } else {
        IOLog("MateBookAudio: VoodooI2CDeviceNub not yet matched, will retry\n");
    }

    // 2. Initialize GPIO control for speaker & headphone amplifier paths
    initGPIO();
    // Quirk 0x1a0 rev_order: silence both amp enables (active-low HIGH = OFF).
    silenceAmps();

    return (voodooNub != nullptr);
}

bool ES8336Codec::initVoodooNub() {
    if (voodooNub) return true;

    OSDictionary *match = IOService::serviceMatching("VoodooI2CDeviceNub");
    if (!match) return false;

    OSIterator *iter = IOService::getMatchingServices(match);
    match->release();
    if (!iter) return false;

    VoodooI2CDeviceNub *found = nullptr;
    IOService *svc = nullptr;
    while ((svc = OSDynamicCast(IOService, iter->getNextObject()))) {
        bool isEs8336 = false;
        const char *svcName = svc->getName();
        if (svcName && (strcmp(svcName, "ESSX") == 0 || strcmp(svcName, "ESSX8336") == 0)) {
            isEs8336 = true;
        }

        OSObject *nameProp = svc->getProperty("IOName");
        if (!nameProp) nameProp = svc->getProperty("name");
        if (nameProp) {
            if (OSString *str = OSDynamicCast(OSString, nameProp)) {
                if (str->isEqualTo("ESSX8336") || str->isEqualTo("ESSX")) isEs8336 = true;
            } else if (OSData *data = OSDynamicCast(OSData, nameProp)) {
                const char *bytes = (const char *)data->getBytesNoCopy();
                if (bytes && data->getLength() >= 4 && strncmp(bytes, "ESSX", 4) == 0) isEs8336 = true;
            }
        }

        OSObject *compProp = svc->getProperty("compatible");
        if (compProp) {
            if (OSString *str = OSDynamicCast(OSString, compProp)) {
                if (str->isEqualTo("ESSX8336") || str->isEqualTo("ESSX")) isEs8336 = true;
            } else if (OSData *data = OSDynamicCast(OSData, compProp)) {
                const char *bytes = (const char *)data->getBytesNoCopy();
                if (bytes && data->getLength() >= 4 && strncmp(bytes, "ESSX", 4) == 0) isEs8336 = true;
            }
        }

        OSNumber *addr = OSDynamicCast(OSNumber, svc->getProperty("i2cAddress"));
        if (addr && addr->unsigned16BitValue() == ES8316_I2C_ADDR) isEs8336 = true;

        if (isEs8336) {
            found = (VoodooI2CDeviceNub *)svc;
            found->retain();
            break;
        }
    }
    iter->release();

    if (found) {
        voodooNub = found;
        return true;
    }
    return false;
}

bool ES8336Codec::initGPIO() {
    if (gpioAmpBase && gpioBase) return true;

    // Search for GPI0 ACPI platform device (INT34BB on Comet Lake PCH-LP)
    OSDictionary *match = IOService::serviceMatching("IOACPIPlatformDevice");
    if (!match) return false;

    OSIterator *iter = IOService::getMatchingServices(match);
    match->release();
    if (!iter) return false;

    IOService *svc = nullptr;
    while ((svc = OSDynamicCast(IOService, iter->getNextObject()))) {
        bool isGpi = false;
        const char *svcName = svc->getName();
        if (svcName && (strcmp(svcName, "GPI0") == 0 || strcmp(svcName, "INT34BB") == 0)) {
            isGpi = true;
        }

        OSObject *nameProp = svc->getProperty("name");
        if (nameProp) {
            if (OSString *str = OSDynamicCast(OSString, nameProp)) {
                if (str->isEqualTo("GPI0") || str->isEqualTo("INT34BB")) isGpi = true;
            } else if (OSData *data = OSDynamicCast(OSData, nameProp)) {
                const char *bytes = (const char *)data->getBytesNoCopy();
                unsigned int len = data->getLength();
                if (bytes && len >= 7 && strncmp(bytes, "INT34BB", 7) == 0) isGpi = true;
                if (bytes && len >= 4 && strncmp(bytes, "GPI0", 4) == 0) isGpi = true;
            }
        }

        if (isGpi) {
            /* Index 0 = Community 0 @ 0xFD6E0000 (GPP_B15/B16 amp enables).
             * Index 1 = Community 1 @ 0xFD6D0000. */
            gpioMap = svc->mapDeviceMemoryWithIndex(0);
            if (gpioMap) {
                gpioBase = (volatile UInt8 *)gpioMap->getVirtualAddress();
                if (!gpioBase) {
                    gpioMap->release();
                    gpioMap = nullptr;
                } else {
                    IOLog("MateBookAudio: GPIO Community 0 mapped from ACPI %s at %p\n",
                          svcName ? svcName : "GPI0", gpioBase);
                }
            }
            gpioAmpMap = svc->mapDeviceMemoryWithIndex(1);
            if (gpioAmpMap) {
                gpioAmpBase = (volatile UInt8 *)gpioAmpMap->getVirtualAddress();
                if (!gpioAmpBase) {
                    gpioAmpMap->release();
                    gpioAmpMap = nullptr;
                } else {
                    IOLog("MateBookAudio: GPIO Community 1 (amp) mapped from ACPI %s at %p\n",
                          svcName ? svcName : "GPI0", gpioAmpBase);
                }
            }
            if (gpioBase || gpioAmpBase)
                break;
        }
    }
    iter->release();

    if (!gpioBase) {
        IOMemoryDescriptor *desc = IOMemoryDescriptor::withPhysicalAddress(0xFD6E0000ULL, 0x10000, kIODirectionInOut);
        if (desc) {
            gpioMap = desc->map();
            desc->release();
            if (gpioMap) {
                gpioBase = (volatile UInt8 *)gpioMap->getVirtualAddress();
                if (!gpioBase) { gpioMap->release(); gpioMap = nullptr; }
                else IOLog("MateBookAudio: GPIO Community 0 fallback 0xFD6E0000 at %p\n", gpioBase);
            }
        }
    }
    if (!gpioAmpBase) {
        IOMemoryDescriptor *desc = IOMemoryDescriptor::withPhysicalAddress(0xFD6D0000ULL, 0x10000, kIODirectionInOut);
        if (desc) {
            gpioAmpMap = desc->map();
            desc->release();
            if (gpioAmpMap) {
                gpioAmpBase = (volatile UInt8 *)gpioAmpMap->getVirtualAddress();
                if (!gpioAmpBase) { gpioAmpMap->release(); gpioAmpMap = nullptr; }
                else IOLog("MateBookAudio: GPIO Community 1 fallback 0xFD6D0000 at %p\n", gpioAmpBase);
            }
        }
    }

    if (!gpioAmpBase)
        IOLog("MateBookAudio: GPIO Community 1 not mapped (amps are on C0, this is OK)\n");
    if (!gpioBase)
        IOLog("MateBookAudio: FAILED to map GPIO Community 0\n");

    return (gpioAmpBase != nullptr || gpioBase != nullptr);
}

IOByteCount ES8336Codec::getGpioLength() const {
    if (!gpioMap) return 0;
    return gpioMap->getLength();
}

IOByteCount ES8336Codec::getGpioAmpLength() const {
    if (!gpioAmpMap) return 0;
    return gpioAmpMap->getLength();
}

UInt32 ES8336Codec::ampPadcfgOffset(UInt32 pin) {
    /* PADBAR lives at community window + 0x0C and is the offset of pad 0.
     * Community 0 starts at pin 0, so the pin number is also the pad index. */
    UInt32 padbar = HUAWEI_PADBAR_FALLBACK;
    if (gpioBase) {
        UInt32 v = rd32(gpioBase, HUAWEI_PADBAR_REG) & 0xFFFFU;
        if (v >= 0x100U && v < 0x8000U) padbar = v;
    }
    return padbar + pin * HUAWEI_PAD_STRIDE;
}

void ES8336Codec::setAmpPadcfg(UInt32 pin, bool high) {
    /* Community 0 (0xFD6E0000), where GPP_B actually lives. */
    if (!gpioBase) return;
    if (pin != HUAWEI_AMP_PAD_PIN_SPK && pin != HUAWEI_AMP_PAD_PIN_HP) return;

    UInt32 padcfg0 = ampPadcfgOffset(pin);
    if ((padcfg0 + 4) > getGpioLength()) return;

    /* TXSTATE only (bit 0), like Linux intel_gpio_set. PADCFGLOCK is set on
     * these pads so the rest of PADCFG0 must not be touched; PADCFGLOCKTX is
     * not set, so bit 0 takes. HOSTSW_OWN is deliberately left as firmware set
     * it (Linux keeps these pads ACPI-owned and drives them anyway). */
    UInt32 oldVal = rd32(gpioBase, padcfg0);
    UInt32 val = (oldVal & ~1U) | (high ? 1U : 0U);
    wr32(gpioBase, padcfg0, val);
    UInt32 verify = rd32(gpioBase, padcfg0);
    IOLog("MateBookAudio: amp pad C0 pin=%u @0x%x on=%d: 0x%08x -> wr 0x%08x rd 0x%08x (TXDIS=%u)\n",
          pin, padcfg0, high ? 1 : 0, oldVal, val, verify, (verify >> 8) & 1U);
}

void ES8336Codec::setGpioPin(UInt32 pin, bool high) {
    /* Only the two amp enables, GPP_B15/B16 in community 0. */
    if (pin == HUAWEI_GPIO_HP_EN)
        setAmpPadcfg(HUAWEI_AMP_PAD_PIN_HP, high);
    else if (pin == HUAWEI_GPIO_SPK_EN)
        setAmpPadcfg(HUAWEI_AMP_PAD_PIN_SPK, high);
}

bool ES8336Codec::writeReg(UInt8 reg, UInt8 val) {
    if (voodooNub) {
        UInt8 buf[2] = { reg, val };
        IOReturn ret = voodooNub->writeI2C(buf, 2);
        return (ret == kIOReturnSuccess);
    }
    return false;
}

UInt8 ES8336Codec::readReg(UInt8 reg) {
    if (voodooNub) {
        UInt8 val = 0;
        IOReturn ret = voodooNub->writeReadI2C(&reg, 1, &val, 1);
        if (ret == kIOReturnSuccess) {
            return val;
        }
        return 0;
    }
    return 0;
}

bool ES8336Codec::probe() {
    UInt8 vmid = readReg(ES8316_SYS_VMIDSEL);
    IOLog("MateBookAudio: Probing ES8336 at 0x10, VMID read=0x%02x\n", vmid);
    return (voodooNub != nullptr);
}

bool ES8336Codec::configure() {
    if (!voodooNub) {
        if (!initVoodooNub()) return false;
    }

    IOLog("MateBookAudio: Starting ES8336 initialization sequence\n");

    // Step 1: Reset codec and enable state machine (from Linux snd_soc_es8316.c:780-783)
    writeReg(ES8316_RESET, 0x3F);
    IOSleep(10);
    writeReg(ES8316_RESET, ES8316_RESET_CSM_ON);
    IOSleep(35);

    // Step 2: System VMID and Reference (Linux: needed otherwise audio output is silent)
    writeReg(ES8316_SYS_VMIDSEL, 0xFF);
    writeReg(ES8316_CLKMGR_ADCOSR, 0x32);

    // Step 3: Clock Management — match Linux FULL3 playing (CLKSW=0x77).
    // sof_es8336 forces codec sysclk math at 19.2 MHz; SSP still runs 24 MHz MCLK.
    writeReg(ES8316_CLKMGR_CLKSW, 0x77);

    // Step 3b: LRCK divider = 19200000/48000 = 400 = 0x0190 (live mega ADCDIV1=0x11)
    writeReg(ES8316_CLKMGR_CLKSEL,  0x00);
    writeReg(ES8316_CLKMGR_ADCDIV1, 0x11);
    writeReg(ES8316_CLKMGR_ADCDIV2, 0x90);
    writeReg(ES8316_CLKMGR_DACDIV1, 0x11);
    writeReg(ES8316_CLKMGR_DACDIV2, 0x90);
    writeReg(ES8316_CLKMGR_CPDIV,   0x00);

    // Step 4: I2S slave 24-bit — mega1 playing-speakers SERDATA (LEN_24).
    // Old LEN_16 (0x0C) with SSP 24/25 scrambled the bit window → air on SPK+HP.
    writeReg(ES8316_SERDATA1,    0x04);  /* slave, bclk_div 4 */
    writeReg(ES8316_SERDATA_ADC, 0x40);
    writeReg(ES8316_SERDATA_DAC, 0x00);  /* I2S, LEN_24 */

    // Step 5: Power up analog references
    // SYS_PDN bits are DAPM-inverted: 0x3F = full power-down (Linux idle).
    // Linux paplay headphones uses 0x06 (DAC Vref + Bias + Analog on).
    writeReg(ES8316_SYS_PDN, sysPdn());
    writeReg(ES8316_SYS_LP1, 0x00);
    writeReg(ES8316_SYS_LP2, 0x00);
    writeReg(ES8316_SYS_REF, 0x00);
    writeReg(ES8316_HPMIX_PDN, 0x00);

    // Step 6: DAC powered — soft-muted initially for anti-pop
    // Ref: linux_ref/es8316.c:es8316_mute() L561: ES8316_DAC_SET1 bit5 = DAC mute
    writeReg(ES8316_DAC_PDN,  0x00);
    writeReg(ES8316_DAC_SET1, 0x20);  // DAC muted during route setup
    writeReg(ES8316_DAC_SET2, 0x00);
    writeReg(ES8316_DAC_SET3, 0x00);
    writeReg(ES8316_DAC_VOLL, 0x00);  // 0 dB (Linux 0x17 was −11.5; too quiet once format OK)
    writeReg(ES8316_DAC_VOLR, 0x00);

    // Step 7: Headphone Mixer & Driver Routing
    writeReg(ES8316_HPMIX_SEL,    0x00);
    writeReg(ES8316_HPMIX_SWITCH, 0x88);    // Bit 7: LDAC, Bit 3: RDAC
    writeReg(ES8316_HPMIX_PDN,    0x00);
    /* HPMIX 0xBB = 0 dB. Linux FULL3 used 0x00 (−12 dB) for headroom; with
     * HOST S32 / DAI S24 that path was ~24 dB too quiet (clean but tiny). */
    writeReg(ES8316_HPMIX_VOL,    0xBB);

    // Step 8: Charge Pump & Output Driver
    writeReg(ES8316_CPHP_PDN1,     0x00);
    writeReg(ES8316_CPHP_PDN2,     0x00);
    writeReg(ES8316_CPHP_LDOCTL,   0x00);
    writeReg(ES8316_CPHP_ICAL_VOL, 0x00); // MANDATORY: pdn_Lical=0, pdn_Rical=0
    writeReg(ES8316_CPHP_OUTEN,    0x66); // L+R CP + L+R Driver enabled

    // Step 9: Jack detection (one-shot — no aggressive I2C poll, no jack timer)
    writeReg(ES8316_GPIO_DEBOUNCE, 0x02);

    // Step 10: Anti-pop GPIO sequencing — DAC unmute FIRST, GPIO amp AFTER 70ms
    // FIX: linux_ref/sof_es8336.c:pcm_pop_work_events() L120 +
    //      sof_es8316_speaker_power_event() L166: msecs_to_jiffies(70)
    // PREVIOUS BUG: GPIO was toggled before DAC unmute → amp clamp / pop
    {
        bool hp = isHeadphoneInserted();
        mute(false);        // Step 10a: DAC live FIRST (ES8316_DAC_SET1 bit5 = 0)
        IOSleep(70);        // Step 10b: 70ms anti-pop (mirrors pcm_pop_work delay)
        setSpeakerEnable(!hp); // Step 10c: GPIO amp enabled LAST
    }

    IOLog("MateBookAudio: ES8336 configure done (I2S slave LEN_24 @19.2div/48k, speakers preferred, DAC unmuted)\n");
    return true;
}

void ES8336Codec::mute(bool muted) {
    /* Linux play: DAC_SET1=0x10 (soft-ramp / stream active). Mute bit5=0x20.
     * Do NOT use 0x00 for unmute — live dump never shows 0x00 while playing. */
    writeReg(ES8316_DAC_SET1, muted ? 0x20 : 0x10);
}

bool ES8336Codec::isHeadphoneInserted() {
    if (!voodooNub) return false; /* no I2C: prefer internal speakers */
    UInt8 flag = readReg(ES8316_GPIO_FLAG);
    /* Linux es8316: flags==0 means reset/error — do NOT treat as HP inserted
     * (that would keep SPK off forever). Default to speakers. */
    if (flag == 0x00) return false;
    /* Bit 2 (ES8316_GPIO_HP_NOT_INS): 0 = HP inserted, 1 = unplugged */
    return ((flag & ES8316_GPIO_HP_NOT_INS) == 0);
}

void ES8336Codec::setSpeakerEnable(bool enable) {
    /* Both pads together, exactly as Linux FULL3: speakers = both TXSTATE 1,
     * headphones = both 0. Jack is driven by the codec charge pump alone.
     * Do NOT change HPMIX/DAC here — a sticky HP attenuate left speakers
     * almost silent after jack/route flips. Keep 0 dB always; HP EQ later. */
    setGpioPin(HUAWEI_GPIO_SPK_EN, enable);
    setGpioPin(HUAWEI_GPIO_HP_EN, enable);

    writeReg(ES8316_CPHP_OUTEN, 0x66);
    writeReg(ES8316_SYS_PDN, sysPdn());
    writeReg(ES8316_HPMIX_VOL, 0xBB); /* 0 dB — validated loud speakers */
    writeReg(ES8316_DAC_VOLL, 0x00);
    writeReg(ES8316_DAC_VOLR, 0x00);
    writeReg(ES8316_DAC_SET1, 0x10);

    IOLog("MateBookAudio: Audio route -> %s (C0 GPP_B15+B16 TXSTATE=%d)\n",
          enable ? "Internal Speakers" : "Headphones", enable ? 1 : 0);
}

void ES8336Codec::silenceAmps() {
    /* TXSTATE 0 = amp off, which is also the state Linux leaves at idle. */
    setGpioPin(HUAWEI_GPIO_SPK_EN, false);
    setGpioPin(HUAWEI_GPIO_HP_EN, false);
    mute(true);
    IOLog("MateBookAudio: Amps silenced (C0 GPP_B15+B16 TXSTATE=0)\n");
}

void ES8336Codec::setHeadsetMic(bool on) {
    /* Linux es8316 DAPM sequence for "Headset Mic" -> MIC1 -> Mono ADC -> I2S OUT:
     *   CLKSW bit3 (ADC Clock), SYS_PDN bits 1/2/3/4/5 (ADC Vref, ADC bias, Bias,
     *   Analog, Mic Bias), ADC_PDN_LINSEL bits 7/6 (PGA, ADC) + lin1-rin1,
     *   SERDATA_ADC bit6 (I2S OUT) + LEN_24 (same width as the DAC / SSP0). */
    headsetMic = on;
    if (on) {
        writeReg(ES8316_CLKMGR_CLKSW, 0x7F);
        writeReg(ES8316_SYS_PDN, 0x00);
        writeReg(ES8316_ADC_PDN_LINSEL, 0x00);   /* PGA + ADC on, lin1-rin1 */
        writeReg(ES8316_ADC_PGAGAIN, 0x40);      /* +7 dB: loud voice without clipping the ADC */
        writeReg(ES8316_ADC_D2SEPGA, 0x00);
        /* Everest vendor sequence (Rockchip es8316 driver), missing from Linux: without
         * ADC_DMIC=0x08 (decimation mode + high-pass) the ADC outputs digital zero. */
        writeReg(0x50, 0xA0); writeReg(0x51, 0x00); writeReg(0x52, 0x00);
        writeReg(ES8316_SYS_VMIDLOW, 0x11);
        writeReg(ES8316_SYS_REF, 0x28);
        writeReg(ES8316_SYS_LP1, 0x04);
        writeReg(ES8316_SYS_LP2, 0x0C);
        writeReg(ES8316_ADC_DMIC, 0x08);
        writeReg(ES8316_ADC_MUTE, 0x00);
        writeReg(ES8316_ADC_VOLUME, 0x00);       /* 0 dB digital */
        writeReg(ES8316_SERDATA_ADC, 0x00);      /* I2S OUT on, I2S, 24 bits */
    } else {
        writeReg(ES8316_SERDATA_ADC, 0x40);      /* I2S OUT off */
        writeReg(ES8316_ADC_PDN_LINSEL, 0xC0);   /* PGA + ADC off */
        writeReg(ES8316_SYS_PDN, 0x06);
        writeReg(ES8316_CLKMGR_CLKSW, 0x77);
    }
    IOLog("MateBookAudio: headset mic ADC %s\n", on ? "ON (MIC1 lin1-rin1)" : "off");
}
