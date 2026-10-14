#ifndef ES8336Codec_hpp
#define ES8336Codec_hpp

#include <IOKit/IOService.h>
#include <IOKit/pci/IOPCIDevice.h>

// Forward declaration of VoodooI2CDeviceNub
// NOTE: writeI2C, readI2C, and writeReadI2C are NON-VIRTUAL member functions in VoodooI2CDeviceNub.
// Declaring them as virtual in a forward declaration creates an invalid vtable layout that dispatches
// to random IOService methods (causing a fatal kernel panic). They must remain non-virtual.
class VoodooI2CDeviceNub : public IOService {
    OSDeclareDefaultStructors(VoodooI2CDeviceNub);
public:
    IOReturn writeI2C(UInt8 *values, UInt16 length);
    IOReturn readI2C(UInt8 *values, UInt16 length);
    IOReturn writeReadI2C(UInt8 *write_buffer, UInt16 write_length, UInt8 *read_buffer, UInt16 read_length);
};

// ES8316 / ES8336 Registers
#define ES8316_RESET            0x00
#define ES8316_CLKMGR_CLKSW     0x01
#define ES8316_CLKMGR_CLKSEL    0x02
#define ES8316_CLKMGR_ADCOSR    0x03
#define ES8316_CLKMGR_ADCDIV1   0x04
#define ES8316_CLKMGR_ADCDIV2   0x05
#define ES8316_CLKMGR_DACDIV1   0x06
#define ES8316_CLKMGR_DACDIV2   0x07
#define ES8316_CLKMGR_CPDIV     0x08
#define ES8316_SERDATA1         0x09
#define ES8316_SERDATA_ADC      0x0A
#define ES8316_SERDATA_DAC      0x0B
#define ES8316_SYS_VMIDSEL      0x0C
#define ES8316_SYS_PDN          0x0D
#define ES8316_SYS_LP1          0x0E
#define ES8316_SYS_LP2          0x0F
#define ES8316_SYS_VMIDLOW      0x10
#define ES8316_SYS_VSEL         0x11
#define ES8316_SYS_REF          0x12
#define ES8316_HPMIX_SEL        0x13
#define ES8316_HPMIX_SWITCH     0x14
#define ES8316_HPMIX_PDN        0x15
#define ES8316_HPMIX_VOL        0x16
#define ES8316_CPHP_OUTEN       0x17
#define ES8316_CPHP_ICAL_VOL    0x18
#define ES8316_CPHP_PDN1        0x19
#define ES8316_CPHP_PDN2        0x1A
#define ES8316_CPHP_LDOCTL      0x1B
#define ES8316_ADC_PDN_LINSEL   0x22
#define ES8316_ADC_PGAGAIN      0x23
#define ES8316_ADC_D2SEPGA      0x24
#define ES8316_ADC_DMIC         0x25
#define ES8316_ADC_MUTE         0x26
#define ES8316_ADC_VOLUME       0x27
#define ES8316_DAC_PDN          0x2F
#define ES8316_DAC_SET1         0x30
#define ES8316_DAC_SET2         0x31
#define ES8316_DAC_SET3         0x32
#define ES8316_DAC_VOLL         0x33
#define ES8316_DAC_VOLR         0x34
#define ES8316_GPIO_SEL         0x4D
#define ES8316_GPIO_DEBOUNCE    0x4E
#define ES8316_GPIO_FLAG        0x4F

// Bitfield definitions
#define ES8316_RESET_CSM_ON     0x80
#define ES8316_GPIO_HP_NOT_INS  0x04    // Bit 2: 1 = HP unplugged, 0 = HP inserted
#define ES8316_I2C_ADDR         0x10    // 7-bit I2C address (16 decimal)

/* Huawei MateBook D15 amp enables — DSDT ESSX._INI:
 *   PIN1 = GNUM(0x09000010) CRS idx0, PIN2 = GNUM(0x09000011) CRS idx1
 * GPCL[9]: PCR 0x006D0000 (= Community 1 @ 0xFD6D0000), PADBAR 0x0A50.
 * Padcfg = 0xA50 + local*0x10 → local16 @ 0xB50, local17 @ 0xB60.
 * Live Ubuntu (BOHB-WAX9-PCB-B2): quirk mask 0x1a0 = SSP0|DMIC|HP_GPIO|MIC1
 *   (NO SPEAKERS_EN_GPIO1) → acpi_enable_both_gpios_rev_order.
 *
 * Pad identity comes from the Ubuntu pinctrl dump taken on this exact machine
 * while a tone was confirmed audible (linux-live/mega1):
 *   gpio-559 "speakers-enable"  = pin 40 = GPP_B15 (GSPI0_CS0B)
 *   gpio-560 "headphone-enable" = pin 41 = GPP_B16 (GSPI0_CLK)
 * GPP_B is pins 25..50, which /proc/iomem places in community 0 (PCR port
 * 0x6E, 0xFD6E0000). The old C1 offsets 0xB50/0xB60 were pads 16/17 of
 * community 1, i.e. GPP_D16/D17 = ISH_UART0_CTSB and DMIC_CLK_1 — a digital
 * mic clock, not an amplifier.
 *
 * Polarity, from the same dump: PADCFG0 goes 0x44000200 -> 0x44000201 when the
 * speakers come up, so bit 0 (GPIOTXSTATE) 1 = amp ON. The "active low" in the
 * gpio debugfs listing is the ACPI polarity flag the consumer applies, not the
 * raw register level. Linux drives both pads to 1 for the internal speakers and
 * leaves both at 0 for headphones (which need no external amp).
 *
 * Both pads report [LOCKED, ACPI]: PADCFGLOCK is set but PADCFGLOCKTX is not,
 * so GPIOTXSTATE stays writable. HOSTSW_OWN is left alone — Linux keeps these
 * pads ACPI-owned and still drives them. */
#define HUAWEI_GPIO_SPK_EN      40
#define HUAWEI_GPIO_HP_EN       41
#define HUAWEI_AMP_PAD_PIN_SPK  40u    /* GPP_B15, community 0 */
#define HUAWEI_AMP_PAD_PIN_HP   41u    /* GPP_B16, community 0 */
#define HUAWEI_PAD_STRIDE       16u    /* 4 dwords per pad on CNL/CML */
#define HUAWEI_PADBAR_REG       0x0Cu  /* community window -> first padcfg */
#define HUAWEI_PADBAR_FALLBACK  0x0A50u

class ES8336Codec {
public:
    ES8336Codec();
    ~ES8336Codec();

    bool init();
    void free();

    bool writeReg(UInt8 reg, UInt8 val);
    UInt8 readReg(UInt8 reg);

    bool probe();
    bool configure();
    void mute(bool muted);
    bool isHeadphoneInserted();
    void setSpeakerEnable(bool enable);
    void silenceAmps();
    /* Headset mic (MIC1, Linux quirk SOC_ES8336_HEADSET_MIC1 for BOHB-WAX9-PCB-B2):
     * powers the whole ADC chain on/off. SYS_PDN depends on this state (0x00 / 0x06). */
    void setHeadsetMic(bool on);
    UInt8 sysPdn() const { return headsetMic ? 0x00 : 0x06; }

    bool isReady() const { return (voodooNub != nullptr); }
    const char *getStatus() const { return statusMsg; }
    volatile UInt8 *getGpioBase() const { return gpioBase; }
    IOByteCount getGpioLength() const;
    volatile UInt8 *getGpioAmpBase() const { return gpioAmpBase; }
    IOByteCount getGpioAmpLength() const;

private:
    VoodooI2CDeviceNub *voodooNub;
    IOMemoryMap        *gpioMap;
    volatile UInt8     *gpioBase;     /* Community 0 — do not use for amp enables */
    IOMemoryMap        *gpioAmpMap;
    volatile UInt8     *gpioAmpBase;  /* Community 1 — real HP/SPK amp pads */
    char                statusMsg[128];

    bool headsetMic;
    bool initVoodooNub();
    bool initGPIO();
    void setGpioPin(UInt32 pin, bool high);
    void setAmpPadcfg(UInt32 pin, bool high);
    UInt32 ampPadcfgOffset(UInt32 pin);
};

#endif /* ES8336Codec_hpp */
