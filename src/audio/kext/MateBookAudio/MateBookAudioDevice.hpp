#ifndef MateBookAudioDevice_hpp
#define MateBookAudioDevice_hpp

#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include "ES8336Codec.hpp"

// UserClient method selectors
enum {
    kCmlSOF_StartPlayback  = 0,
    kCmlSOF_StopPlayback   = 1,
    kCmlSOF_GetPosition    = 2,
    kCmlSOF_UpdateSPIB     = 3,
    kCmlSOF_StartCapture   = 4,
    kCmlSOF_StopCapture    = 5,
    kCmlSOF_GetCapPosition = 6,
    kCmlSOF_TriggerInitDSP = 7,
    kCmlSOF_ReadBAR0_8     = 8,
    kCmlSOF_ReadBAR0_16    = 9,
    kCmlSOF_ReadBAR0_32    = 10,
    kCmlSOF_WriteBAR0_8    = 11,
    kCmlSOF_WriteBAR0_16   = 12,
    kCmlSOF_WriteBAR0_32   = 13,
    kCmlSOF_ReadBAR4_8     = 14,
    kCmlSOF_ReadBAR4_16    = 15,
    kCmlSOF_ReadBAR4_32    = 16,
    kCmlSOF_WriteBAR4_8    = 17,
    kCmlSOF_WriteBAR4_16   = 18,
    kCmlSOF_WriteBAR4_32   = 19,
    kCmlSOF_ReadBAR4_64    = 20,
    kCmlSOF_WriteBAR4_64   = 21,
    kCmlSOF_ReadGPIO32     = 22,
    kCmlSOF_WriteGPIO32    = 23,
    kCmlSOF_I2CWriteReg    = 24,
    kCmlSOF_I2CReadReg     = 25,
    kCmlSOF_GetPhysicalAddrs = 26,
    kCmlSOF_ForceResetDSP    = 27,
    kCmlSOF_InitDSPFromStaging = 28,
    kCmlSOF_GetFwPhys        = 29,
    kCmlSOF_SetFwStagingLen  = 30,
    kCmlSOF_SetCaptureSource = 31,  /* 0=DMIC PIPE3, 1=Headset MIC1/SSP0.IN */
    kCmlSOF_SetTuning        = 32,  /* id, value: headset capture tuning (before forced init) */
    kCmlSOF_MethodCount      = 33
};

// Shared buffer memory types for clientMemoryForType
enum {
    kCmlSOF_MemPlayback = 0,
    kCmlSOF_MemCapture  = 1,
    kCmlSOF_MemFlags    = 2,
    kCmlSOF_MemFirmware = 3   // persistent FW staging (~600KB)
};

#define kCmlSOF_FwStagingSize  (600U * 1024U)

#define kCmlSOF_FlagOff_ClamshellMuted  0

#define kCmlSOF_BufferFrames    16384
#define kCmlSOF_BytesPerFrame   8       // S32 stereo, 24-bit left-justified in container
#define kCmlSOF_BufferSize      (kCmlSOF_BufferFrames * kCmlSOF_BytesPerFrame)
/* Huawei quirk 0x1B0 / sof-cml-es8336-dmic2ch-ssp0: stereo DMIC (2ch S32) */
#define kCmlSOF_CapChannels     2
#define kCmlSOF_CapBytesPerFrame (kCmlSOF_CapChannels * 4)  // 8
#define kCmlSOF_CapBufferSize   (kCmlSOF_BufferFrames * kCmlSOF_CapBytesPerFrame)

class MateBookAudioUserClient;

class MateBookAudioDevice : public IOService {
    OSDeclareDefaultStructors(MateBookAudioDevice)
    friend class MateBookAudioUserClient;

public:
    bool init(OSDictionary *dict = nullptr) override;
    void free() override;
    IOService *probe(IOService *provider, SInt32 *score) override;
    bool start(IOService *provider) override;
    void stop(IOService *provider) override;

    IOReturn setPowerState(unsigned long powerStateOrdinal, IOService *whatDevice) override;

    // Playback
    IOReturn startPlayback();
    IOReturn stopPlayback();
    UInt32   getSamplePosition();
    IOReturn updateSPIB(UInt32 byteOffset);
    IOBufferMemoryDescriptor *getSharedBuffer() { return sharedDmaBuf; }

    // Capture
    IOReturn startCapture();
    IOReturn stopCapture();
    UInt32   getCapturePosition();
    IOReturn setCaptureSource(UInt32 source); /* 0=DMIC, 1=Headset */
    IOBufferMemoryDescriptor *getCaptureBuffer() { return capDmaBuf; }

    IOBufferMemoryDescriptor *getFlagsBuffer() { return flagsBuf; }
    IOBufferMemoryDescriptor *getFwStagingBuffer() { return fwStagingBuf; }

    // Hardware bridge (UserClient)
    IOReturn readBAR0(UInt32 width, UInt32 offset, UInt64 *outVal);
    IOReturn writeBAR0(UInt32 width, UInt32 offset, UInt64 value);
    IOReturn readBAR4(UInt32 width, UInt32 offset, UInt64 *outVal);
    IOReturn writeBAR4(UInt32 width, UInt32 offset, UInt64 value);
    IOReturn readGPIO32(UInt32 offset, UInt32 *outVal);
    IOReturn writeGPIO32(UInt32 offset, UInt32 value);
    IOReturn i2cWriteReg(UInt8 reg, UInt8 val);
    IOReturn i2cReadReg(UInt8 reg, UInt8 *outVal);
    IOReturn getPhysicalAddrs(UInt64 *playPhys, UInt64 *capPhys, UInt64 *flagsPhys);
    IOReturn getFwPhys(UInt64 *fwPhys);
    IOReturn forceResetDSP();
    IOReturn setFwStagingLen(UInt32 len);
    bool     initDSP(bool force = false, bool useStaging = false);

private:
    IOPCIDevice *pciDevice;
    IOMemoryMap *hdaBarMap;
    IOMemoryMap *dspBarMap;

    volatile UInt8 *hdaBase;
    volatile UInt8 *dspBase;
    UInt32 ppCap, spibCap, mlCap;
    int sIdx;                  // playback stream index
    UInt32 sTag, sd;           // playback stream tag & SD offset
    int capIdx;                // capture stream index
    UInt32 capTag, capSd;      // capture stream tag & SD offset
    UInt32 outboxOff;
    bool hwReady;
    bool isPlaying;
    bool isCapturing;
    bool codecConfigured;
    UInt32 activePlaybackHost;
    UInt32 captureSource;      // 0=DMIC PIPE3, 1=Headset SSP0.IN
    UInt32 capActiveHost;      // host comp of the running capture (PIPE3 or PIPE2)
    UInt32 tuning[8];          // 0: PIPE2 DAI frame (0=S24,1=S32) 1: PIPE2 host frame 2: SSP rx_slots
    IOReturn sspDaiConfig(UInt8 flags);
    UInt32 activeCaptureHost;  // PIPE3_HOST_ID or PIPE2_HOST_ID
    UInt32 fwStagingLen;       // bytes valid in fwStagingBuf (0 = empty)

    IOCommandGate     *commandGate;
    IOPMrootDomain    *rootDomain;
    IONotifier        *powerNotifier;

    static IOReturn    sPowerInterestHandler(void *target, void *refCon,
                                             UInt32 messageType, IOService *provider,
                                             void *messageArg, vm_size_t argSize);

    IOReturn setPowerStateGated(unsigned long powerStateOrdinal);
    IOReturn startPlaybackGated();
    IOReturn stopPlaybackGated();
    IOReturn startCaptureGated();
    IOReturn stopCaptureGated();
    IOReturn setCaptureSourceGated(UInt32 source);
    void     handleWillSleepGated();
    void     handleClamshellChangeGated(bool closed);
    void     fastMute();
    void     powerGateDSP();
    void     clearDspFailProps();
    bool     stagingHasPayload(UInt32 *outOffset, UInt32 *outSize) const;

    static IOReturn s_setPowerState(OSObject *o, void *a0, void *, void *, void *);
    static IOReturn s_startPlayback(OSObject *o, void *, void *, void *, void *);
    static IOReturn s_stopPlayback (OSObject *o, void *, void *, void *, void *);
    static IOReturn s_startCapture (OSObject *o, void *, void *, void *, void *);
    static IOReturn s_stopCapture  (OSObject *o, void *, void *, void *, void *);
    static IOReturn s_setCaptureSource(OSObject *o, void *src, void *, void *, void *);
    static IOReturn s_handleWillSleep(OSObject *o, void *, void *, void *, void *);
    static IOReturn s_handleClamshellChange(OSObject *o, void *closed, void *, void *, void *);

    bool allocateDmaBuffers();
    bool allocateFwStaging();
    void shutdownDSPGated();
    void shutdownForSleepGated();

    IOReturn mmioRead(volatile UInt8 *base, IOMemoryMap *map, UInt32 width,
                      UInt32 offset, UInt64 *outVal);
    IOReturn mmioWrite(volatile UInt8 *base, IOMemoryMap *map, UInt32 width,
                       UInt32 offset, UInt64 value);

    // Everest ES8336 codec instance
    ES8336Codec codec;

    IOBufferMemoryDescriptor *sharedDmaBuf;
    IOBufferMemoryDescriptor *sharedBdlBuf;

    IOBufferMemoryDescriptor *capDmaBuf;
    IOBufferMemoryDescriptor *capBdlBuf;

    IOBufferMemoryDescriptor *flagsBuf;
    IOBufferMemoryDescriptor *fwStagingBuf;
};

class MateBookAudioUserClient : public IOUserClient {
    OSDeclareDefaultStructors(MateBookAudioUserClient)
public:
    bool initWithTask(task_t owningTask, void *securityToken, UInt32 type) override;
    bool start(IOService *provider) override;
    void stop(IOService *provider) override;
    IOReturn clientClose() override;
    IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options,
                                 IOMemoryDescriptor **memory) override;
    IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *arguments,
                           IOExternalMethodDispatch *dispatch, OSObject *target,
                           void *reference) override;
private:
    MateBookAudioDevice *device;
    task_t clientTask;
    static IOReturn sStart(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sStop(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sGetPos(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sUpdateSPIB(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sStartCap(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sStopCap(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sGetCapPos(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sTriggerInitDSP(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sReadBAR0_8(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sReadBAR0_16(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sReadBAR0_32(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sWriteBAR0_8(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sWriteBAR0_16(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sWriteBAR0_32(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sReadBAR4_8(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sReadBAR4_16(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sReadBAR4_32(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sWriteBAR4_8(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sWriteBAR4_16(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sWriteBAR4_32(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sReadBAR4_64(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sWriteBAR4_64(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sReadGPIO32(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sWriteGPIO32(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sI2CWriteReg(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sI2CReadReg(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sGetPhysicalAddrs(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sForceResetDSP(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sInitDSPFromStaging(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sGetFwPhys(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sSetFwStagingLen(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sSetCaptureSource(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static IOReturn sSetTuning(MateBookAudioUserClient *t, void *r, IOExternalMethodArguments *a);
    static const IOExternalMethodDispatch sMethods[kCmlSOF_MethodCount];
};

#endif /* MateBookAudioDevice_hpp */
