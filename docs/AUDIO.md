# Audio — Huawei MateBook D15 (BOHB-WAX9)

Internal speakers, headphones, internal microphone and headset microphone on macOS.

| | |
|---|---|
| Audio path | Intel cAVS DSP running **Sound Open Firmware (SOF, IPC3)** → **SSP0 (I2S)** → **Everest ES8336** codec |
| Codec control | I2C, shared with the trackpad through VoodooI2C (`ESSX8336`) |
| Microphones | internal DMIC array (SOF PIPE3) + headset mic on codec **MIC1** (SSP0 RX, PIPE2) |
| macOS parts | `MateBookAudio.kext` (DSP + codec), CoreAudio HAL plugin `MateBookAudio.driver`, boot script |
| Linux reference | `sof_es8336` + `es8316` drivers, topology `sof-cml-es8336-dmic2ch-ssp0.tplg` |

There is no HDA codec on this machine, so AppleALC cannot help: the whole SOF path has to be
driven by hand. Everything below was reverse-engineered by mirroring what Linux does on the
same hardware (register dumps taken from an Ubuntu live session while audio was playing).

---

## 1. Hardware facts (what Linux does)

### 1.1 SSP0 format and clocks

| Parameter | Value |
|---|---|
| Host (front-end) format | `S32_LE` |
| DAI / SSP widget format | `S24_LE` |
| MCLK | 24 MHz (NHLT MCLK mask `0x1` → `mclk_id = 0`) |
| BCLK | 2.4 MHz = 48000 × 2 × 25 |
| Valid bits / slot width | 24 / 25 |
| Slots | 2 (L + R), I2S, codec is clock slave (`0x4001`, CBC_CFC) |

### 1.2 ES8336 registers during playback

| Reg | Name | Speakers | Headphones |
|---|---|---|---|
| 0x01 | CLKMGR_CLKSW | 0x77 | 0x77 |
| 0x06/07 | DACDIV | 0x11 / 0x90 | same |
| 0x09/0a/0b | SERDATA | 0x04 / 0x40 / **0x00 (LEN_24)** | same |
| 0x0d | SYS_PDN | 0x26 | 0x06 |
| 0x16 | HPMIX_VOL | 0x00 (−12 dB) | 0x00 |
| 0x17 | CPHP_OUTEN | 0x66 | 0x66 |
| 0x30 | DAC_SET1 | 0x10 | 0x10 |
| 0x33/34 | DAC_VOL | 0x17 (−11.5 dB) | 0x17 |
| 0x4f | GPIO_FLAG | 0x24 (jack out) | 0x20 (jack in) |

`sof_es8336` computes the codec dividers for 19.2 MHz while SSP0 actually runs at 24 MHz — it
still works, and the macOS driver keeps the same values.

### 1.3 Amplifier GPIOs (PCH community 0, pads GPP_B15 / GPP_B16)

| Mode | pin 40 | pin 41 |
|---|---|---|
| Speakers | TXSTATE **1** | TXSTATE **1** |
| Headphones | TXSTATE **0** | TXSTATE **0** |

Both pads always move together. Speakers and headphones share the codec HP outputs; the
speaker amplifier is switched by these GPIOs. PCR port `0x6E`, PADBAR `0x600` → padcfg
`0x880` / `0x890`.

### 1.4 Headset microphone

Linux quirk `SOC_ES8336_HEADSET_MIC1` applies to board `BOHB-WAX9-PCB-B2`: the headset mic is
on **MIC1** (lin1-rin1), captured through **SSP0 RX** (topology `PIPELINE.2.SSP0.IN`).
Mic presence is the `GM_NOT_SHORTED` bit (0x02) of GPIO_FLAG once MicBias is on
(1 = 4-pole headset, 0 = plain headphones).

---

## 2. macOS architecture

### 2.1 `MateBookAudio.kext`

- Loads the SOF firmware (`sof-cml.ri`) and topology into the cAVS DSP, then builds the
  pipelines over IPC3:
  - **PIPE1** playback: host `S32` → buffer → DAI SSP0 `S24`;
  - **PIPE2** headset capture: DAI SSP0 `S24` → buffer → host `S32` (same port, same
    DAI_CONFIG as playback);
  - **PIPE3** DMIC capture.
- Programs SSP0 with the Linux values (24 MHz / 2.4 MHz / 24-in-25 bits, `clks_control = 0`).
  SSP0 clocks are shared by playback and headset capture: `HW_PARAMS` / `HW_FREE` is only sent
  by whichever stream starts first / stops last.
- Configures the ES8336 over I2C (`ES8336Codec`), drives the amplifier GPIOs, exposes a
  UserClient used by the HAL plugin and by `matebook_ctl`.
- Headset mic (`ES8336Codec::setHeadsetMic`): ADC clock on, `SYS_PDN = 0x00` (ADC Vref/bias +
  MicBias), PGA + ADC on with lin1-rin1, PGA +7 dB, plus the Everest vendor ADC sequence
  (`0x50-0x52`, VMID, reference, low-power regs and **`ADC_DMIC = 0x08`** — without that
  register the ADC outputs digital zero; Linux never writes it), I2S OUT enabled.
- UserClient selectors added for the mic: `SetCaptureSource` (31: 0 = DMIC, 1 = headset)
  and `SetTuning` (32, debug).

### 2.2 HAL plugin `MateBookAudio.driver`

- CoreAudio AudioServerPlugIn, 48 kHz stereo, S32 container; the DMA ring buffers are shared
  with the kext (no copy through the UserClient in the IO path).
- **Output DSP** (`MateBookDSP.h`, header-only, no allocation or lock on the IO thread):
  - *Speakers profile*, calibrated with the internal microphone (§3): 120 Hz 4th-order HPF,
    virtual bass (harmonics of the < 150 Hz band in 150–500 Hz), EQ (−7 dB @ 820 Hz,
    +6 dB @ 2.9 kHz, +7 dB high shelf @ 6 kHz), +6 dB makeup, bass limiter (LR4 crossover
    @ 400 Hz) and a look-ahead peak limiter (ceiling −1 dBFS).
  - *Headphones profile*: transparent except for a −0.5 dBFS peak limiter (flat, like Linux).
- **Volume**: cubic curve like PipeWire (gain = s³, dB = 60·log10 s, −60…0 dB).
- **Jack handling**: a background thread polls GPIO_FLAG (0x4f) from plugin load, switches
  amps + DSP profile on hotplug (DAC muted 70 ms around the switch to avoid pops) and checks
  `GM_NOT_SHORTED` for a headset mic.
- **Data sources** (like a real Mac): output shows *Internal Speakers* or *Headphones*, input
  shows *Internal Microphone* or *External Microphone* — only the source that is actually
  connected is listed, and the UI follows the jack.
- **Microphone capture**: DMIC gain ×32; headset path ×3 after the PGA, with:
  - a **1-bit shift** of the raw SSP0 RX samples — the SSP latches one bit early, so the first
    received bit is garbage and the real MSB is the second one (measured: DC −1.0 → +0.005,
    1 kHz test tone reproduced at 1 kHz);
  - a DC-blocking high-pass (~77 Hz headset, ~8 Hz DMIC);
  - a soft limiter above 0.8 full scale instead of hard clipping.
- **Sleep / wake**: I2C paused on `WillSleep` (polling during sleep caused a VoodooI2C panic);
  on wake the DSP is re-initialised (`TriggerInitDSP`), running streams are restarted and the
  codec is restored without muting (no dropouts). Lid-open only does the soft restore; the
  clamshell mute flag is debounced by 150 ms.
- Real-time hygiene: helper threads run at `QOS_CLASS_BACKGROUND`, DSP state is `mlock`ed,
  no UserClient call per IO cycle.

### 2.3 Boot script `matebook_boot_init.sh`

Run once at boot by a LaunchDaemon: initialises the DSP firmware/topology, enables the right
output for the jack state (speakers if unplugged, headphones otherwise), renices `coreaudiod`
to −15 and finally restarts `coreaudiod` so the plugin re-detects jack and mic **after** the
script (the playback setup powers the ADC down).

---

## 3. Speaker calibration

Tool `src/audio/tools/measure/fr_measure.c`: plays 1/6-octave sines (50 Hz–16 kHz) on the
default output while recording the default input, aligns on the measured latency and measures
each fundamental (windowed Goertzel) + H2/H3 + noise floor into a CSV. `eqdesign.py` smooths the
curve (1/2 octave) and simulates candidate biquads.

Raw speaker response (repeatable ±1 dB above 110 Hz):

- +8 dB bump at 600–1000 Hz ("boxy" sound);
- −9 dB dip at 2.2–3.6 kHz (presence / voice);
- steep roll-off below 300 Hz, and H3 ≈ −10…−13 dB below 300 Hz at −3 dBFS = driver excursion
  distortion (mids stay clean, so not the amplifier);
- narrow notches at 4 / 6.4 / 9 kHz are microphone-position artefacts and are left alone.

Result with the DSP, closed-loop measurement:

| | Before | After |
|---|---|---|
| Std. deviation 350 Hz–6 kHz | ±5.7 dB | ±2.4 dB |
| H3 at 180–300 Hz, loud | −14 dB | −24…−35 dB |

Offline checks (`dsptest.c`): exact −1.00 / −0.50 dBFS ceilings on noise + 80 Hz at 0.9 FS, no
NaN, headphone profile transparent (max error 4.6e-10). Cost ≈ 0.3 % of one core.

Knobs (all in `MateBookDSP.h`): `MBDSP_SPK_MAKEUP_DB`, `MBDSP_SPK_VB_GAIN`,
`MBDSP_SPK_BASS_THR_DB`, `MBDSP_SPK_HPF_HZ`, EQ in `MBDSP_Init`.

Note: at the codec level macOS already runs at the hardware maximum (DAC 0 dB, HPMIX 0 dB).
Linux tests were done well below that (PipeWire ≈ −20 dB, DAC −8 dB, HPMIX −12 dB), and its
playback pipeline is flat — there is no Linux "calibration" to copy.

---

## 4. Bugs fixed along the way

| Symptom | Cause | Fix |
|---|---|---|
| Silence | `SYS_PDN = 0x3F`, amps off, DSP wedged | `SYS_PDN 0x06`, GPIO amps, DSP reset, trigger the host component only |
| IPC mailbox wedge (`0xffffffff`) | `TRIG_START` sent to DAI/scheduler components | trigger host only |
| No position updates | DPLBASE buffer at physical address 0 | proper DMA position buffer |
| Silence while DMA runs | host S16 vs 24-bit SSP | host S32 |
| "Air" / hiss instead of sound | host **and** DAI both S32, codec still LEN_16 | DAI **S24** + codec **LEN_24** + full-scale S32 packing |
| Wrong amp behaviour | wrong GPIO pads (community 1 offsets were the DMIC clock) | community 0 pads 0x880/0x890, both pads together |
| Speakers "dead" after a headphone test | headphone attenuation left applied | no sticky attenuation, routing by GPIO only |
| Muted after StartIO | `DAC_SET1 = 0x00` | `0x10` like Linux |
| No sound after sleep | DSP lost its firmware (`hwReady = false`) | plugin re-inits the DSP on wake |
| Panic on wake | jack I2C polling during sleep (VoodooI2C) | pause polling around sleep |
| Dropouts on lid blips | mute + re-route on every clamshell event | soft restore + 150 ms debounce |
| Headset mic silent | ADC clocks/reference off, I2S OUT disabled, `ADC_DMIC` never set | full ADC sequence incl. vendor `0x25 = 0x08` |
| Headset mic saturated | SSP RX one bit early | 1-bit shift in the plugin |
| Headset mic clipping on loud voice | PGA +18 dB and ×4 digital gain | PGA +7 dB, ×3, soft limiter |
| Boot with headphones plugged: mic dead | boot script forced the speaker setup (ADC off) | jack-aware script + `coreaudiod` restart; jack thread starts at plugin load |
| Two copies of every source in Sound settings | both data-source items always "available" | only the connected item is listed |

---

## 5. Debugging

```bash
cd src/audio/tools && make matebook_ctl
sudo ./matebook_ctl status            # DSP / firmware / topology state
sudo ./matebook_ctl dump-play         # stream + codec state while playing
sudo ./matebook_ctl calibrate spk     # live registers vs. expected values (spk | hp)
sudo ./matebook_ctl playtone 750 5 spk
sudo ./matebook_ctl rdi2c 0x4f        # GPIO_FLAG: bit2 = 1 jack out, bit1 = headset mic
sudo ./matebook_ctl probe-dma 0.2
sudo ./matebook_ctl resetdsp && sudo ./matebook_ctl init force
log stream --predicate 'eventMessage CONTAINS "MateBookAudio"'
```

`hp_ab.sh` is an A/B helper that switches codec settings live while music plays (it was used
to prove that a "weird vocals" headphone problem was a broken headphone plug, not the driver).

A new kext needs a reboot (it is injected by OpenCore); the HAL plugin is reloaded by
restarting `coreaudiod` (`sudo killall coreaudiod`).

---

## 6. Lessons

1. Copy what Linux does on the live system (register dumps while playing) rather than old
   comments or documentation.
2. Host and DAI formats must follow the topology (`s32le` host, `s24le` DAI), not both S32.
3. Undocumented vendor registers matter: `ADC_DMIC = 0x08` is the difference between a
   working headset mic and digital silence.
4. Never leave a per-output setting applied when switching outputs.
5. Anything touching the shared I2C bus must stop before sleep.

## 7. Open items

- Port the Linux reset values of 0x12 / 0x19–0x1b / 0x1e–0x1f into
  `ES8336Codec::configure()` (no audible difference measured so far).
- The 1-bit SSP RX shift is handled in the plugin; fixing the SSP RX timing in the topology
  would be cleaner.
