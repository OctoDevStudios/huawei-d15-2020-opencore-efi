# OpenCore EFI — Huawei MateBook D15 2020 (BOHB-WAX9)

> **This repository has moved.** It is archived and kept only for the pull request to lastant's EFI.
> Use the new repositories instead:
> - **[huawei-matebook-d15-hackintosh](https://github.com/OctoDevStudios/huawei-matebook-d15-hackintosh)** — the EFI (with releases)
> - **[MateBookAudio](https://github.com/OctoDevStudios/MateBookAudio)** — Intel SOF + ES8336 audio driver
> - **[MateBookFP](https://github.com/OctoDevStudios/MateBookFP)** — Goodix GXFP51A0 fingerprint driver

Fork of [lastant/huawei-d15-2020-opencore-efi](https://github.com/lastant/huawei-d15-2020-opencore-efi)
(OpenCore 1.0.5, macOS Sonoma), cleaned up and extended with two custom drivers so that the
**internal audio** (speakers, headphones, both microphones) and the **fingerprint reader** work.

| | |
|---|---|
| Machine | Huawei MateBook D15 2020, i5-10210U (Comet Lake), board BOHB-WAX9 |
| macOS | Sonoma 14.8.x |
| OpenCore | 1.0.5 (`ocvalidate` clean) |
| SMBIOS | MacBookAir9,1 — **the serials in this repo are placeholders** |

## What works

| Feature | Status | How |
|---|---|---|
| Internal speakers | ✅ | `MateBookAudio.kext` (Intel SOF DSP + ES8336 codec) + CoreAudio plugin |
| Speaker tuning | ✅ | DSP in the plugin, calibrated by measurement (EQ, virtual bass, limiters) |
| Headphones (hot-plug) | ✅ | amp switching + flat profile; *Headphones* shown in Sound settings |
| Internal microphone | ✅ | DMIC through SOF |
| Headset microphone | ✅ | codec MIC1 through SSP0; *External Microphone* shown when plugged in |
| Fingerprint for `sudo` | ✅ | `MateBookFP.kext` + `gxfp` driver (Goodix GXFP51A0, TLS-PSK) + PAM |
| Fingerprint lock-screen unlock | ✅ | agent typing the login password (stored in the System keychain) |
| Sleep / wake | ✅ | audio DSP and fingerprint controller restored on wake |
| Intel Wi-Fi / Bluetooth | ✅ | AirportItlwm, IntelBluetoothFirmware |
| I2C trackpad, keyboard, brightness keys | ✅ | VoodooI2C, VoodooPS2, BrightnessKeys |
| Battery | ✅ | SMCBatteryManager |
| Fingerprint at the boot login window | ❌ | not supported (a real Mac also asks for the password after a restart) |
| Real Touch ID / Apple Pay | ❌ | impossible without a Secure Enclave |
| AirDrop / Handoff | ❌ | needs a Broadcom Wi-Fi card |

## Before using this EFI

1. **Generate your own SMBIOS** with [GenSMBIOS](https://github.com/corpnewt/GenSMBIOS) (model
   `MacBookAir9,1`) and fill `EFI/OC/config.plist` → `PlatformInfo > Generic`:
   `SystemSerialNumber`, `MLB`, `SystemUUID`, `ROM`. The values in the repo
   (`W00000000001`, zero UUID, …) are placeholders.
2. Change `itlwm_cc=FR` in `boot-args` to your own country code (Wi-Fi regulatory domain).
3. Validate: `ocvalidate EFI/OC/config.plist` (version 1.0.5).

## Changes from the original EFI

- **Cleanup**: only the kexts / SSDTs / drivers that are actually loaded are kept (82 MB → 35 MB).
  Removed: `itlwm`, `AppleALC` (useless here: there is no HDA codec), `RtWlanU*`, Ethernet,
  `USBInjectAll`/`USBMap`, unused SSDTs…
- **Added**: `MateBookAudio.kext`, `MateBookFP.kext`, `RestrictEvents.kext` (`revpatch=cpuname`).
- **About This Mac**: real CPU name, GPU shown as "Intel UHD Graphics 620", `ProcessorType` Core i5.
- `ResetNvramEntry.efi` moved to drivers; debug boot-args removed; `csr-active-config` = 0 (SIP on).

## macOS-side installation

The kexts are injected by OpenCore, but both features also need userspace components.
Requirement: Xcode Command Line Tools.

### Audio

```bash
sudo src/audio/install/install.sh
```

Builds and installs the HAL plugin (`/Library/Audio/Plug-Ins/HAL/MateBookAudio.driver`),
`matebook_ctl` and the boot-init LaunchDaemon, which initialises the DSP, selects speakers or
headphones from the jack state and restarts `coreaudiod`. Details: [docs/AUDIO.md](docs/AUDIO.md).

### Fingerprint

```bash
cd src/fingerprint/gxfp && ./build-openssl.sh && make all          # gxfp driver + agent
cd ../pam && clang -O2 -bundle -o pam_gxfp.so pam_gxfp.c -lpam && codesign -s - -f pam_gxfp.so
sudo ../install/install.sh                                          # gxfpd daemon (+ PAM for sudo)
sudo /usr/local/bin/gxfp enroll $USER                               # 20 presses
sudo ../install/unlock.sh on                                        # lock screen unlock
```

Then allow `/usr/local/bin/gxfp-agent` in **System Settings → Privacy & Security → Accessibility**.
Details, protocol and history: [docs/FINGERPRINT.md](docs/FINGERPRINT.md).

> ⚠️ Lock-screen unlock stores your login password in the **System** keychain (readable by root
> only). This is not Touch ID: it is a convenience, not a security boundary.

### Optional: stay on Sonoma

`src/system/block-macos15-updates.sh` (+ LaunchDaemon plist) hides the macOS 15+ upgrade by
pinning the Sonoma software-update catalog. Check for Sonoma security updates manually.

## Layout

```
EFI/                      ready-to-use OpenCore EFI (placeholder SMBIOS)
src/audio/kext/           MateBookAudio.kext — SOF IPC3, SSP0, ES8336 (I2C via VoodooI2C)
src/audio/plugin/         CoreAudio HAL plugin + output DSP (MateBookDSP.h)
src/audio/tools/          matebook_ctl (codec/DSP debugging), boot init script, measurement tools
src/audio/install/        audio installer + LaunchDaemon
src/fingerprint/kext/     MateBookFP.kext — LPSS SPI1 + sensor GPIO bridge
src/fingerprint/gxfp/     userspace driver (port of the libfprint goodix51a0 driver), daemon, agent
src/fingerprint/pam/      PAM module (sudo)
src/fingerprint/install/  install scripts, LaunchDaemon / LaunchAgent
src/fingerprint/tools/    fpctl (low-level SPI debugging)
src/system/               optional: stay on Sonoma
docs/                     technical write-ups (audio, fingerprint)
```

## Rebuilding the kexts

```bash
make -C src/audio/kext          # → src/audio/kext/MateBookAudio.kext
make -C src/fingerprint/kext    # → src/fingerprint/kext/MateBookFP.kext
```

Copy the bundle into `EFI/OC/Kexts/` and reboot (kexts are injected at boot; SIP stays on).

## Credits and licenses

- Base EFI: [lastant](https://github.com/lastant/huawei-d15-2020-opencore-efi). OpenCore and kexts:
  [Acidanthera](https://github.com/acidanthera), [VoodooI2C](https://github.com/VoodooI2C/VoodooI2C),
  [OpenIntelWireless](https://github.com/OpenIntelWireless).
- Audio: modelled on the Linux `sof_es8336` / `es8316` drivers and the
  [Sound Open Firmware](https://github.com/thesofproject) project. `sof-cml.ri` and the topology
  `sof-cml-es8336-dmic2ch-ssp0.tplg` come from SOF (`sof-bin`, BSD-3-Clause).
- Fingerprint: libfprint GXFP51A0 driver by **Benjamin Allègre (Sigfrodr) /
  [GodsQuantum](https://github.com/GodsQuantum/Huawei-Matebook-13-Linux)**, used in
  `src/fingerprint/gxfp/src/` under **LGPL-2.1-or-later** (see `COPYING.LGPL-2.1` and the SPDX
  headers); protocol and PSK method documented by
  [szlukabence](https://github.com/szlukabence/goodix-fingerprint-spi-linux); key derivation:
  lexakimov. The macOS adaptations of that code remain under the LGPL.
- The original code in this fork (MateBookAudio, MateBookFP, tools, docs) is under the
  [MIT license](LICENSE). That covers src/audio/ (except the SOF firmware), src/fingerprint/
  (except `src/fingerprint/gxfp/src/`, which is LGPL), src/system/ and docs/. Third-party
  components in `EFI/` keep their own licenses.
- No fingerprint template, sensor key or personal identifier is included in this repository.
