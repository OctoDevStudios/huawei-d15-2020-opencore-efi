# Fingerprint reader — Huawei MateBook D15 (Goodix GXFP51A0)

The fingerprint sensor in the power button works on macOS for `sudo` and for unlocking the
lock screen. It is **not** Touch ID: there is no Secure Enclave, so Apple Pay, keychain
Touch ID prompts and the boot-time login window are out of reach.

## Hardware (from the DSDT and Linux dumps)

| | Value |
|---|---|
| Sensor | Goodix **GXFP51A0** (STM32F411 MCU, firmware `GF_ST411SEC_APP_*`) |
| ACPI | `\_SB.PCI0.SPI1.FPNT` — `_CRS`: SPI mode 0, 8 bits, CS0 active-low, 10 MHz; pins patched from GNVS (`GFPI`/`GFPS`) |
| Controller | Intel LPSS SPI #1, PCI `00:1e.3` `8086:02ab`, 4 KB BAR0 (SSP @0, LPSS private @0x200, CS_CTRL @0x224) |
| Reset | pin **189** `UART0_RXD`, active-HIGH (LOW = MCU running), padcfg `0xFD6A0680` |
| IRQ | pin **204** `UART2_CTSB`, level-triggered, padcfg `0xFD6A0770` |
| Sister platform | HONOR **BBR-WAX9** (szlukabence's notes): same pins 189/204 |
| Caveat | pin 41 is **not** the IRQ here (it is the audio amp GPP_B16) — different from the MateBook 13 |

References: [szlukabence/goodix-fingerprint-spi-linux](https://github.com/szlukabence/goodix-fingerprint-spi-linux)
(protocol, TLS-PSK), [GodsQuantum/Huawei-Matebook-13-Linux](https://github.com/GodsQuantum/Huawei-Matebook-13-Linux)
(libfprint GXFP51A0 driver by Benjamin Allègre / Sigfrodr).

## macOS architecture

```
power button ── SPI1 ── MateBookFP.kext ── gxfp (daemon gxfpd, root) ──┬── pam_gxfp.so  (sudo)
                        (BAR0, pads,         TLS-PSK, capture,          └── gxfp-agent   (lock screen)
                         PIO transfers)      matching, templates
```

1. **`MateBookFP.kext`** (`src/fingerprint/kext`) — minimal bridge injected by OpenCore. Root-only
   UserClient: BAR0 read/write (< 0x400), read pads 189/204, write only TXSTATE of the reset
   pad (with a PMODE/TXDIS safety check), full-duplex PIO SPI transfer with caller-supplied
   CS_CTRL values and a 64 KB shared buffer. No sensor logic in the kernel.
   v0.2: power management (no transfers while asleep, D0 restored on wake), all loops bounded,
   CS→clock setup delay done in the kernel.
2. **`gxfp`** (`src/fingerprint/gxfp`) — the libfprint goodix51a0 driver compiled for macOS with a
   mini-glib (`mac/glib.h`) and spidev/GPIO emulated through the kext (`mac/mac_hw.c`), linked
   against a static OpenSSL 3.3.2. The driver core is kept as-is (LGPL-2.1-or-later):
   `src/core_a.c`, `core_b.c`, `core_c.c` are excerpts of `goodix51a0.c`.
   Commands: `init`, `capture`, `enroll`, `verify`, `list`, `delete`, `setpw`, `daemon`.
3. **`gxfpd`** (`gxfp daemon`, LaunchDaemon) — keeps the TLS session warm, serves
   `/var/run/gxfpd.sock` (root, used by PAM) and `/var/run/gxfpd-agent.sock` (lock screen agent).
   Idle = no sensor activity; while the screen is locked the finger is polled at 4 Hz.
4. **`pam_gxfp.so`** — `auth sufficient` module for `sudo` (falls through to the password on
   no match / timeout, skipped over SSH).
5. **`gxfp-agent`** (LaunchAgent, user session) — macOS does not run PAM for an empty
   password on the lock screen, so the agent watches the lock state; on a match, the daemon
   hands it the login password stored in the **System keychain** (root-only) and the agent
   types it into loginwindow (needs the Accessibility permission). The daemon only answers if
   the client is the installed `/usr/local/bin/gxfp-agent`, run by the console user, while
   that user's screen is locked.

> The stored password makes this a convenience feature, not a security boundary: anyone with
> root on the machine can read it. Do not use it if that matters to you.

## Install

```bash
cd src/fingerprint/gxfp && ./build-openssl.sh && make all         # gxfp + gxfp-agent
cd ../pam && clang -O2 -bundle -o pam_gxfp.so pam_gxfp.c -lpam && codesign -s - -f pam_gxfp.so
sudo ../install/install.sh                                         # gxfpd daemon + PAM for sudo
sudo /usr/local/bin/gxfp enroll $USER                              # 20 presses
sudo ../install/unlock.sh on                                       # lock screen unlock
```

Then allow `/usr/local/bin/gxfp-agent` in **System Settings → Privacy & Security → Accessibility**.

- Extra fingers: `sudo gxfp enroll $USER-2` (all `<user>*.gxt` templates are used).
- Uninstall: `sudo ../install/unlock.sh off` and `sudo ../install/install.sh uninstall`.
- Logs: `/var/log/gxfpd.log`, `/tmp/gxfp-agent.log`.
- `install/screensaver.sh` is an older alternative using the loginwindow `screensaver` PAM stack
  (empty password + Return, then touch the sensor). `unlock.sh` is preferred.

Templates (`*.gxt`) and the sensor key cache live in `/var/db/gxfp` (root, 0700) and are never
part of this repository.

## Debugging

```bash
cd src/fingerprint/tools && clang -O2 -framework IOKit -o fpctl fpctl.c
sudo ./fpctl info      # kext loaded? BAR0, SSCR0, CS_CTRL, pads
sudo ./fpctl probe     # first contact: cases A and E must answer
sudo gxfp -v init      # reset + PMK + TLS + calibration
```

Expected `probe` case A: `a0 1a 00 ba a8 17 00 "GF_ST411SEC_APP_…"`; cases B/C/D silent.

## History

- **Kext not injected** — OpenCore: `Prelinked injection MateBookFP.kext - Invalid Parameter`.
  Missing `_kmod_info` symbol (`KextFindKmodAddress`) → `KMOD_EXPLICIT_DECL`.
- **First contact** — byte-for-byte identical to the reference:
  - A: `a0 1a 00 ba a8 17 00 "GF_ST411SEC_APP_14115"` (MCU firmware version)
  - E: ACK `a0 06 00 a6 b0 03 00 a8 03 4c` (status 0x03: commands 0–5 blocked until TLS)
  - B (classic CS), C (reset HIGH), D (no command): silent, as expected
  - Working config: LPSS CLK `0x80010003`, RESETS 7, SSCR0 `0x6387` (8 bits, SCR 99 ≈ 1 MHz),
    SSCR1 `0x1dc0` (mode 0), CS in software mode, **CS "high" during the transfer** (Linux
    `SPI_CS_HIGH`).
- **PMK** — read from the sensor's factory staging area; stable across resets.
- **TLS digest failures / "no-irq" retries** — the STM32 needs a **CS→clock setup delay**: CS is
  asserted, then 50 µs (`GXFP_CS_SETUP_US`) before clocking. Result: zero retries, TLS-PSK OK,
  background frame in 1.1 s.
- **100 % CPU after sleep (kext v0.1.0)** — after sleep the LPSS block reads `0xffffffff` and the
  RX drain loop never ended. v0.2.0: bounded loops, refuse transfers when the block is off,
  power management; `gxfp` reconfigures the SSP on every open and retries once (never busy-loops).
- **Lock screen** — PAM is not consulted for an empty password field on the lock screen, hence
  the agent + System keychain design above.
