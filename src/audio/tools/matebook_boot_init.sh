#!/bin/bash
# matebook_boot_init.sh — Auto-initialization of Comet Lake cAVS DSP & ES8336 codec
# IMPORTANT: ES8336 shares VoodooI2C with the trackpad. Never loop I2C. One-shot only.
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
CTL="$DIR/matebook_ctl"
LOCK=/tmp/matebook_boot_init.lock

# Single-flight: never pile concurrent I2C UserClient calls
if ! mkdir "$LOCK" 2>/dev/null; then
    echo "matebook_boot_init: already running, exit"
    exit 0
fi
trap 'rmdir "$LOCK" 2>/dev/null || true' EXIT

# Wait for MateBookAudioDevice to register in IORegistry (up to 30s)
for i in $(seq 1 60); do
    if ioreg -r -c MateBookAudioDevice 2>/dev/null | grep -q "MateBookAudioDevice"; then
        break
    fi
    sleep 0.5
done

# Initialize DSP firmware & topology (MMIO/IPC — avoid extra I2C here)
"$CTL" init 2>&1 || true

# Wait for DSP firmware entry (ROM_STATUS FSR=5) via status props only
for i in $(seq 1 30); do
    status=$("$CTL" status 2>&1) || true
    if echo "$status" | grep -q "ROM=0x05000001"; then
        break
    fi
    sleep 0.2
done

# One-shot codec path only (no loops). Shared I2C with trackpad.
# Jack (ES8336 GPIO_FLAG 0x4f, bit2 = 1 -> unplugged): no speaker amps when headphones are plugged.
flag=$("$CTL" rdi2c 0x4f 2>/dev/null | grep -oE '0x[0-9a-fA-F]+$' || echo 0x24)
if (( (flag & 0x04) != 0 )); then
    "$CTL" unmute-speakers 2>&1 || true
else
    "$CTL" unmute-headphones 2>&1 || true
fi

# Under memory pressure (8 GB): give coreaudiod a bit more priority to avoid dropouts.
# -15 stays below the real-time audio threads; better than the default nice.
for _ in 1 2 3 4 5 6 7 8 9 10; do
	pid=$(pgrep -x coreaudiod 2>/dev/null || true)
	if [ -n "$pid" ]; then
		renice -n -15 -p $pid >/dev/null 2>&1 || true
		break
	fi
	sleep 0.5
done

# The playback setup above powers down the ADC (headset mic): restart coreaudiod so
# the plugin redoes its jack / mic detection AFTER this script.
sleep 1
killall coreaudiod 2>/dev/null || true

exit 0
