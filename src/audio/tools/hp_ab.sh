#!/bin/zsh
# Headphone A/B test: switches ES8336 codec settings live while music is
# playing; the listener rates each variant. Results -> $TMPDIR/hp_ab.log
# Everything is ephemeral (I2C): a reboot or `unmute-speakers` restores the kext state.
CTL="sudo $(dirname "$0")/matebook_ctl"
LOG=${TMPDIR:-/tmp}/hp_ab.log
w(){ $CTL wri2c $1 $2 >/dev/null 2>&1; }
r(){ $CTL rdi2c $1 2>/dev/null | grep -oE '0x[0-9a-f]+$'; }

flag=$(r 0x4f)
if (( (flag & 0x04) != 0 )); then
  echo "Headphones not detected (GPIO_FLAG=$flag). Plug them in and run again."; exit 1
fi

$CTL unmute-headphones >/dev/null 2>&1
# Linux FULL3 baseline: reference + charge pump (zeroed by the kext)
base(){
  w 0x12 0x28; w 0x19 0x02; w 0x1a 0x02; w 0x1b 0x03; w 0x1e 0x99; w 0x1f 0x90
  w 0x30 0x10; w 0x31 0x00; w 0x32 0x00
  w 0x16 0xbb; w 0x33 0x00; w 0x34 0x00
}

typeset -a NAMES CMDS
NAMES+=("A  current (HPMIX 0dB, DAC 0dB)");            CMDS+=(":")
NAMES+=("B  exact Linux copy (HPMIX -12, DAC -11.5)"); CMDS+=("w 0x16 0x00; w 0x33 0x17; w 0x34 0x17")
NAMES+=("C  HPMIX -12 dB, DAC 0 dB");                  CMDS+=("w 0x16 0x00")
NAMES+=("D  DAC -11.5 dB, HPMIX 0 dB");                CMDS+=("w 0x33 0x17; w 0x34 0x17")
NAMES+=("E  RIGHT polarity inverted");                 CMDS+=("w 0x30 0x11")
NAMES+=("F  LEFT polarity inverted");                  CMDS+=("w 0x30 0x12")
NAMES+=("G  mono (L+R mixed)");                        CMDS+=("w 0x32 0x08")
NAMES+=("H  left only in both ears");                  CMDS+=("w 0x30 0x50")
NAMES+=("I  right only in both ears");                 CMDS+=("w 0x30 0x90")

echo "=== HP A/B $(date) ===" >> $LOG
echo "Play music WITH vocals (MateBook output), headphones on."
echo "For each variant: listen, then type a 0-5 score + a word (e.g. '4 vocals ok') and press Enter."
for i in {1..${#NAMES}}; do
  base; eval "${CMDS[$i]}"
  echo; echo ">>> ${NAMES[$i]}"
  read "note?score: "
  echo "${NAMES[$i]} | $note" >> $LOG
done
base
echo; echo "Done. Results in $LOG"
