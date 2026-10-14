#!/bin/zsh
# Installs the macOS side of MateBook audio: HAL plugin, matebook_ctl, boot-init LaunchDaemon.
# The kext itself is injected by OpenCore (EFI/OC/Kexts/MateBookAudio.kext).
# Usage: sudo ./install.sh          (uninstall: sudo ./install.sh uninstall)
set -e
[[ $EUID -eq 0 ]] || { echo "run it with sudo"; exit 1; }
D=${0:A:h}/..
HAL=/Library/Audio/Plug-Ins/HAL/MateBookAudio.driver
LIB=/usr/local/libexec/matebook
PLIST=/Library/LaunchDaemons/com.hackintosh.matebookaudio.init.plist

if [[ $1 == uninstall ]]; then
  launchctl bootout system $PLIST 2>/dev/null || true
  rm -rf $PLIST $LIB $HAL
  killall coreaudiod 2>/dev/null || true
  echo "uninstalled"; exit 0
fi

U=${SUDO_USER:-root}
sudo -u $U make -C $D/plugin
sudo -u $U make -C $D/tools matebook_ctl

rm -rf $HAL
mkdir -p /Library/Audio/Plug-Ins/HAL
cp -R $D/plugin/MateBookAudio.driver $HAL
install -d -m 755 $LIB
install -m 755 -o root -g wheel $D/tools/matebook_ctl $D/tools/matebook_boot_init.sh $LIB/
install -m 644 -o root -g wheel $D/install/com.hackintosh.matebookaudio.init.plist $PLIST
launchctl bootout system $PLIST 2>/dev/null || true
launchctl bootstrap system $PLIST      # runs the init script now (it restarts coreaudiod)
echo "audio installed (log: /var/log/matebook_audio_init.log)"
