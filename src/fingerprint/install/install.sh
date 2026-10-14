#!/bin/zsh
# Installs GXFP51A0 fingerprint support: gxfpd daemon + PAM module, then enables it for sudo.
# Usage: sudo ./install.sh          (uninstall: sudo ./install.sh uninstall)
set -e
[[ $EUID -eq 0 ]] || { echo "run it with sudo"; exit 1; }
D=${0:A:h}/..
PLIST=/Library/LaunchDaemons/com.hackintosh.gxfpd.plist
LINE="auth       sufficient     /usr/local/lib/pam/pam_gxfp.so timeout=10"

if [[ $1 == uninstall ]]; then
  launchctl bootout system $PLIST 2>/dev/null || true
  rm -f $PLIST /usr/local/bin/gxfp /usr/local/lib/pam/pam_gxfp.so
  [[ -f /etc/pam.d/sudo_local ]] && sed -i '' '/pam_gxfp.so/d' /etc/pam.d/sudo_local
  echo "uninstalled (templates kept in /var/db/gxfp)"; exit 0
fi

install -d -m 755 /usr/local/bin /usr/local/lib/pam
install -m 755 -o root -g wheel $D/gxfp/gxfp /usr/local/bin/gxfp
install -m 444 -o root -g wheel $D/pam/pam_gxfp.so /usr/local/lib/pam/pam_gxfp.so
install -m 644 -o root -g wheel $D/install/com.hackintosh.gxfpd.plist $PLIST
launchctl bootout system $PLIST 2>/dev/null || true
launchctl bootstrap system $PLIST
echo "gxfpd daemon started (log: /var/log/gxfpd.log)"

# sudo: /etc/pam.d/sudo includes sudo_local (Sonoma+), which survives macOS updates
[[ -f /etc/pam.d/sudo_local ]] || cp /etc/pam.d/sudo_local.template /etc/pam.d/sudo_local
grep -q pam_gxfp.so /etc/pam.d/sudo_local || sed -i '' "1a\\
$LINE
" /etc/pam.d/sudo_local
echo "sudo: fingerprint enabled (password still works as a fallback)"
cat /etc/pam.d/sudo_local | grep -v '^#'
