#!/bin/zsh
# Alternative unlock method via the loginwindow "screensaver" PAM stack (unlock.sh is preferred).
#   sudo ./screensaver.sh on   |   sudo ./screensaver.sh off
# Usage: locked screen -> EMPTY password field -> Return -> finger on the power button.
# A typed password keeps working normally.
# RECOVERY if the lock screen rejects everything: reboot (the boot login does not use this file),
# then run "sudo ./screensaver.sh off" or restore /etc/pam.d/screensaver.bak-gxfp.
set -e
[[ $EUID -eq 0 ]] || { echo "run it with sudo"; exit 1; }
F=/etc/pam.d/screensaver
LINE="auth       sufficient     /usr/local/lib/pam/pam_gxfp.so timeout=10 skip_if_password"
install -m 444 -o root -g wheel ${0:A:h}/../pam/pam_gxfp.so /usr/local/lib/pam/pam_gxfp.so
case $1 in
  on)  [[ -f $F.bak-gxfp ]] || cp -p $F $F.bak-gxfp
       grep -q pam_gxfp.so $F || sed -i '' "1a\\
$LINE
" $F ;;
  off) sed -i '' '/pam_gxfp.so/d' $F ;;
  *) echo "usage: on | off"; exit 1 ;;
esac
grep -v '^#' $F
