#!/bin/zsh
# Fingerprint unlock of the lock screen (the boot-time login window still needs the password).
#   sudo ./unlock.sh on [--pw]   (--pw: store the password again)
#   sudo ./unlock.sh off
set -e
[[ $EUID -eq 0 ]] || { echo "run it with sudo"; exit 1; }
D=${0:A:h}/..
U=${SUDO_USER:-$(stat -f %Su /dev/console)}
UID_=$(id -u $U)
AG=/Library/LaunchAgents/com.hackintosh.gxfp-agent.plist
DM=/Library/LaunchDaemons/com.hackintosh.gxfpd.plist
KC=/Library/Keychains/System.keychain
LA=/Library/LaunchAgents/com.hackintosh.gxfp-loginagent.plist   # older installs only

if [[ $1 == off ]]; then
  launchctl bootout gui/$UID_ $AG 2>/dev/null || true
  rm -f $AG $LA /usr/local/bin/gxfp-agent
  security delete-generic-password -s gxfp-unlock -a $U $KC >/dev/null 2>&1 || true
  echo "fingerprint unlock disabled (password removed from the keychain)"; exit 0
fi
[[ $1 == on ]] || { echo "usage: on [--pw] | off"; exit 1; }

# 1. driver + daemon
install -m 755 -o root -g wheel $D/gxfp/gxfp /usr/local/bin/gxfp
launchctl bootout system $DM 2>/dev/null || true
launchctl bootstrap system $DM
sed -i '' '/pam_gxfp.so/d' /etc/pam.d/screensaver

# 2. agent: replaced only if it changed (the Accessibility permission is tied to the binary)
NEWAGENT=0
if ! cmp -s $D/gxfp/gxfp-agent /usr/local/bin/gxfp-agent; then
  install -m 755 -o root -g wheel $D/gxfp/gxfp-agent /usr/local/bin/gxfp-agent
  NEWAGENT=1
fi

# 3. password (once, or with --pw)
if [[ $2 == --pw ]] || ! security find-generic-password -s gxfp-unlock -a $U $KC >/dev/null 2>&1; then
  /usr/local/bin/gxfp setpw $U
fi

# 4. user-session agent
install -m 644 -o root -g wheel $D/install/com.hackintosh.gxfp-agent.plist $AG
launchctl bootout gui/$UID_ $AG 2>/dev/null || true
launchctl bootstrap gui/$UID_ $AG

if (( NEWAGENT )); then
  echo
  echo "Accessibility (once): System Settings > Privacy & Security > Accessibility"
  echo "  -> remove any old \"gxfp\" entry"
  echo "  -> + -> Cmd+Shift+G -> /usr/local/bin/gxfp-agent -> enable"
  echo "  then: launchctl kickstart -k gui/$UID_/com.hackintosh.gxfp-agent"
  open "x-apple.systempreferences:com.apple.preference.security?Privacy_Accessibility" 2>/dev/null || true
fi
echo "ok"
