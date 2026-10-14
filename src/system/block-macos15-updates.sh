#!/bin/bash
# Optional: keep this Sonoma EFI on macOS 14. Hides the macOS 15+ upgrade (Sonoma-only catalog,
# no automatic checks/downloads). Sonoma updates still show up when you check manually
# (System Settings > Software Update) — do that regularly to get security fixes.
# Install: sudo install -m 755 block-macos15-updates.sh /usr/local/bin/ and load the LaunchDaemon plist.
set -e
CATALOG='https://swscan.apple.com/content/catalogs/others/index-14-13-12-10.16-10.15-10.14-10.13-10.12-10.11-10.10-10.9-mountainlion-lion-snowleopard-leopard.merged-1.sucatalog'
PLIST=/Library/Preferences/com.apple.SoftwareUpdate

/usr/bin/defaults write "$PLIST" AutomaticCheckEnabled -bool false
/usr/bin/defaults write "$PLIST" AutomaticDownload -bool false
/usr/bin/defaults write "$PLIST" AutomaticallyInstallMacOSUpdates -bool false
/usr/bin/defaults write "$PLIST" AutomaticallyInstallAppUpdates -bool false
/usr/bin/defaults write "$PLIST" CatalogURL "$CATALOG"
/usr/bin/defaults write "$PLIST" forceDelayedMajorSoftwareUpdates -bool true
/usr/bin/defaults write "$PLIST" MajorOSManagedDeferredInstallDelay -int 90
/usr/bin/defaults write "$PLIST" MajorOSUserNotificationDate -date "2035-01-01 00:00:00 +0000"
/usr/bin/defaults write "$PLIST" UserNotificationDate -date "2035-01-01 00:00:00 +0000"

/usr/sbin/softwareupdate --schedule off >/dev/null 2>&1 || true
/usr/bin/killall softwareupdated >/dev/null 2>&1 || true
exit 0
