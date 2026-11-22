#!/usr/bin/env bash
# Install wr-mode on the Orin. Safe to re-run: edited profiles are kept.
set -euo pipefail
[[ $EUID -eq 0 ]] || exec sudo "$0" "$@"
modes_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

install -m 0755 "$modes_dir/wr-mode" /usr/local/sbin/wr-mode
install -d -m 0755 /etc/wr-mode/profiles
for modes_profile in "$modes_dir"/profiles/*.conf; do
  modes_dest=/etc/wr-mode/profiles/$(basename "$modes_profile")
  if [[ -e $modes_dest ]]; then
    echo "keeping existing $modes_dest"
  else
    install -m 0644 "$modes_profile" "$modes_dest"
  fi
done

# The way back in must exist before any profile can turn the radios off.
systemctl enable --now ssh >/dev/null
/usr/local/sbin/wr-mode baseline

# Recording hooks. wr-mapping.service runs the preflight itself; recorders
# started as a normal user call: sudo -n /usr/local/sbin/wr-mode ensure <profile>
groupadd -f wr-mode
if [[ -n ${SUDO_USER:-} && $SUDO_USER != root ]]; then
  usermod -aG wr-mode "$SUDO_USER"
  echo "added $SUDO_USER to group wr-mode (log in again to use it)"
fi
modes_sudoers=$(mktemp)
install -m 0440 "$modes_dir/wr-mode.sudoers" "$modes_sudoers"
visudo -cqf "$modes_sudoers" || { rm -f "$modes_sudoers"; echo "sudoers rule failed validation" >&2; exit 1; }
install -m 0440 -o root -g root "$modes_sudoers" /etc/sudoers.d/wr-mode
rm -f "$modes_sudoers"
install -d -m 0755 /etc/systemd/system/wr-mapping.service.d
install -m 0644 "$modes_dir/wr-mapping-mode.conf" /etc/systemd/system/wr-mapping.service.d/wr-mode.conf
systemctl daemon-reload

echo
/usr/local/sbin/wr-mode list
echo "next: sudo wr-mode apply field --dry-run"
