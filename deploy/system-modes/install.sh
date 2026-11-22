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

echo
/usr/local/sbin/wr-mode list
echo "next: sudo wr-mode apply field --dry-run"
