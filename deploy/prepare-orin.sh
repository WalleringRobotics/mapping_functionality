#!/usr/bin/env bash
# Prepare a freshly flashed Jetson Orin as a mapping host. Everything else
# (ROS, drivers, Python, PyCOLMAP) lives in the image from deploy/build-image.sh.
#
#   sudo deploy/prepare-orin.sh [--user NAME] [--l4t VERSION] [--check]
#
# Idempotent. Never partitions, formats or mounts storage and never changes the
# power mode; it reports both. --check reports the host without changing it.
set -euo pipefail
prep_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Commissioned release (docs/jetson-setup.md). Other releases need requalification.
prep_l4t=36.5.0
prep_user="${SUDO_USER:-}"
prep_check=false
prep_storage=/mnt/nvme
while (($#)); do
  case "$1" in
    --user) prep_user="$2"; shift 2 ;;
    --l4t) prep_l4t="$2"; shift 2 ;;
    --storage) prep_storage="$2"; shift 2 ;;
    --check) prep_check=true; shift ;;
    --help) sed -n '2,9p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
prep_fail=0
ok() { printf 'ok    %s\n' "$*"; }
warn() { printf 'WARN  %s\n' "$*"; }
fail() { printf 'FAIL  %s\n' "$*"; prep_fail=1; }
apply() { if [[ "$prep_check" == true ]]; then printf 'todo  %s\n' "$1"; else printf 'do    %s\n' "$1"; shift; "$@"; fi; }

# --- Platform identity: refuse to adapt a host we have not qualified. --------
[[ "$prep_check" == true || $EUID -eq 0 ]] || { echo "Run with sudo (or use --check)" >&2; exit 2; }
[[ -n "$prep_user" && "$prep_user" != root ]] || { echo "Name the operator with --user" >&2; exit 2; }
id "$prep_user" >/dev/null 2>&1 || { echo "User $prep_user does not exist" >&2; exit 2; }
[[ "$(uname -m)" == aarch64 ]] || { echo "Not an aarch64 Jetson" >&2; exit 2; }
# shellcheck disable=SC1091
. /etc/os-release
[[ "$VERSION_ID" == 22.04 ]] || { echo "Ubuntu 22.04 required, found $VERSION_ID" >&2; exit 2; }
[[ -r /etc/nv_tegra_release ]] || { echo "/etc/nv_tegra_release is absent; not L4T" >&2; exit 2; }
prep_found="$(sed -nE 's/^# R([0-9]+) \(release\), REVISION: ([0-9.]+).*/\1.\2/p' /etc/nv_tegra_release)"
if [[ "$prep_found" == "$prep_l4t" ]]; then
  ok "L4T $prep_found, Ubuntu $VERSION_ID, $(tr -d '\0' < /proc/device-tree/model 2>/dev/null)"
else
  echo "L4T $prep_found found, $prep_l4t expected. Reflash, or pass --l4t $prep_found and requalify." >&2
  exit 2
fi

# --- Base tools and Docker Engine with buildx. --------------------------------
prep_apt_updated=false
apt_install() {
  [[ "$prep_apt_updated" == true ]] || { apt-get update; prep_apt_updated=true; }
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"
}
prep_missing=()
for prep_pkg in ca-certificates curl git gnupg usbutils; do
  dpkg-query -W -f='${Status}' "$prep_pkg" 2>/dev/null | grep -q 'ok installed' || prep_missing+=("$prep_pkg")
done
if ((${#prep_missing[@]})); then apply "install ${prep_missing[*]}" apt_install "${prep_missing[@]}"; else ok "base tools"; fi

install_docker_ce() {
  install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  chmod a+r /etc/apt/keyrings/docker.asc
  echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu jammy stable" \
    > /etc/apt/sources.list.d/docker.list
  prep_apt_updated=false
  apt_install docker-ce docker-ce-cli containerd.io docker-buildx-plugin
  systemctl enable --now docker
}
if ! command -v docker >/dev/null; then
  apply "install Docker Engine and buildx from download.docker.com" install_docker_ce
elif ! docker buildx version >/dev/null 2>&1; then
  # JetPack may ship Ubuntu's docker.io; add its buildx rather than replace it.
  apply "install docker-buildx for the existing docker.io" apt_install docker-buildx
else
  ok "Docker $(docker version --format '{{.Server.Version}}' 2>/dev/null || echo '(daemon down)'), $(docker buildx version | cut -d' ' -f2)"
fi
if command -v docker >/dev/null; then
  prep_docker_root="$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo /var/lib/docker)"
  prep_free=$(( $(df -B1 --output=avail "$(dirname "$prep_docker_root")" | tail -1) / 1073741824 ))
  if ((prep_free >= 30)); then ok "${prep_free} GiB free for Docker at $prep_docker_root"
  else warn "${prep_free} GiB free for Docker at $prep_docker_root; the first image build needs about 30 GiB"; fi
fi

# --- Device access: OAK USB rule and operator groups. -------------------------
if cmp -s "$prep_repo/deploy/80-luxonis.rules" /etc/udev/rules.d/80-luxonis.rules; then
  ok "OAK udev rule"
else
  install_udev() {
    install -m 0644 "$prep_repo/deploy/80-luxonis.rules" /etc/udev/rules.d/80-luxonis.rules
    udevadm control --reload-rules
    udevadm trigger
  }
  apply "install OAK udev rule (replug the camera afterwards)" install_udev
fi
for prep_group in plugdev dialout docker; do
  if ! getent group "$prep_group" >/dev/null; then
    [[ "$prep_check" == true ]] && { printf 'todo  create group %s (with Docker)\n' "$prep_group"; continue; }
    fail "group $prep_group is absent"; continue
  fi
  if id -nG "$prep_user" | tr ' ' '\n' | grep -qx "$prep_group"; then
    ok "$prep_user in $prep_group"
  else
    # docker membership is root-equivalent on this host; it is required to record.
    apply "add $prep_user to $prep_group (log out and back in)" usermod -aG "$prep_group" "$prep_user"
  fi
done

# --- Serial: keep the PX4 USB-UART free of desktop probes. -------------------
# ModemManager probes new ttyUSB devices; brltty claims CP210x (10c4:ea60).
if systemctl is-enabled ModemManager >/dev/null 2>&1; then
  apply "disable ModemManager (probes the PX4 serial adapter)" systemctl disable --now ModemManager
else
  ok "ModemManager not enabled"
fi
if dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q 'ok installed'; then
  apply "remove brltty (claims CP210x serial adapters)" apt-get purge -y brltty
else
  ok "brltty absent"
fi

# --- Time: recordings rely on a disciplined host clock. -----------------------
if [[ "$(timedatectl show -p NTP --value 2>/dev/null)" == yes ]]; then
  ok "NTP enabled (synchronized=$(timedatectl show -p NTPSynchronized --value))"
else
  apply "enable NTP" timedatectl set-ntp true
fi

# --- Report only: storage and power mode are deliberate operator decisions. --
if mountpoint -q "$prep_storage"; then
  ok "$prep_storage mounted ($(findmnt -no SOURCE,FSTYPE "$prep_storage" | tr -s ' '))"
  grep -qsE "[[:space:]]$prep_storage[[:space:]]" /etc/fstab \
    && ok "$prep_storage in /etc/fstab" || warn "$prep_storage is not in /etc/fstab; it will not return after reboot"
else
  warn "$prep_storage is not mounted; see docs/jetson-setup.md#storage"
fi
if command -v nvpmodel >/dev/null; then
  ok "power mode: $(nvpmodel -q 2>/dev/null | head -1)"
fi

if [[ "$prep_check" == true ]]; then
  echo "Check only; nothing changed."
elif ((prep_fail == 0)); then
  echo "Host prepared. Log out/in as $prep_user, replug the OAK, then run deploy/build-image.sh."
fi
exit "$prep_fail"
