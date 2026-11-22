"""wr-mode against stand-ins for the Jetson system commands.

The stubs keep their state in files so each test can see exactly what the tool
changed: power mode, fan profile, boot target, desktop, radios and units.
"""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
MODES = ROOT / "deploy" / "system-modes"
WR_MODE = MODES / "wr-mode"

TRIMMED_ON = [
    "cups.service", "cups-browsed.service", "cups.socket", "cups.path", "ModemManager.service",
    "whoopsie.service", "unattended-upgrades.service", "apt-daily.timer",
    "apt-daily-upgrade.timer", "motd-news.timer", "man-db.timer",
]

STUBS = {
    "systemctl": r"""#!/usr/bin/env bash
S=$WR_TEST_STATE; echo "systemctl $*" >> "$S/log"
args=(); quiet=0
for a in "$@"; do [[ $a == --quiet ]] && quiet=1; [[ $a == --* ]] || args+=("$a"); done
u=${args[1]:-}
case ${args[0]} in
  list-unit-files) [[ -f $S/units/$u ]] && echo "$u $(cat "$S/units/$u")"; exit 0 ;;
  is-enabled) st=$(cat "$S/units/$u" 2>/dev/null || echo not-found); echo "$st"; [[ $st == enabled ]] ;;
  is-active)
    case $u in
      display-manager.service) st=$(cat "$S/desktop") ;;
      ssh|nv-l4t-usb-device-mode) st=active ;;
      *) [[ $(cat "$S/units/$u" 2>/dev/null) == enabled ]] && st=active || st=inactive ;;
    esac
    [[ $st == running ]] && st=active; [[ $st == stopped ]] && st=inactive
    (( quiet )) || echo "$st"; [[ $st == active ]] ;;
  disable) echo disabled > "$S/units/$u" ;;
  enable) echo enabled > "$S/units/$u" ;;
  get-default) cat "$S/default" ;;
  set-default) echo "$u" > "$S/default" ;;
  stop) [[ $u == display-manager.service ]] && echo stopped > "$S/desktop"; exit 0 ;;
  start) [[ $u == graphical.target ]] && echo running > "$S/desktop"; exit 0 ;;
  isolate) echo "isolate is not allowed" >&2; exit 9 ;;
esac
""",
    "nvpmodel": r"""#!/usr/bin/env bash
S=$WR_TEST_STATE; echo "nvpmodel $*" >> "$S/log"
if [[ $1 == -q ]]; then printf 'NV Power Mode: %s\n0\n' "$(cat "$S/power")"; exit 0; fi
name=$(sed -nE "s/^< POWER_MODEL ID=$2 NAME=([^ ]+) >/\1/p" "$WR_MODE_NVPMODEL_CONF")
old=$(cat "$S/power"); echo "$name" > "$S/power"
if [[ $old == MAXN_SUPER || $name == MAXN_SUPER ]]; then
  echo "NVPM WARN: Reboot required for changing to this power mode: $2"
  printf 'Do you want to reboot now? (yes/no): '; read -r ans; echo "answer=$ans" >> "$S/log"
fi
""",
    "nmcli": r"""#!/usr/bin/env bash
S=$WR_TEST_STATE; echo "nmcli $*" >> "$S/log"
if [[ $# -eq 2 ]]; then cat "$S/wifi"; exit 0; fi
[[ -f $S/wifi_stuck ]] && exit 0
[[ $3 == on ]] && echo enabled > "$S/wifi" || echo disabled > "$S/wifi"
""",
    "rfkill": r"""#!/usr/bin/env bash
S=$WR_TEST_STATE; echo "rfkill $*" >> "$S/log"
case $1 in
  block) echo yes > "$S/bt" ;;
  unblock) echo no > "$S/bt" ;;
  list) printf '0: hci0: Bluetooth\n\tSoft blocked: %s\n' "$(cat "$S/bt")" ;;
esac
""",
    "ip": r"""#!/usr/bin/env bash
echo "3: wlP1p1s0    inet 192.168.1.50/24 brd 192.168.1.255 scope global wlP1p1s0"
echo "5: l4tbr0    inet 192.168.55.1/24 brd 192.168.55.255 scope global l4tbr0"
""",
    "ss": r"""#!/usr/bin/env bash
S=$WR_TEST_STATE
[[ -f $S/ssh_wifi ]] && echo "0 0 192.168.1.50:22 192.168.1.20:51234"
[[ -f $S/ssh_usb ]] && echo "0 0 [::ffff:192.168.55.1]:22 [::ffff:192.168.55.100]:40000"
exit 0
""",
}

NVPMODEL_CONF = """< POWER_MODEL ID=0 NAME=15W >
CPU_ONLINE CORE_0 1
< POWER_MODEL ID=1 NAME=25W >
CPU_ONLINE CORE_0 1
< POWER_MODEL ID=2 NAME=MAXN_SUPER >
CPU_ONLINE CORE_0 1
< PM_CONFIG DEFAULT=1 >
"""

FAN_CONF = """POLLING_INTERVAL 2
\tFAN_PROFILE quiet {
\t\t0\t0\t255\t6000
\t}
\tFAN_PROFILE cool {
\t\t0\t0\t255\t6000
\t}
\tFAN_DEFAULT_CONTROL close_loop
\tFAN_DEFAULT_PROFILE quiet
"""


class Orin:
    """A stock Orin as the stubs see it: 25 W, desktop running, radios on."""

    def __init__(self, tmp: Path):
        self.state = tmp / "state"
        (self.state / "units").mkdir(parents=True)
        for unit in TRIMMED_ON:
            self.write(f"units/{unit}", "enabled")
        self.write("units/kerneloops.service", "disabled")  # installed but already off
        for name, value in {"power": "25W", "default": "graphical.target", "desktop": "running",
                            "wifi": "enabled", "bt": "no", "log": ""}.items():
            self.write(name, value)
        bin_dir = tmp / "bin"
        bin_dir.mkdir()
        for name, body in STUBS.items():
            (bin_dir / name).write_text(body)
            (bin_dir / name).chmod(0o755)
        etc = tmp / "etc"
        etc.mkdir()
        (etc / "nvpmodel.conf").write_text(NVPMODEL_CONF)
        self.fan_conf = etc / "nvfancontrol.conf"
        self.fan_conf.write_text(FAN_CONF)
        self.profiles = tmp / "profiles"
        shutil.copytree(MODES / "profiles", self.profiles)
        self.etc = etc / "wr-mode"
        self.env = {
            **os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}", "WR_TEST_STATE": str(self.state),
            "WR_MODE_TEST_NONROOT": "1", "WR_MODE_ETC": str(self.etc),
            "WR_MODE_PROFILE_DIR": str(self.profiles), "WR_MODE_NVPMODEL_CONF": str(etc / "nvpmodel.conf"),
            "WR_MODE_FAN_CONF": str(self.fan_conf), "WR_MODE_FAN_STATUS": str(tmp / "fan-status"),
        }
        self.env.pop("WR_MODE_RECORD_PROFILE", None)

    def write(self, name: str, value: str):
        (self.state / name).write_text(value + "\n" if value else "")

    def read(self, name: str) -> str:
        return (self.state / name).read_text().strip()

    def fan(self) -> str:
        return next(line.split()[1] for line in self.fan_conf.read_text().splitlines()
                    if "FAN_DEFAULT_PROFILE" in line)

    def units(self) -> dict:
        return {p.name: p.read_text().strip() for p in (self.state / "units").iterdir()}

    def snapshot(self) -> dict:
        return {**{n: self.read(n) for n in ("power", "default", "desktop", "wifi", "bt")},
                "fan": self.fan(), "units": self.units()}

    def run(self, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run(["bash", str(WR_MODE), *args], env=self.env, capture_output=True,
                              text=True, timeout=30)


@pytest.fixture
def orin(tmp_path):
    return Orin(tmp_path)


def test_profiles_list_and_parse(orin):
    listed = orin.run("list")
    assert listed.returncode == 0
    for name in ("home", "field", "flight"):
        assert name in listed.stdout
        assert orin.run("check", name).returncode in (0, 1)  # parses; state may differ


def test_field_then_home_restores_services_but_not_modem_manager(orin):
    assert orin.run("apply", "field", "--now").returncode == 0
    assert orin.snapshot() | {"units": None} == {
        "power": "15W", "default": "multi-user.target", "desktop": "stopped",
        "wifi": "enabled", "bt": "yes", "fan": "cool", "units": None}
    assert all(state == "disabled" for state in orin.units().values())
    assert orin.run("check", "field").returncode == 0

    home = orin.run("apply", "home", "--now")
    assert home.returncode == 0, home.stderr
    assert orin.run("check", "home").returncode == 0
    expected = {unit: "enabled" for unit in TRIMMED_ON} | {"kerneloops.service": "disabled"}
    expected["ModemManager.service"] = "disabled"  # probes the PX4 adapter: never restored
    assert orin.units() == expected
    assert orin.read("desktop") == "running"
    assert "isolate" not in orin.read("log")


def test_apply_without_now_waits_for_reboot_to_drop_the_desktop(orin):
    result = orin.run("apply", "field")
    assert result.returncode == 0
    assert "next boot" in result.stdout
    assert orin.read("default") == "multi-user.target"
    assert orin.read("desktop") == "running"


def test_dry_run_changes_nothing(orin):
    before = orin.snapshot()
    result = orin.run("apply", "flight", "--dry-run")
    assert result.returncode == 0
    assert "[dry-run] nvpmodel -m 0" in result.stdout
    assert orin.snapshot() == before
    assert not (orin.etc / "current").exists()


def test_ensure_defaults_to_field_and_verifies(orin):
    result = orin.run("ensure")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "in 'field'" in result.stdout
    assert orin.read("desktop") == "stopped"
    assert (orin.etc / "current").read_text().strip() == "field"


def test_ensure_takes_the_profile_from_the_environment(orin):
    orin.write("ssh_usb", "")
    orin.env["WR_MODE_RECORD_PROFILE"] = "flight"
    assert orin.run("ensure").returncode == 0
    assert orin.read("wifi") == "disabled"


def test_ensure_refuses_a_profile_not_meant_for_recording(orin):
    before = orin.snapshot()
    result = orin.run("ensure", "home")
    assert result.returncode == 2
    assert "not a recording profile" in result.stderr
    assert orin.snapshot() == before


def test_ensure_will_not_cut_an_ssh_session_on_wifi(orin):
    orin.write("ssh_wifi", "")
    before = orin.snapshot()
    refused = orin.run("ensure", "flight")
    assert refused.returncode == 5
    assert "192.168.55.1" in refused.stderr
    assert orin.snapshot() == before  # refused before anything changed
    assert orin.run("ensure", "flight", "--force").returncode == 0
    assert orin.read("wifi") == "disabled"


def test_ensure_over_usb_c_turns_wifi_off(orin):
    orin.write("ssh_usb", "")
    assert orin.run("ensure", "flight").returncode == 0
    assert orin.read("wifi") == "disabled"


def test_ensure_stops_when_a_reboot_is_needed(orin):
    (orin.profiles / "fast.conf").write_text(
        (orin.profiles / "field.conf").read_text().replace("POWER_MODE=15W", "POWER_MODE=MAXN_SUPER"))
    result = orin.run("ensure", "fast")
    assert result.returncode == 3
    assert "answer=no" in orin.read("log")  # nvpmodel's reboot prompt was declined


def test_ensure_fails_when_the_state_does_not_stick(orin):
    orin.write("ssh_usb", "")
    orin.write("wifi_stuck", "")
    result = orin.run("ensure", "flight")
    assert result.returncode == 4
    assert "wifi is enabled, want off" in result.stdout


def test_check_reports_drift(orin):
    assert orin.run("ensure", "field").returncode == 0
    orin.write("units/cups.service", "enabled")
    result = orin.run("check", "field")
    assert result.returncode == 1
    assert "cups.service" in result.stdout


def test_unknown_power_mode_changes_nothing(orin):
    (orin.profiles / "bad.conf").write_text(
        (orin.profiles / "field.conf").read_text().replace("POWER_MODE=15W", "POWER_MODE=30W"))
    before = orin.snapshot()
    result = orin.run("apply", "bad")
    assert result.returncode == 2
    assert "Available: 15W 25W MAXN_SUPER" in result.stderr
    assert orin.snapshot() == before


def test_recording_hooks_only_grant_ensure():
    dropin = (MODES / "wr-mapping-mode.conf").read_text()
    assert "ExecStartPre=+/usr/local/sbin/wr-mode ensure\n" in dropin
    rules = [line for line in (MODES / "wr-mode.sudoers").read_text().splitlines()
             if line and not line.startswith("#")]
    assert rules == ["%wr-mode ALL=(root) NOPASSWD: /usr/local/sbin/wr-mode ensure, "
                     "/usr/local/sbin/wr-mode ensure *"]
