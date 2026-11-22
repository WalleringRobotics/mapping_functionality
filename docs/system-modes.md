# System modes

`deploy/system-modes/wr-mode` puts the Orin into a named system profile. It is
standalone: once installed it needs nothing else from this repository. Written for
the Orin Nano Super developer kit on JetPack 6.

| | home | field | flight |
|---|---|---|---|
| Power mode | 25W | 15W | 15W |
| Fan profile | quiet | cool | cool |
| Desktop | yes | no (headless) | no (headless) |
| Wi-Fi | on | on, for the phone hotspot | off |
| Bluetooth | on | off | off |
| Optional services | as installed | stopped | stopped |
| Recording profile | no | yes | yes |

Profiles are plain text in `/etc/wr-mode/profiles/`; edit them on the device or add
new ones. `flight` keeps Wi-Fi and Bluetooth off beside a 2.4 GHz RC link.

## Install

```bash
sudo deploy/system-modes/install.sh
```

This installs `/usr/local/sbin/wr-mode` and the profiles (an edited profile already
on the device is kept), enables `ssh`, and records which optional services are on,
so `home` can restore them. It also installs the recording hooks described below and
adds the installing user to the `wr-mode` group.

Add Wi-Fi networks as system connections so they come up at boot, before anyone
logs in. The higher priority wins when both are in range:

```bash
sudo nmcli dev wifi connect "<home SSID>" password "<password>" name home-wifi
sudo nmcli con modify home-wifi connection.autoconnect-priority 20
sudo nmcli con add type wifi con-name field-hotspot ssid "<hotspot SSID>" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "<password>" connection.autoconnect-priority 10
```

## Use

```bash
wr-mode status                        # active profile, power, fan, radios, addresses
sudo wr-mode apply field --dry-run    # show what would change
sudo wr-mode apply field              # desktop stops at the next boot
sudo wr-mode apply field --now        # ... or now (ends a desktop session)
wr-mode check field                   # exit 0 only if the live state matches
```

Every setting is checked before anything changes, and settings persist: the Orin
boots into the last profile applied. If a power-mode change needs a reboot, the tool
says so.

## Recording forces the mode

`wr-mode ensure [profile]` is the recording preflight. It applies a `RECORDING=yes`
profile immediately, including stopping the desktop, then verifies the result. The
profile defaults to `WR_MODE_RECORD_PROFILE`, else `field`.

| Exit | Meaning |
|---|---|
| 0 | in the profile; safe to record |
| 2 | usage error, unknown profile, or a profile not meant for recording |
| 3 | the power-mode change needs a reboot first |
| 4 | the state did not verify after applying |
| 5 | refused: the profile turns Wi-Fi off and an SSH session is on Wi-Fi |

- **`wr-mapping.service`**: a drop-in runs `wr-mode ensure` before the recorder, as
  root (`ExecStartPre=+`), so the service does not start unless the Orin reached the
  profile. Set `WR_MODE_RECORD_PROFILE=flight` in `/etc/wallering-mapping.env` to
  record in `flight`. To remove the hook, delete
  `/etc/systemd/system/wr-mapping.service.d/wr-mode.conf` and run
  `sudo systemctl daemon-reload`.
- **Recorders started as a normal user** call the preflight first and stop on a
  non-zero exit. Members of the `wr-mode` group may run `ensure`, and only `ensure`,
  without a password:

  ```bash
  sudo -n /usr/local/sbin/wr-mode ensure field || exit
  ```

## Getting back in

These are never touched, so one of them always works:

- **USB-C cable** from the port next to the 40-pin header to a laptop:
  `ssh <user>@192.168.55.1`
- `ssh`, `NetworkManager` and `avahi-daemon` (for `<hostname>.local`)

If you are connected over Wi-Fi, `flight` refuses to run until you reconnect over
USB-C (`--force` overrides). To leave `flight`, connect over USB-C and run
`sudo wr-mode field`.

## Optional services

Printing (`cups`), `ModemManager`, crash reporters, automatic apt upgrades and
maintenance timers; only installed units are touched (`TRIM_UNITS` in the script).
Automatic upgrades stay off in the field so a package update cannot change a system
that was tested before the trip. `ModemManager` is never restored, because it probes
the PX4 serial adapter (`deploy/prepare-orin.sh` disables it too).

## Verification status

`tests/test_system_modes.py` runs the tool against stand-ins for `systemctl`,
`nvpmodel`, `nmcli`, `rfkill`, `ip` and `ss`. It has not yet run on a physical Orin;
run `sudo wr-mode apply field --dry-run` there first.
