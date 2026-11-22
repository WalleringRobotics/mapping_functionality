#!/usr/bin/env bash
# Field recording that outlives the laptop's SSH session. Connect the laptop to
# the Orin's USB-C port (ssh to 192.168.55.1), start, unplug, and stop after
# landing before removing power. A transient systemd user unit owns the run:
# it needs lingering for this user, not root.
set -euo pipefail
field_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
field_unit=wr-field-record
field_root="${WR_MAPPING_FIELD_ROOT:-$field_repo/runs/field}"
field_ready_seconds="${WR_MAPPING_READY_SECONDS:-180}"

field_session() { readlink -f "$field_root/latest" 2>/dev/null || true; }
field_report() {
  local session state
  session="$(field_session)"
  [[ -n "$session" ]] || { echo "No field recording yet"; return; }
  state="$(cat "$session/state" 2>/dev/null || echo unknown)"
  case "$state" in
    starting) state+=" (startup checks, about 10 s; not yet safe to unplug)" ;;
    warming|recording) state+=" (recording; safe to unplug)" ;;
    postroll|stopping) state+=" (stopping; keep power on)" ;;
    complete) state+=" (sealed; safe to power off)" ;;
    failed) state+=" (see $session.log)" ;;
  esac
  echo "recorder: $(systemctl --user is-active "$field_unit" 2>/dev/null || true)"
  echo "session:  $session"
  echo "state:    $state"
  echo "size:     $(du -sh "$session" 2>/dev/null | cut -f1)"
}

case "${1:-}" in
  start)
    shift
    field_args=(--duration 0)
    while (($#)); do
      case "$1" in
        --duration|--survey-check) (($# >= 2)) || exit 2; field_args+=("$1" "$2"); shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
      esac
    done
    [[ "$(loginctl show-user "$(id -un)" -p Linger --value 2>/dev/null)" == yes ]] || {
      echo "The recording would stop with the SSH session; run: sudo loginctl enable-linger $(id -un)" >&2
      exit 2
    }
    if systemctl --user is-active --quiet "$field_unit"; then
      echo "A field recording is already running" >&2; field_report >&2; exit 2
    fi
    if [[ -z "${WR_MAPPING_SERIAL_DEVICE:-}" ]]; then
      shopt -s nullglob
      field_serial=(/dev/serial/by-id/*)
      shopt -u nullglob
      ((${#field_serial[@]} == 1)) || {
        echo "Expected one PX4 adapter in /dev/serial/by-id, found ${#field_serial[@]}; set WR_MAPPING_SERIAL_DEVICE" >&2
        exit 2
      }
      WR_MAPPING_SERIAL_DEVICE="${field_serial[0]}"
    fi
    mkdir -p "$field_root"
    field_output="$field_root/$(date -u +%Y%m%dT%H%M%SZ)"
    systemctl --user reset-failed "$field_unit" 2>/dev/null || true
    # SIGINT is the recorder's clean stop (as Ctrl+C); sealing may take minutes.
    # The user journal is not kept on this host, so log beside the session.
    systemd-run --user --quiet --unit "$field_unit" \
      --property KillSignal=SIGINT --property TimeoutStopSec=infinity \
      --property "StandardOutput=append:$field_output.log" \
      --property "StandardError=append:$field_output.log" \
      --working-directory "$field_repo" \
      --setenv "WR_MAPPING_SERIAL_DEVICE=$WR_MAPPING_SERIAL_DEVICE" \
      --setenv "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}" \
      "$field_repo/.venv/bin/wr-map" capture --output "$field_output" \
      --warmup "${WR_MAPPING_WARMUP:-60}" --start-mavros --announce "${field_args[@]}"
    ln -sfn "$field_output" "$field_root/latest"
    echo "Starting $field_output"
    echo "Waiting for the startup checks (about 10 s). Ctrl+C only stops waiting; check with status."
    # Report success only after the startup checks pass, so unplugging is safe.
    field_deadline=$((SECONDS + field_ready_seconds))
    while ((SECONDS < field_deadline)); do
      case "$(cat "$field_output/state" 2>/dev/null || true)" in
        warming|recording)
          echo "Startup checks passed; recording continues after you unplug."
          echo "The vehicle buzzer plays when warmup ends."
          exit 0 ;;
        failed) break ;;
      esac
      systemctl --user is-active --quiet "$field_unit" || break
      sleep 0.2
    done
    echo "Recording did not start; see $field_output.log" >&2
    systemctl --user stop "$field_unit" 2>/dev/null || true
    field_report >&2
    exit 1 ;;
  stop)
    if systemctl --user is-active --quiet "$field_unit"; then
      [[ "$(cat "$(field_session)/state" 2>/dev/null)" != starting ]] ||
        echo "Startup checks were still running; this aborts the session."
      echo "Stopping and sealing; keep the Orin powered until this returns."
      systemctl --user stop "$field_unit"
    fi
    field_report ;;
  status) field_report ;;
  *)
    echo "Usage: $0 start [--duration SECONDS] [--survey-check CHECKED_DIR] | stop | status" >&2
    exit 2 ;;
esac
