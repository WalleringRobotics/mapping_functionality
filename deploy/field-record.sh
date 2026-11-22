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
# The startup check already records 5 s of continuous data from every sensor.
field_warmup="${WR_MAPPING_WARMUP:-0}"
field_rule="============================================================"
if [[ -t 1 ]]; then
  field_good=$'\e[1;32m' field_bad=$'\e[1;31m' field_plain=$'\e[0m'
else
  field_good="" field_bad="" field_plain=""
fi
if [[ -t 1 && "$(locale charmap 2>/dev/null)" == UTF-8 ]]; then
  field_frames=(⠋ ⠙ ⠹ ⠸ ⠼ ⠴ ⠦ ⠧ ⠇ ⠏)
else
  field_frames=('|' / - '\')
fi
field_frame=0
field_t0="${EPOCHREALTIME/./}"
field_note=$((SECONDS + 10))

field_session() { readlink -f "$field_root/latest" 2>/dev/null || true; }
field_elapsed() {
  local tenths=$(((${EPOCHREALTIME/./} - field_t0) / 100000))
  printf '%d.%d s' $((tenths / 10)) $((tenths % 10))
}
field_clear() { [[ ! -t 1 ]] || printf '\r\e[K'; }
# One moving status line on a terminal; a plain line every 10 s otherwise.
field_spin() {
  if [[ -t 1 ]]; then
    printf '\r\e[K  %s %s... %s' "${field_frames[field_frame++ % ${#field_frames[@]}]}" "$1" "$(field_elapsed)"
  elif ((SECONDS >= field_note)); then
    echo "  ... $1 ($(field_elapsed))"
    field_note=$((SECONDS + 10))
  fi
}
# Why a session is not recording, in operator terms.
field_reason() {
  local session="$1" missing reason=""
  missing="$(python3 -c 'import json, sys; print(*json.load(open(sys.argv[1]))["missing"])' \
    "$session/readiness.json" 2>/dev/null || true)"
  [[ "$missing" != */mavros/* ]] || reason="PX4 not connected (TELEM2 adapter on USB-A, PX4 powered?)"
  [[ "$missing" != */oak* ]] || reason="${reason:+$reason; }OAK camera not streaming (USB 3 cable?)"
  [[ -n "$reason" || " $missing " != *" solid "* ]] ||
    reason="sensor data kept stalling; no 5 s without a gap (see $session/readiness.json)"
  [[ -n "$reason" || ! -s "$session/failure.txt" ]] || reason="$(head -1 "$session/failure.txt")"
  # The recorder's own refusals are plain sentences; ROS log lines start with "[".
  [[ -n "$reason" ]] || reason="$(grep -E '^[A-Z][a-z]' "$session.log" 2>/dev/null | tail -1 || true)"
  echo "${reason:-unknown}"
}
field_banner() {
  local color="$1" title="$2"
  shift 2
  echo "$color$field_rule"
  echo "  $title"
  echo "$field_rule$field_plain"
  printf '  %s\n' "$@"
}
field_report() {
  local session state active verdict color="$field_bad"
  session="$(field_session)"
  [[ -n "$session" ]] || { echo "No field recording yet"; return; }
  state="$(cat "$session/state" 2>/dev/null || echo unknown)"
  active="$(systemctl --user is-active "$field_unit" 2>/dev/null || true)"
  case "$state:$active" in
    starting:active) verdict="STARTING: wait for the all-clear (about 15 s); not yet safe to disconnect" color="" ;;
    warming:active|recording:active) verdict="RECORDING: safe to disconnect" color="$field_good" ;;
    postroll:active|stopping:active) verdict="STOPPING: sealing; keep the Orin powered" color="" ;;
    complete:*) verdict="COMPLETE: sealed; safe to power off" color="$field_good" ;;
    failed:*) verdict="NOT RECORDING: $(field_reason "$session")" ;;
    *) verdict="NOT RECORDING: the recorder is $active with the session in state $state" ;;
  esac
  echo "$color$verdict${color:+$field_plain}"
  echo "session:  $session"
  echo "state:    $state (recorder $active)"
  echo "size:     $(du -sh "$session" 2>/dev/null | cut -f1)"
  echo "log:      $session.log"
}
# What a starting recorder is doing, from the files its checks leave behind.
field_doing() {
  local session="$1" pending=()
  if [[ ! -e "$session/started-utc.txt" ]]; then
    echo "Starting the recorder container"
  elif [[ ! -e "$session/topics.txt" ]]; then
    echo "Checking that no other recorder or MAVROS is running"
  else
    [[ -e "$session/mavros-state.yaml" ]] || pending+=("the PX4 link")
    [[ -e "$session/oak-parameters.yaml" ]] || pending+=("the OAK camera to boot and stream")
    if ((${#pending[@]})); then
      echo "Waiting for ${pending[0]}${pending[1]:+ and ${pending[1]}}"
    else
      echo "Checking 5 s of continuous data from the cameras, OAK IMU and PX4 IMU"
    fi
  fi
}

# Ctrl+Z would freeze the waits below and hide the verdict.
trap '' TSTP
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
      echo "NOT RECORDING: it would stop with the SSH session; run: sudo loginctl enable-linger $(id -un)" >&2
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
        echo "NOT RECORDING: expected one PX4 adapter in /dev/serial/by-id, found ${#field_serial[@]} (TELEM2 adapter on USB-A?)" >&2
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
      --warmup "$field_warmup" --start-mavros --announce "${field_args[@]}"
    ln -sfn "$field_output" "$field_root/latest"
    echo "Starting field recording $(basename "$field_output")."
    echo "Keep the cable connected until the all-clear below (about 15 s)."
    # Ctrl+C stops only the waiting; the recording carries on and status reports it.
    trap 'field_clear; echo; echo "Stopped waiting; the recording was not stopped."; field_report; exit 130' INT
    field_steps=("started-utc.txt|Recorder container started"
                 "topics.txt|No other recorder or MAVROS running"
                 "mavros-state.yaml|PX4 connected over TELEM2"
                 "oak-parameters.yaml|OAK camera and IMU streaming"
                 "graph.txt|5 s of continuous data from every sensor")
    declare -A field_shown=()
    field_t0="${EPOCHREALTIME/./}"
    field_started=$SECONDS
    while ((SECONDS < field_started + field_ready_seconds)); do
      for field_step in "${field_steps[@]}"; do
        if [[ -z "${field_shown[${field_step%%|*}]:-}" && -e "$field_output/${field_step%%|*}" ]]; then
          field_shown[${field_step%%|*}]=1
          field_clear
          printf '  [ok] %-42s %s\n' "${field_step#*|}" "$(field_elapsed)"
        fi
      done
      case "$(cat "$field_output/state" 2>/dev/null || true)" in
        warming|recording)
          field_clear
          field_length="until you run stop"
          for ((field_i = 0; field_i < ${#field_args[@]}; field_i++)); do
            if [[ "${field_args[field_i]}" == --duration && "${field_args[field_i + 1]}" != 0 ]]; then
              field_length="${field_args[field_i + 1]} s, then it stops by itself"
            fi
          done
          field_buzzer="plays on the vehicle now"
          ((field_warmup == 0)) || field_buzzer="plays on the vehicle when the ${field_warmup} s warmup ends"
          field_banner "$field_good" "ALL CLEAR: recording. Safe to disconnect." \
            "Session: $(basename "$field_output")" \
            "Buzzer:  $field_buzzer" \
            "Length:  $field_length" \
            "Check:   bash deploy/field-record.sh status" \
            "Stop:    after landing, before power off: bash deploy/field-record.sh stop"
          exit 0 ;;
        failed) break ;;
      esac
      systemctl --user is-active --quiet "$field_unit" || break
      field_spin "$(field_doing "$field_output")"
      sleep 0.1
    done
    field_clear
    systemctl --user stop "$field_unit" 2>/dev/null || true
    field_banner "$field_bad" "NOT RECORDING: startup failed. Do not fly expecting data." \
      "Reason:  $(field_reason "$field_output")" "Log:     $field_output.log"
    exit 1 ;;
  stop)
    if systemctl --user is-active --quiet "$field_unit"; then
      field_output="$(field_session)"
      [[ "$(cat "$field_output/state" 2>/dev/null)" != starting ]] ||
        echo "Startup checks were still running; this aborts the session."
      echo "Stopping and sealing (about 20 s); keep the Orin powered until the verdict below."
      trap 'field_clear; echo; echo "Stopped waiting; sealing continues. Keep the Orin powered and check status."; exit 130' INT
      systemctl --user stop --no-block "$field_unit"
      field_t0="${EPOCHREALTIME/./}"
      while [[ "$(systemctl --user show -p ActiveState --value "$field_unit" 2>/dev/null)" =~ ^(active|activating|deactivating)$ ]]; do
        case "$(cat "$field_output/state" 2>/dev/null || true)" in
          postroll|stopping) field_spin "Closing the bag and writing checksums" ;;
          *) field_spin "Stopping the camera, MAVROS and recorder" ;;
        esac
        sleep 0.1
      done
      field_clear
    fi
    field_report ;;
  status) field_report ;;
  *)
    echo "Usage: $0 start [--duration SECONDS] [--survey-check CHECKED_DIR] | stop | status" >&2
    exit 2 ;;
esac
