# Calibration mission drafts

`wr-map calibrate mission` generates files offline. It cannot connect to a vehicle,
upload, arm or fly. Use the repository runtime (includes PyProj); the portable
environment needs the `terrain` extra. Example **synthetic site coordinates**:

```bash
wr-map calibrate mission --home 52,13 --home-amsl-m 42 \
  --alt 20 --speed 2 --radius 15 --site-radius 30 --margin 10 --max-alt 30 \
  --output runs/calibration-flight/example.plan
```

Replace every site value with an operator-approved location and limits. `--alt`
and `--max-alt` are heights above home, not terrain. `--home-amsl-m` is the home
altitude above mean sea level. The clear site is a circle centred at home and must
include takeoff and landing. The generator reserves `--margin` outside the pattern
and at least three seconds of travel at the requested speed. These engineering
bounds do not establish braking distance, terrain clearance, legality or safety.

The draft contains takeoff, a speed command, heading targets at hover in both
directions, two sampled figure-eights, two-axis translation reversals with holds,
altitude steps, return and landing at home. Heading targets are **not proof of a
completed pirouette**. Actual yaw response, tracking, cornering and holds need SITL
and flight evidence. Multicopter motion cannot replace rich hand rotations for
roll/pitch calibration; solvers must reject poorly observed axes or translation.

The JSON uses QGC Plan v1, Mission v2, simple MAVLink items, and a circular inclusion
fence. Navigation uses relative-home altitude. See the official
[QGC plan format](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html)
and [PX4 multicopter mission behavior](https://docs.px4.io/main/en/flight_modes_mc/mission).
The generated horizontal fence does not configure vertical limits or enforcement.

## Required acceptance sequence

1. Load the `.plan` into a disconnected QGC Plan view. Record QGC version, plan hash
   and screenshots. Inspect home, altitude reference, every heading/hold, speed,
   flight corridor, takeoff/land and fence. Save/reload it and compare the effective
   mission items. A syntax test in Python is not a QGC validation.
2. Use a separate PX4 SITL environment with the intended firmware and multicopter
   model. Load this exact reviewed plan into the simulated vehicle. Save firmware
   revision, parameters, simulator setup, downloaded mission, ULog and QGC log.
   Check mission feasibility and all phases, including fence/ceiling and margin.
   Verify actual yaw headings and holds, maximum speed, tracking excursions and
   landing. Exercise mode-switch abort and link-loss behavior in simulation.
3. Record synthetic OAK-equivalent image/IMU and PX4 data through the repository
   recorder. Confirm topic frames, rates and clock domains match real acquisition.
   Feed the sealed session through both solvers; save per-axis excitation, lag,
   held-out residuals, and translation-observability outcomes. A vehicle-only SITL
   log does not establish an image/IMU calibration pipeline.
4. Before any real flight, the operator records site/airspace approval, terrain and
   obstacle clearance, weather, battery reserve, takeoff/landing area, active fence
   configuration and independent abort route. Verify the actual RC mode switch and
   intended Hold/Position/Return behavior using the deployed firmware's controls.
   Stick override is firmware/parameter-dependent; do not assume it works. The
   operator signs the exact plan hash and remains responsible for arming and flight.
5. Compare the flight calibration to independent bench results within propagated
   uncertainty. Retain disagreements and refuse unobservable parameters. Repeat
   under representative vibration/thermal/load conditions before operational use.

## Mission progress and phase provenance

`example.plan.phases.json` binds every generated item to the plan SHA256. Its
`mission_seq` is the expected zero-based PX4 mission index and `do_jump_id` is the
one-based QGC item identifier. The map includes non-navigation commands. QGC may
rewrite items while loading/uploading; compare the downloaded mission before
using indices as evidence. `sequence_mapping_verified` starts false.

Record `/mavros/mission/reached` with the calibration topics. A reached event marks
an endpoint, not the start of the named movement or necessarily the end of its
hold. Preserve source stamp, receive time and sequence. A downstream phase
derivation must bind the downloaded mission and phase-map hashes, reject missing,
reordered/repeated indices or mission restarts, and conservatively exclude gaps
and aborted segments. Never label an entire flight from a wall-clock schedule.

The generated sidecar always says `flight_ready: false`, with QGC, PX4 SITL,
recorder/solver SITL and operator sign-off **pending**. Preserve it unchanged;
store measured acceptance and the hashes it covers in a separate private report.
These acceptance steps remain outstanding for issue #16.

## Offline flight phase extraction

The recorder retains `/mavros/mission/reached` when published. After the recording
has completed and been sealed, extract the original events without touching it:

```bash
wr-map calibrate flight-phases runs/flight-session \
  --plan runs/calibration-flight/example.plan \
  --phase-map runs/calibration-flight/example.plan.phases.json \
  --output runs/flight-phase-review
```

The default output is diagnostic. `flight-phases.json` preserves every reached
sequence, original header stamp/frame and bag receipt timestamp. Candidate windows
are only the intervals between ordered endpoints, with a destination-phase hint.
They include an unknown mixture of motion, settling and hold time. They do not
establish actual movement onset, exposure alignment or axis excitation, and
`solver_window_qualified` remains false.

To qualify **sequence labeling only**, review the mission downloaded from PX4 in
the same SITL/flight session, compare each index/command to this exact plan, and
write a separate review JSON beside that downloaded artifact:

```json
{
  "schema_version": 1,
  "operator": "reviewer name",
  "plan_sha256": "SHA256 of example.plan",
  "phase_map_sha256": "SHA256 of example.plan.phases.json",
  "zero_based_px4_items_match": true,
  "downloaded_mission": {
    "path": "downloaded-mission.json",
    "sha256": "SHA256 of that downloaded artifact"
  }
}
```

Pass `--verified-numbering --numbering-evidence PATH` only after this review.
Changing either mission/map or downloaded artifact invalidates the binding.
Missing navigation endpoints, reordered/duplicate indices, restarts, unknown
indices or invalid clocks leave the entire extraction unqualified; original
events remain visible for diagnosis. Non-navigation commands need not emit
reached events. Changing `sequence_mapping_verified` in the generated phase map
does not substitute for this explicit review. `passed` refers only to this
sequence check, while `survey_ready` always remains false.

### Runtime prerequisite check

Before claiming simulator acceptance, locate and record the QGC executable, PX4
checkout/build or runtime image, simulator version and synthetic camera/IMU bridge.
Check executable paths, known installation/checkout roots and existing images
read-only; retain the dated environment inventory under ignored `runs/`. A mapping
runtime image alone does not provide a PX4 simulator. If these components are
unavailable, leave QGC/SITL acceptance pending and report provisioning as the blocker.
