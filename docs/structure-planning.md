# Structure fly-bys: planning a building with the fixed forward camera

The platform has one camera, fixed to the airframe, facing forward and pitched about
30° below the horizon. There is no gimbal, so the aircraft's yaw points the camera.
`wr-map structure-plan` generates a mission that images the walls of a structure with
a **fixed yaw on every leg**, tests it, and writes a mission view of the whole flight.

**Status: review only.** The plan has not been flown in the simulator or opened in
QGroundControl, and the capture workflow does not accept it yet. See
[what is not done](#what-is-not-done).

## What the aircraft does

Round the structure, counter-clockwise from the corner nearest the take-off point:

- **Faces.** For each wall, straight passes parallel to it at the stand-off distance,
  one per height, back and forth. The aircraft flies **sideways** with the nose square
  to the wall. The yaw does not change during a face.
- **Corners.** Stops on an arc round each corner, with the yaw stepped from one face's
  heading to the next (30° steps by default) and held on the corner. There is one stop
  per pass height and the aircraft is stationary at each. These views join the images
  of one face to the next; without them each face would reconstruct on its own.
- **Yaw changes happen only between these**, while the aircraft moves to the next
  position. The first waypoint of each new yaw holds for 2 s so the yaw has settled
  before any frame is counted.

The plan is plain waypoints, each with a yaw, because a QGroundControl pattern cannot
carry one. PX4 turns to a waypoint's yaw while flying to it and does not accept the
waypoint until the yaw is within `MIS_YAW_ERR` of it (read in PX4 v1.17.0 source).

## What the camera sees of a wall

At stand-off `d`, a ray `δ` below the horizon meets the wall `d × tan δ` below the
aircraft. With the camera 30° down, the image covers rays from 4° to 56° below the
horizon:

| Part of the image | Looks at the wall | Pixel on the wall at 10 m |
|---|---|---|
| Top edge | nearly square on | 0.29 cm |
| Centre | 30° off square | 0.37 × 0.43 cm |
| Bottom edge | 56° off square | 0.52 × 0.84 cm |

So for a wall the **top** of the image is the best part, the opposite of the ground
case. The plan counts a view only down to the steepest usable ray (45° by default),
which leaves a band on the wall from about `0.1 d` to `1.0 d` below the aircraft.
Consequences:

- **The aircraft flies above what it images.** The top pass is above the roof line.
- **The foot of the wall sets the stand-off.** It must be seen from the required
  number of passes, none lower than the minimum height (5 m by default). At 5 m that
  needs a stand-off of at least 8 m; closer is refused with the distance that works.
- **Blur sets the speed.** At the reviewed 1000 µs exposure and 10 m stand-off the
  limit is 1.4 m/s. Speed grows in proportion to the stand-off.

## Generate the plan

This runs on the workstation or the site laptop, like `survey-plan`
(`pip install -e '.[planner]'`).

1. Fill in the aircraft definition as for
   [`survey-plan`](survey-planning.md#generate-the-plan). Two more values are needed:
   `yaw_rate_deg_s` (`MPC_YAWRAUTO_MAX`) and `yaw_accel_deg_s2` (`MPC_YAWRAUTO_ACC`).
2. Check [`configs/oak-forward-mount.json`](../configs/oak-forward-mount.json). It
   holds the camera pitch (30°) and a tolerance (± 3°). **The pitch is the mount's
   design value, not a measurement.** Every plan is tested at the pitch and at both
   ends of the tolerance. Replace the values when the pitch has been measured.
3. Save the structure's footprint as KML or GeoJSON (one polygon).
4. Generate:

   ```bash
   wr-map structure-plan --outline barn.kml --structure-height-m 6 --base-m 0 \
     --home 48.40,12.90 --home-amsl-m 450 --aircraft /data/vehicles/hexa-1.json \
     --standoff-m 10 --flight-time-budget-min 15 --output runs/plans/barn-001
   ```

   - `--standoff-m` or `--gsd-cm`: exactly one. A GSD is the coarsest allowed in the
     usable band and sets the stand-off.
   - `--structure-height-m`: wall height above the ground at its base.
   - `--base-m`: that ground relative to the take-off point. It must be stated; `0`
     means level.
   - `--front` (80 %) and `--vertical` (60 %): overlap between frames along a pass and
     between passes up the wall.
   - `--max-depression-deg` (45), `--max-yaw-step-deg` (30), `--min-height-m` (5),
     `--min-clearance-m` (5): what counts as a usable view and how close the aircraft
     may come.

A plan is written only if it passes the check below. Otherwise the command exits 2,
writes nothing and prints the reasons. If the flight is over the time budget it also
prints the nearest stand-off that fits.

### Worked example: 20 m × 12 m barn, 6 m walls, 10 m stand-off

| Quantity | Value |
|---|---|
| Passes per face | 3, at 5.0, 8.4 and 11.9 m |
| Speed | 1.4 m/s, set by blur |
| Frame spacing, overlap along a pass | 0.7 m, 94 % |
| GSD in the usable band | 0.29 to 0.62 cm |
| Corner stops | 2 stations × 3 heights at each corner, 3 s each |
| Headings | every 30° from 0° to 330° |
| Estimated flight | 9.6 min |

## The mission view

`mission.html` in the output directory opens in any browser, with nothing to install.
It shows the whole sequence from take-off to landing:

- **Scene:** the structure, the path and the fence on satellite imagery, in a view
  that turns and tilts. Walls are coloured by how many passes see each point. The
  yellow patch is what the camera can use at the current time.
- **Camera frame:** what the camera sees at that moment, with the usable band marked
  and the share of it the structure fills.
- **Timeline and sequence:** every step, with the periods in which frames are counted.
  Play, step or drag.
- **Walls unfolded:** the coverage of each face, with the current view outlined.

The imagery is Esri World Imagery and needs a connection; without one the scene draws
on a plain grid. The map is a backdrop only. `review.kml` shows the same path in
Google Earth.

## The check

`wr-map structure-check` reads only the finished plan and the structure. It shares no
code with the route generator. It simulates the flight and refuses a plan unless:

- every waypoint carries a yaw, and each new yaw starts with a hold to settle;
- every wall sample is seen **sharply** from the required number of different lines,
  with the required frames on each, at the pitch and at both ends of its tolerance;
- neighbouring views round each corner are no more than the yaw step apart;
- the path stays the minimum clearance from the structure unless it is 5 m above it;
- no point is under the minimum height or over the height limit, and all are inside
  the inclusion fence;
- the estimated flight time fits the budget.

```bash
wr-map structure-check edited.plan --outline barn.kml --structure-height-m 6 --base-m 0 \
  --aircraft /data/vehicles/hexa-1.json --flight-time-budget-min 15 --output runs/plans/barn-001-recheck
```

A plan edited in QGroundControl is a different plan and must be checked again.

## Outputs

| File | Content |
|---|---|
| `structure.plan` | QGroundControl plan: speed, take-off, waypoints with yaw, return, inclusion fence. The same inputs give the same bytes |
| `structure-check.json` | The unmodified check report, with the simulated flight |
| `review.json` | Inputs with hashes, derived values and the limit that binds, each face's passes with their entry and exit items, every step with its times, warnings |
| `mission.html` | The mission view |
| `review.geojson`, `review.kml` | Outline, fence, path, passes and stops for a GIS or Google Earth |

## Limitations

- **Walls only.** The structure is its convex outline with vertical walls of one
  height. The roof is not planned or tested; recessed walls are further away than
  tested and the report says so.
- **The structure is the only obstacle known.** Trees, wires and neighbouring
  buildings are not. The aircraft passes behind the structure; line of sight stays
  with the operator.
- **Frames on climbs and descents are not counted.** They can be blurred and should
  be left out of processing.
- **One speed for the whole flight**, including transits.

## What is not done

- [ ] PX4 SIH flight of a generated plan, measuring heading against the planned yaw
- [ ] The operator's QGroundControl opening and uploading a generated plan
- [ ] Camera pitch measured on the airframe
- [ ] `capture --survey-check` and `survey-legs` accepting fly-by plans
- [ ] The yaw-sweep orbit (later, by decision) and the ground mission for this camera
