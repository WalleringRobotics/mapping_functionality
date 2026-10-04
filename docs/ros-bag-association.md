# Diagnostic association for ROS recordings

`sync-bag` works directly from a completed, SHA256-sealed rosbag2/MCAP recording.
It associates the selected image stream with `/mavros/local_position/pose` and
optionally composes the camera-to-body rig transform. It writes metadata only:
the original images, IMU samples and poses remain in the source MCAP.

```bash
wr-map sync-bag runs/ros-session --stream left \
  --rig-calibration runs/rig-calibration/rig.json \
  --output runs/ros-session-association
```

Without `--timestamp-bridge`, lookup uses the original image ROS header timestamp
unchanged. A blocking reason says that the camera and pose header domains have no
validated bridge. Merely sharing nanosecond units or the same computer does not
prove that the headers describe the same physical instant. Existing rig time
offsets are retained as provenance, but are not applied without explicit links.

`associations.jsonl`, `body-poses.csv` and `camera-poses.csv` contain original image
header and bag receipt times plus the pose lookup timestamp. Associations retain
bracketing pose source and receipt times. `image_id` is a bag topic/ordinal
identifier, **not** an exported image filename. These diagnostic CSVs are not
qualified camera geolocation inputs to the reconstruction/accuracy pipeline.

The command rejects changed/missing frames, nonmonotonic timestamps, invalid
quaternions, extrapolation, and pose gaps over `--max-pose-gap-ms` (default 100 ms).
`--min-fraction` gates association coverage. `passed` means sufficient samples
were interpolated; `survey_ready` always remains false. Vehicle body/ENU frame
semantics, body-attitude uncertainty, absolute exposure timing, row readout and
GNSS reference/accuracy require separate evidence. Image header frame names must
exactly match the rig optical frame for camera composition; no frame alias is
inferred. RGB/right factory covariance remains unknown.

## Explicit timestamp bridge

Use `--timestamp-bridge PATH` only when the missing clock relations have measured
evidence. The file binds the source recording seal and the exact rig calibration
hash. It supplies an **ordered** chain beginning at
`ros_header:/oak/<stream>/image_raw` and ending at
`ros_header:/mavros/local_position/pose`. Every link follows
`target_ns = source_ns + offset_ns`. Adjacent domains must match exactly.

A link either references an existing rig clock pair or supplies a measured offset,
known nonnegative one-sigma uncertainty and hashed evidence. The latter includes
zero-offset identity relations: zero still requires measurements. For example,
the structure below is a template only; placeholder hashes and offsets must be
replaced with actual results before use:

```json
{
  "schema_version": 1,
  "source_seal_sha256": "SHA256 of session/SHA256SUMS",
  "rig_calibration_sha256": "SHA256 of rig.json",
  "links": [
    {
      "source": "ros_header:/oak/left/image_raw",
      "target": "oak_camera_exposure",
      "offset_ns": 0,
      "sigma_ns": 1000,
      "evidence": {"path": "camera-header-reference.json", "sha256": "measured evidence hash"}
    },
    {
      "source": "oak_camera_exposure",
      "target": "oak_imu",
      "rig_offset": ["oak_camera_exposure", "oak_imu"]
    },
    {
      "source": "oak_imu",
      "target": "oak_ros_stamp",
      "offset_ns": 0,
      "sigma_ns": 1000,
      "evidence": {"path": "imu-header-reference.json", "sha256": "measured evidence hash"}
    },
    {
      "source": "oak_ros_stamp",
      "target": "px4_ros_stamp",
      "rig_offset": ["oak_ros_stamp", "px4_ros_stamp"]
    },
    {
      "source": "px4_ros_stamp",
      "target": "ros_header:/mavros/local_position/pose",
      "offset_ns": 0,
      "sigma_ns": 1000,
      "evidence": {"path": "px4-pose-reference.json", "sha256": "measured evidence hash"}
    }
  ]
}
```

Evidence paths are relative to the bridge file and must stay within its directory.
Record the measured timestamp convention, devices, software, observation data,
method and uncertainty in each evidence artifact. Do not use this example's
zero offsets or sigma values as calibration. A direct independently measured
image-header-to-pose-header relation is also accepted; no rig pair is silently
added to it. Offsets use integer nanoseconds, with a one-second per-link/net limit.
Unknown rig offset/sigma, wrong endpoints, discontinuities, cycles or changed
hashes reject the supplied bridge before outputs are created.

The report lists each resolved link and signed offset sum. Its conservative sigma
sum allows correlation; it is a statistical uncertainty, not a deterministic
latency bound. Applying a bridge changes the diagnostic interpolation query only.
It does not certify the optical exposure convention or remove survey blockers.
Use the [optical procedure](optical-target-acceptance.md) for those measurements.
