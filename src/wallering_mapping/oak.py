"""DepthAI 3.10 adapter. All hardware access is isolated in this module."""

import queue
import signal
import sys
import threading
import time
from contextlib import contextmanager

from .dataset import AsyncWriter, Session
from .operations import host_health
from .timing import ClockTracker, sample_clock


def nanoseconds(delta):
    # timedelta is microsecond-resolution; avoid floating-point conversion.
    return ((delta.days * 86400 + delta.seconds) * 1000000 + delta.microseconds) * 1000


def depthai():
    import depthai as dai

    if dai.__version__ != "3.10.0":
        raise RuntimeError(f"Expected depthai 3.10.0, found {dai.__version__}; install .[oak]")
    return dai


def device_details(device, dai):
    imu_type = device.getConnectedIMU()
    has_imu = bool(imu_type and imu_type.upper() not in {"NONE", "UNKNOWN"})
    return {
        "id": device.getDeviceId(), "depthai_version": dai.__version__,
        "usb_speed": str(device.getUsbSpeed()), "imu_type": imu_type,
        "imu_firmware": str(device.getIMUFirmwareVersion()) if has_imu else None,
        "imu_firmware_embedded": str(device.getEmbeddedIMUFirmwareVersion()) if has_imu else None,
        "cameras": [{"socket": str(f.socket), "sensor": f.sensorName,
                     "width": f.width, "height": f.height, "autofocus": f.hasAutofocus}
                    for f in device.getConnectedCameraFeatures()],
    }


def inspect_device(device_id=None):
    dai = depthai()
    with dai.Device(device_id) if device_id else dai.Device() as device:
        return device_details(device, dai)


@contextmanager
def stop_signals():
    stop = threading.Event()
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, lambda *_: stop.set())
    try:
        yield stop
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def build_pipeline(device, dai, config):
    pipeline = dai.Pipeline(device)
    features = {f.socket: f for f in device.getConnectedCameraFeatures()}
    calibration = device.readCalibration()
    sockets = {"rgb": dai.CameraBoardSocket.CAM_A, "left": dai.CameraBoardSocket.CAM_B,
               "right": dai.CameraBoardSocket.CAM_C}
    queues, settings = {}, {}
    for stream in config.streams:
        socket = sockets[stream]
        if socket not in features:
            raise ValueError(f"Missing {stream}/{socket}. Run inspect and select available streams")
        camera = pipeline.create(dai.node.Camera).build(socket, sensorFps=config.fps)
        control = camera.initialControl
        control.setManualExposure(config.exposure_us, config.iso)
        lens = None
        if stream == "rgb":
            control.setManualWhiteBalance(config.white_balance_k)
            if features[socket].hasAutofocus:
                lens = config.lens_position
                if lens is None:
                    lens = calibration.getLensPosition(socket)
                if not 0 <= lens <= 255:
                    raise ValueError("No valid factory focus; set lens_position after a focus test")
                control.setAutoFocusMode(dai.CameraControl.AutoFocusMode.OFF)
                control.setManualFocus(lens)
        output = camera.requestFullResolutionOutput(
            type=dai.ImgFrame.Type.NV12 if stream == "rgb" else dai.ImgFrame.Type.GRAY8,
            fps=config.fps,
        )
        # Full FOV, no resize/rectification/undistortion requested. Capture all streams
        # independently: a Sync node would hide unmatched frames from the archive.
        queues[stream] = output.createOutputQueue(maxSize=4, blocking=True)
        settings[stream] = {"socket": str(socket), "lens_position_requested": lens}
    imu_type = device.getConnectedIMU()
    has_imu = bool(imu_type and imu_type.upper() not in {"NONE", "UNKNOWN"})
    if config.imu == "required" and not has_imu:
        raise ValueError("IMU required, but this device reports none")
    if has_imu and config.imu != "off":
        check_imu_firmware(device)
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, config.imu_hz)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, config.imu_hz)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        queues["imu"] = imu.out.createOutputQueue(maxSize=50, blocking=True)
    return pipeline, queues, calibration.eepromToJson(), settings


def check_imu_firmware(device):
    # DepthAI 3.10 rejects the old BNO086 3.2.13 firmware on the commissioned OAK.
    # Keep the pinned SDK's bundled BNO firmware as the deployment baseline.
    if device.getConnectedIMU().upper() in {"BNO085", "BNO086"}:
        installed = str(device.getIMUFirmwareVersion())
        embedded = str(device.getEmbeddedIMUFirmwareVersion())
        if installed != embedded:
            raise RuntimeError(f"BNO IMU firmware {installed} differs from the pinned DepthAI "
                               f"baseline {embedded}; commission the IMU firmware before capture. "
                               "Startup checks never flash firmware. Use imu=off only for an "
                               "explicit camera-only profile.")


def frame_metadata(message, dai):
    transform = message.getTransformation()
    if not transform.isValid():
        raise ValueError("Frame lacks valid pixel calibration transformation")
    return {
        "sequence": message.getSequenceNum(),
        "device_ns": nanoseconds(message.getTimestampDevice(dai.CameraExposureOffset.MIDDLE)),
        "host_synced_ns": nanoseconds(message.getTimestamp(dai.CameraExposureOffset.MIDDLE)),
        "received_monotonic_ns": time.monotonic_ns(), "received_utc_ns": time.time_ns(),
        "exposure_us": nanoseconds(message.getExposureTime()) // 1000,
        "iso": message.getSensitivity(), "lens_position": message.getLensPosition(),
        "white_balance_k": message.getColorTemperature(),
        "camera": {"K": transform.getIntrinsicMatrix(),
                   "distortion": transform.getDistortionCoefficients(),
                   "model": str(transform.getDistortionModel()),
                   "source_width": message.getSourceWidth(),
                   "source_height": message.getSourceHeight(),
                   "processing": "full_fov_no_undistortion_requested"},
    }


@contextmanager
def telemetry_subscriber(config):
    if config is None:
        yield None
        return
    from .mavros import MavrosSubscriber
    source = MavrosSubscriber(config)
    try:
        yield source
    finally:
        source.close()


def record(root, config, duration=None, device_id=None, telemetry_config=None):
    dai = depthai()
    with (dai.Device(device_id) if device_id else dai.Device()) as device, telemetry_subscriber(telemetry_config) as telemetry:
        details = device_details(device, dai)
        if device.getUsbSpeed() not in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS):
            raise RuntimeError("USB 3 is required for this full-resolution recorder; check cable/port")
        pipeline, queues, calibration, settings = build_pipeline(device, dai, config)
        details.update(stream_settings=settings, imu_enabled="imu" in queues)
        telemetry_contract = None if telemetry is None else {
            "schema_version": 1, "adapter": "mavros_ros2", "receive_only": True,
            "config": telemetry_config.to_dict(), "parameters": telemetry.buffer.parameters,
            "frames": "MAVROS local pose ENU, body FLU; OAK IMU sensor-native",
            "altitude": "MAVROS NavSatFix WGS84 ellipsoidal height; not AMSL",
            "timestamp": "Original MAVROS ROS header plus bracketed receipt clocks; not raw PX4 timestamps",
            "limitations": "DDS loss not observable from header; timing qualification is an evidence gate",
        }
        session = Session(root, config, "oak", details, calibration, telemetry_contract)
        writer = AsyncWriter(session)
        status, reason, failure = "complete", "requested stop", None
        try:
            with stop_signals() as stop:
                pipeline.start()
                if telemetry:
                    telemetry.start()
                start = time.monotonic()
                ready = start + config.warmup_seconds
                last_received = dict.fromkeys(queues, start)
                last_imu = {}
                imu_received = dict.fromkeys(("accelerometer", "gyroscope"), ready)
                clock_due = telemetry_due = start
                progress_due = start
                clock_tracker = ClockTracker(telemetry_config.clock_jump_ms if telemetry_config else 5)
                while not stop.is_set():
                    now = time.monotonic()
                    if duration is not None and now >= ready + duration:
                        reason = "duration reached"
                        break
                    if not pipeline.isRunning():
                        raise RuntimeError("Camera pipeline stopped unexpectedly")
                    writer.check()
                    if telemetry:
                        telemetry.buffer.check()
                        if now >= telemetry_due:
                            batch = telemetry.buffer.drain()
                            if batch:
                                writer.submit("telemetry_batch", batch)
                            telemetry_due = now + .2
                    for name, q in queues.items():
                        rows = []
                        # Limit work per queue so images cannot starve the IMU or stop checks.
                        for _ in range(16):
                            message = q.tryGet()
                            if message is None:
                                break
                            last_received[name] = time.monotonic()
                            if now < ready:
                                continue
                            if name != "imu":
                                metadata = frame_metadata(message, dai)
                                writer.submit("frame", name, message.getCvFrame(), metadata)
                            else:
                                for packet in message.packets:
                                    for sensor, report, unit in (
                                        ("accelerometer", packet.acceleroMeter, "m/s^2"),
                                        ("gyroscope", packet.gyroscope, "rad/s"),
                                    ):
                                        sequence = report.getSequenceNum()
                                        if last_imu.get(sensor) == sequence:
                                            continue
                                        if sensor in last_imu and sequence < last_imu[sensor]:
                                            raise ValueError(f"{sensor} sequence reset")
                                        last_imu[sensor] = sequence
                                        imu_received[sensor] = time.monotonic()
                                        rows.append({"sensor": sensor, "sequence": sequence,
                                                     "device_ns": nanoseconds(report.getTimestampDevice()),
                                                     "host_synced_ns": nanoseconds(report.getTimestamp()),
                                                     "received_monotonic_ns": time.monotonic_ns(),
                                                     "received_utc_ns": time.time_ns(),
                                                     "xyz": [report.x, report.y, report.z],
                                                     "unit": unit, "frame": "sensor_native",
                                                     "report": "RAW"})
                        if rows:
                            writer.submit("imu_batch", rows)
                    if now >= clock_due:
                        observation = sample_clock(lambda: nanoseconds(dai.Clock.now()), "depthai_steady")
                        clock_tracker.observe(observation)
                        observation["host_utc_ns"] = time.time_ns()
                        writer.submit("append", "clock", observation)
                        clock_due = now + (1 / telemetry_config.clock_sample_hz if telemetry_config else 1)
                    for name, received in last_received.items():
                        if now - received > config.stall_seconds:
                            raise RuntimeError(f"Stream {name} stalled")
                    if now >= progress_due:
                        health = host_health(root)
                        if telemetry:
                            health["telemetry"] = telemetry.buffer.summary()
                        writer.submit("heartbeat", "warmup" if now < ready else "recording",
                                      health, writer.queue.qsize())
                        print(f"recording {root}: saved={dict(session.counts)} "
                              f"writer_queue={writer.queue.qsize()}", file=sys.stderr, flush=True)
                        progress_due = now + 5
                    if "imu" in queues and now >= ready:
                        for sensor, received in imu_received.items():
                            if now - received > config.stall_seconds:
                                raise RuntimeError(f"IMU report {sensor} stalled")
                    time.sleep(0.002)
        except BaseException as error:
            status, reason, failure = "failed", str(error), error
        finally:
            # Stop callbacks before draining accepted data into the one dataset writer.
            if telemetry:
                try:
                    telemetry.close()
                    telemetry.buffer.finish_check()
                except Exception as error:
                    status, reason, failure = "failed", str(error), error
                session.manifest["telemetry"]["summary"] = telemetry.buffer.summary()
                try:
                    while not telemetry.buffer.queue.empty():
                        batch = telemetry.buffer.drain(check=False)
                        # A bounded blocking handoff during shutdown drains accepted records.
                        writer.check()
                        while writer.thread.is_alive():
                            writer.check()
                            try:
                                writer.queue.put(("telemetry_batch", (batch,)), timeout=.1)
                                break
                            except queue.Full:
                                pass
                except Exception as error:
                    status, reason, failure = "failed", str(error), error
            # Hardware queue tails at the stop boundary are outside the accepted interval.
            try:
                writer.close()
            except Exception as error:
                status, reason, failure = "failed", str(error), error
            try:
                pipeline.stop()
            except Exception as error:
                status, reason, failure = "failed", str(error), error
            if failure is None and any(session.counts[s] == 0 for s in config.streams):
                status, reason = "failed", "One or more requested streams have no saved images"
                failure = failure or RuntimeError(reason)
            if failure is None and "imu" in queues and any(session.counts[s] == 0 for s in ("accelerometer", "gyroscope")):
                status, reason = "failed", "Enabled IMU has no samples for one or more sensors"
                failure = failure or RuntimeError(reason)
            session.finish(status, reason)
        if failure:
            raise RuntimeError(reason) from failure
        return session.manifest
