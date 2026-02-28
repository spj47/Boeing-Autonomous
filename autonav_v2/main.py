# Import libraries
import json
import os
import time
from typing import Any, Dict, List, Optional, Tuple
import yaml

# Serial IO (not used in push_test, but kept for bench/road later)
from serial_io.brake import BrakeSerial
from serial_io.steering import SteeringSerial
from serial_io.throttle import ThrottleSerial

# Navigation / Control logic
from navigation.cone_frame import FrameConfig
from navigation.cone_target import ConeNavConfig, compute_target_from_cones
from navigation.pure_pursuit import steer_deg_from_target, steer_percent_from_deg
from navigation.speed_rules import SpeedConfig, compute_speed_commands

# Cone sources (file replay and live sensor pipeline)
from sensors.cone_source_file import FileConeSource
from sensors.cone_source_live import LiveConeSource

# New sensor sources (camera / lidar / fusion)
from sensors.cone_source_camera import CameraConeSource, CameraConeConfig
from sensors.cone_source_lidar import LidarConeSource, LidarConeConfig
from sensors.cone_source_fusion import FusionConeSource, FusionConfig


JsonDict = Dict[str, Any]
ConeList = List[Any]


def load_config(config_path: str) -> JsonDict:
    # Load YAML so behavior can be tuned without touching code
    with open(config_path, "r", encoding="utf-8") as file_handle:
        return yaml.safe_load(file_handle)


class JsonLinesLogger:
    # Write one JSON object per line so logs survive crashes and stream cleanly
    def __init__(self, log_path: str) -> None:
        logs_folder = os.path.dirname(log_path)
        if logs_folder:
            os.makedirs(logs_folder, exist_ok=True)

        # Overwrite each run so the current test is always easy to find
        self._file_handle = open(log_path, "w", encoding="utf-8")

    def write(self, record: JsonDict) -> None:
        self._file_handle.write(json.dumps(record) + "\n")
        self._file_handle.flush()

    def close(self) -> None:
        try:
            self._file_handle.close()
        except Exception:
            pass


def now_wall_time_s() -> float:
    # Wall time is useful for timestamps humans will read later
    return time.time()


def now_monotonic_s() -> float:
    # Monotonic time is stable for dt measurements and loop timing
    return time.monotonic()


def read_cones_with_metadata(
    cone_source: Any,
) -> Tuple[ConeList, Optional[float], Optional[str]]:
    # Try to read richer live metadata first so we can enforce freshness rules
    if hasattr(cone_source, "get_frame"):
        try:
            cone_boxes, frame_time_s, frame_id = cone_source.get_frame()
            return cone_boxes, frame_time_s, None if frame_id is None else str(frame_id)
        except Exception:
            # If a live read fails, fall back to basic reads so the system keeps running
            pass

    # Fall back to basic cone reads when metadata is not available
    cone_boxes = cone_source.get_cones()
    return cone_boxes, None, None


def serialize_cones(cones: ConeList) -> List[JsonDict]:
    # Convert cones into a consistent shape so logs are easy to parse later
    serialized_cones: List[JsonDict] = []

    # Walk each cone and normalize formats into the same dict keys
    for cone_box in cones:
        # If it looks like a tuple/list, assume it is (xc, yc, zc, w, d, h)
        if isinstance(cone_box, (list, tuple)) and len(cone_box) >= 6:
            serialized_cones.append(
                {
                    "xc": float(cone_box[0]),
                    "yc": float(cone_box[1]),
                    "zc": float(cone_box[2]),
                    "w": float(cone_box[3]),
                    "d": float(cone_box[4]),
                    "h": float(cone_box[5]),
                }
            )
            continue

        # If it is an object with fields, convert it directly
        if hasattr(cone_box, "__dict__"):
            serialized_cones.append(dict(cone_box.__dict__))
            continue

        # If it is already a dict-like object, pass it through
        if isinstance(cone_box, dict):
            serialized_cones.append(dict(cone_box))
            continue

        # Last resort so the log never breaks on a weird cone payload
        serialized_cones.append({"raw": str(cone_box)})

    return serialized_cones


def main() -> None:
    # Load configuration early so all behavior is controlled in one place
    config = load_config("config.yaml")

    # Decide which mode we are running so we can enforce safety rules
    mode = str(config.get("mode", "bench")).lower().strip()
    dry_run_from_config = bool(config.get("dry_run", False))

    # In push_test we always disable actuation even if someone edits config incorrectly
    if mode == "push_test":
        dry_run = True
    else:
        dry_run = dry_run_from_config

    # Set loop timing so we react to sensor updates consistently
    loop_hz = float(config.get("loop_hz", 20))
    loop_period_s = 1.0 / max(1.0, loop_hz)

    # Configure logging so we can confirm results without moving the kart
    logging_config = (
        config.get("logging", {})
        if isinstance(config.get("logging", {}), dict)
        else {}
    )

    # Enable logs by default in push_test because logs are the entire point of the run
    logging_enabled = bool(logging_config.get("enable", mode == "push_test"))
    log_path = str(logging_config.get("path", "logs/push_test.jsonl"))

    write_every_n = int(logging_config.get("write_every_n", 1))
    if write_every_n < 1:
        write_every_n = 1

    if logging_enabled:
        logger: Optional[JsonLinesLogger] = JsonLinesLogger(log_path)
    else:
        logger = None

    # Apply health rules so stale or insane frames do not produce bogus commands
    health_config = (
        config.get("health", {})
        if isinstance(config.get("health", {}), dict)
        else {}
    )

    sensor_timeout_ms = int(health_config.get("sensor_timeout_ms", 200))

    max_cones = int(health_config.get("max_cones", 50))
    if max_cones < 1:
        max_cones = 1

    # Configure the coordinate frame mapping we will validate during the push test
    frame_config = FrameConfig(
        forward_axis=str(config["frame"]["forward_axis"]),
        forward_sign=int(config["frame"]["forward_sign"]),
        right_sign=int(config["frame"]["right_sign"]),
    )

    # Configure how we choose a target point from cone detections
    cone_nav_config = ConeNavConfig(
        fwd_min_m=float(config["cone_nav"]["fwd_min_m"]),
        fwd_max_m=float(config["cone_nav"]["fwd_max_m"]),
        corridor_width_m=float(config["cone_nav"]["corridor_width_m"]),
        target_fwd_m=float(config["cone_nav"]["target_fwd_m"]),
        use_nearest_n=int(config["cone_nav"]["use_nearest_n"]),
    )

    # Lookahead controls how "far ahead" pure pursuit aims
    lookahead_m = float(config["pure_pursuit"]["lookahead_m"])

    # Steering mapping tells us how to convert degrees into 0..100 percent
    max_wheel_deg = float(config["steering"]["max_wheel_deg"])
    steering_invert = bool(config["steering"]["invert"])

    # Speed rules compute throttle/brake decisions we will log during push testing
    speed_config = SpeedConfig(
        base_throttle_pct=int(config["speed"]["base_throttle_pct"]),
        min_throttle_pct=int(config["speed"]["min_throttle_pct"]),
        max_throttle_pct=int(config["speed"]["max_throttle_pct"]),
        steering_slow_gain=float(config["speed"]["steering_slow_gain"]),
        enable_emergency_stop=bool(config["speed"]["enable_emergency_stop"]),
        estop_corridor_half_width_m=float(config["speed"]["estop_corridor_half_width_m"]),
        estop_distance_m=float(config["speed"]["estop_distance_m"]),
        slow_distance_m=float(config["speed"]["slow_distance_m"]),
        brake_pct_estop=int(config["speed"]["brake_pct_estop"]),
        brake_pct_slow=int(config["speed"]["brake_pct_slow"]),
    )

    # Select where cones come from so we can swap between replay and real sensors
    cone_source_config = config.get("cone_source", {})
    cone_source_type = str(cone_source_config.get("type", "file")).lower().strip()

    if cone_source_type == "file":
        # File mode replays a known sequence so we can debug logic without hardware
        cone_source = FileConeSource(str(cone_source_config["file_path"]))

    elif cone_source_type == "live":
        # Live mode reads detections from the running perception pipeline
        live_config = (
            cone_source_config.get("live", {})
            if isinstance(cone_source_config.get("live", {}), dict)
            else {}
        )
        cone_source = LiveConeSource(live_config)

    elif cone_source_type == "camera":
        # Camera mode runs YOLO on-device and converts detections into cone boxes
        camera_raw = (
            cone_source_config.get("camera", {})
            if isinstance(cone_source_config.get("camera", {}), dict)
            else {}
        )

        camera_cfg = CameraConeConfig(
            camera_height_m=float(camera_raw.get("camera_height_m", 0.8)),
            camera_pitch_deg=float(camera_raw.get("camera_pitch_deg", 0.0)),
            horizontal_fov_deg=float(camera_raw.get("horizontal_fov_deg", 70.0)),
            vertical_fov_deg=float(camera_raw.get("vertical_fov_deg", 55.0)),
            assumed_cone_w_m=float(camera_raw.get("assumed_cone_w_m", 0.15)),
            assumed_cone_d_m=float(camera_raw.get("assumed_cone_d_m", 0.15)),
            assumed_cone_h_m=float(camera_raw.get("assumed_cone_h_m", 0.0)),
            min_forward_m=float(camera_raw.get("min_forward_m", 0.25)),
            max_forward_m=float(camera_raw.get("max_forward_m", 30.0)),
        )

        model_path = camera_raw.get("model_path", None)
        rgb = bool(camera_raw.get("rgb", True))
        cone_source = CameraConeSource(model_path=model_path, rgb=rgb, config=camera_cfg)

    elif cone_source_type == "lidar":
        # LiDAR mode clusters LD14 points and outputs object boxes as cone boxes
        lidar_raw = (
            cone_source_config.get("lidar", {})
            if isinstance(cone_source_config.get("lidar", {}), dict)
            else {}
        )

        lidar_cfg = LidarConeConfig(
            port=str(lidar_raw.get("port", "/dev/ttyUSB0")),
            baudrate=int(lidar_raw.get("baudrate", 230400)),
            min_cluster_size=int(lidar_raw.get("min_cluster_size", 5)),
            cluster_distance_threshold=float(
                lidar_raw.get("cluster_distance_threshold", 0.08)
            ),
            min_range_m=float(lidar_raw.get("min_range_m", 0.20)),
            max_range_m=float(lidar_raw.get("max_range_m", 30.0)),
            max_box_width_m=float(lidar_raw.get("max_box_width_m", 1.5)),
            max_box_depth_m=float(lidar_raw.get("max_box_depth_m", 1.5)),
        )

        cone_source = LidarConeSource(config=lidar_cfg)

    elif cone_source_type == "fusion":
        # Fusion mode uses camera detections to label LiDAR clusters as cones
        fusion_raw = (
            cone_source_config.get("fusion", {})
            if isinstance(cone_source_config.get("fusion", {}), dict)
            else {}
        )

        fusion_cfg = FusionConfig(
            lidar_port=str(fusion_raw.get("lidar_port", "/dev/ttyUSB0")),
            cones_only=bool(fusion_raw.get("cones_only", True)),
            min_range_m=float(fusion_raw.get("min_range_m", 0.20)),
            max_range_m=float(fusion_raw.get("max_range_m", 30.0)),
            max_cones=int(fusion_raw.get("max_cones", 50)),
        )

        cone_source = FusionConeSource(config=fusion_cfg)

    else:
        raise ValueError(f"Unknown cone_source.type: {cone_source_type}")

    # Prepare serial interfaces for later, but keep them unused during push_test
    steering_serial: Optional[SteeringSerial] = None
    throttle_serial: Optional[ThrottleSerial] = None
    brake_serial: Optional[BrakeSerial] = None

    if not dry_run:
        # Only open serial when we are actually allowed to send commands
        baud_rate = int(config["serial"]["baud"])
        steering_port = str(config["serial"]["steering_port"])
        throttle_port = str(config["serial"]["throttle_port"])
        brake_port = str(config["serial"]["brake_port"])

        steering_serial = SteeringSerial(steering_port, baud_rate)
        throttle_serial = ThrottleSerial(throttle_port, baud_rate)
        brake_serial = BrakeSerial(brake_port, baud_rate)

        # Start safe so any accidental runtime issue still leaves the kart stable
        steering_serial.set_percent(50)
        throttle_serial.set_percent(0)
        brake_serial.set_percent(0)

        print(
            f"autonav_v2 RUNNING (LIVE SERIAL) mode={mode} source={cone_source_type}"
        )
    else:
        # Push test uses full logic but never touches actuators
        print(
            "autonav_v2 RUNNING (PUSH TEST / DRY RUN) "
            f"mode={mode} source={cone_source_type}"
        )

    print(f"Logging: {'ON' if logger else 'OFF'} -> {log_path}")

    if mode == "push_test":
        print("Push-test safety: ACTUATION DISABLED (throttle/brake sent = 0)")

    # Track loop timing and print rate so the console stays readable
    last_console_print_time_s = 0.0
    loop_count = 0
    previous_loop_time_s = now_monotonic_s()

    try:
        # Loop forever so the kart continuously reacts to incoming sensor data
        while True:
            # Start the loop timer so we can enforce a stable loop rate
            loop_start_time_s = now_monotonic_s()
            wall_time_s = now_wall_time_s()

            # Read cones from the selected source and keep metadata if available
            cone_boxes, frame_time_s, frame_id = read_cones_with_metadata(cone_source)

            # If the sensor gives us nothing, treat it as empty so we stay predictable
            if cone_boxes is None:
                cone_boxes = []

            # Cap cone count so a bad frame cannot explode our logs or computations
            if len(cone_boxes) > max_cones:
                cone_boxes = cone_boxes[:max_cones]

            # Check freshness so old sensor data does not drive new decisions
            sensor_age_ms: Optional[int] = None
            sensor_frame_ok = True

            if frame_time_s is not None:
                sensor_age_ms = int((wall_time_s - float(frame_time_s)) * 1000.0)

                # If the frame is too old, treat it like "no cones"
                if sensor_age_ms > sensor_timeout_ms:
                    sensor_frame_ok = False

            if not sensor_frame_ok:
                # Clearing cones forces safe behavior and keeps logs honest
                cone_boxes = []

            # Compute a target point in the vehicle frame so steering has something to aim at
            target_point_m = compute_target_from_cones(
                cone_boxes,
                frame_config,
                cone_nav_config,
            )

            # If we have no target, bias to safe commands we can log and review later
            if target_point_m is None:
                steering_deg: Optional[float] = None
                steering_pct = 50

                # No cones means we should stop rather than guess
                throttle_pct_would = 0
                brake_pct_would = 60
            else:
                # Split out target coordinates so the math stays readable
                target_right_m, target_forward_m = target_point_m

                # Compute steering angle needed to drive toward the target point
                steering_deg = steer_deg_from_target(
                    target_right_m,
                    target_forward_m,
                    lookahead_m,
                )

                # Convert steering degrees into a 0..100 command we can send to the Arduino
                steering_pct = steer_percent_from_deg(
                    steering_deg,
                    max_wheel_deg,
                    steering_invert,
                )

                # Compute what throttle/brake would be based on steering demand and safety rules
                throttle_pct_would, brake_pct_would = compute_speed_commands(
                    steer_deg=steering_deg,
                    cones_xyzwdh=cone_boxes,
                    frame_cfg=frame_config,
                    spd_cfg=speed_config,
                )

            # Push-test only: we always send zeros so the kart cannot move
            if mode == "push_test":
                throttle_pct_sent = 0
                brake_pct_sent = 0
            else:
                # In bench/road mode these become the real actuator outputs
                throttle_pct_sent = int(throttle_pct_would)
                brake_pct_sent = int(brake_pct_would)

            if not dry_run:
                # In bench/road mode these become the real actuator outputs
                steering_serial.set_percent(int(steering_pct))
                throttle_serial.set_percent(int(throttle_pct_sent))
                brake_serial.set_percent(int(brake_pct_sent))

            # Measure dt so we can verify performance and compute measured loop rate
            current_loop_time_s = now_monotonic_s()
            dt_s = current_loop_time_s - previous_loop_time_s
            previous_loop_time_s = current_loop_time_s

            # Log every N loops so we can trade log size for detail when needed
            if logger is not None and (loop_count % write_every_n == 0):
                # Build one record that fully explains what the system saw and decided
                record: JsonDict = {
                    "t_wall_s": wall_time_s,
                    "dt_s": dt_s,
                    "loop_hz_meas": (1.0 / dt_s) if dt_s > 1e-6 else None,
                    "mode": mode,
                    "dry_run": dry_run,
                    "sensor": {
                        "source": cone_source_type,
                        "frame_id": frame_id,
                        "age_ms": sensor_age_ms,
                        "ok": sensor_frame_ok,
                    },
                    "cones_raw": serialize_cones(cone_boxes),
                    "nav": {
                        "target_point_m": None
                        if target_point_m is None
                        else {
                            "right": float(target_point_m[0]),
                            "fwd": float(target_point_m[1]),
                        },
                        "steer_deg": None
                        if steering_deg is None
                        else float(steering_deg),
                        "steer_pct": int(steering_pct),
                    },
                    "cmd": {
                        "sent": {
                            "steer_pct": int(steering_pct),
                            "throttle_pct": int(throttle_pct_sent),
                            "brake_pct": int(brake_pct_sent),
                        },
                        "would": {
                            "throttle_pct": int(throttle_pct_would),
                            "brake_pct": int(brake_pct_would),
                        },
                    },
                }
                logger.write(record)

            # Print status occasionally so we can sanity-check during a field run
            if (loop_start_time_s - last_console_print_time_s) > 0.5:
                last_console_print_time_s = loop_start_time_s

                if steering_deg is None:
                    print(
                        "cones={} age_ms={} ok={} target=None steer={} "
                        "would_thr={} would_brk={}".format(
                            len(cone_boxes),
                            sensor_age_ms,
                            sensor_frame_ok,
                            steering_pct,
                            throttle_pct_would,
                            brake_pct_would,
                        )
                    )
                else:
                    print(
                        "cones={} age_ms={} ok={} target={} steer_deg={:.2f} "
                        "steer={} would_thr={} would_brk={}".format(
                            len(cone_boxes),
                            sensor_age_ms,
                            sensor_frame_ok,
                            target_point_m,
                            steering_deg,
                            steering_pct,
                            throttle_pct_would,
                            brake_pct_would,
                        )
                    )

            # Sleep the remainder of the loop so our loop_hz stays stable
            elapsed_s = now_monotonic_s() - loop_start_time_s
            if elapsed_s < loop_period_s:
                time.sleep(loop_period_s - elapsed_s)

            loop_count += 1

    except KeyboardInterrupt:
        # Catch Ctrl+C so we can shut down cleanly and leave the system safe
        if not dry_run:
            print("\nStopping: throttle STOP, brake=100, steer center")

            # Try each safety command independently so one failure does not block the rest
            try:
                throttle_serial.stop()
            except Exception:
                pass

            try:
                throttle_serial.set_percent(0)
            except Exception:
                pass

            try:
                brake_serial.set_percent(100)
            except Exception:
                pass

            try:
                steering_serial.set_percent(50)
            except Exception:
                pass
        else:
            print("\nStopped (push test / dry run)")

    finally:
        # Always close logs so we do not lose buffered data
        if logger is not None:
            logger.close()

        # If the cone source has cleanup, call it to release sockets/ports/files
        if hasattr(cone_source, "close"):
            try:
                cone_source.close()
            except Exception:
                pass

        # Close serial ports if they were opened
        if steering_serial is not None:
            try:
                steering_serial.close()
            except Exception:
                pass

        if throttle_serial is not None:
            try:
                throttle_serial.close()
            except Exception:
                pass

        if brake_serial is not None:
            try:
                brake_serial.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()