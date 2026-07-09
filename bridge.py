from __future__ import annotations

import argparse
import logging
import math
import time
from dataclasses import dataclass, field

from foxglove import Channel, set_log_level, start_server
from foxglove.channels import (
    FrameTransformChannel,
    LocationFixChannel,
    PoseInFrameChannel,
)
from foxglove.messages import (
    FrameTransform,
    LocationFix,
    Pose,
    PoseInFrame,
    Quaternion,
    Timestamp,
    Vector3,
)
from pymavlink import mavutil
import serial
from serial.tools import list_ports


LOGGER = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_SERIAL_PORT = None
DEFAULT_BAUD = 115_200
DEFAULT_UDP_ADDRESS = "127.0.0.1:14550"
DEFAULT_FOXGLOVE_HOST = "127.0.0.1"
DEFAULT_FOXGLOVE_PORT = 8765
DEFAULT_RECONNECT_DELAY_S = 2.0
DEFAULT_SERIAL_TIMEOUT_S = 1.0
STATS_LOG_INTERVAL_S = 5.0

# MAVLink message types we care about
MAVLINK_MESSAGES_OF_INTEREST = {
    "ATTITUDE",
    "GLOBAL_POSITION_INT",
    "GPS_RAW_INT",
    "HEARTBEAT",
}


# ---------------------------------------------------------------------------
# NED -> ENU conversion helpers
# ---------------------------------------------------------------------------

def euler_ned_to_quaternion_enu(roll_ned: float, pitch_ned: float, yaw_ned: float) -> Quaternion:
    """Convert MAVLink NED euler angles (radians) to an ENU quaternion.

    MAVLink ATTITUDE uses NED frame:
      - roll  = rotation about North axis
      - pitch = rotation about East axis
      - yaw   = rotation about Down axis (0 = North, pi/2 = East)

    Foxglove 3D panel expects ENU:
      - x = East, y = North, z = Up

    Conversion from NED to ENU euler angles:
      roll_enu  =  pitch_ned
      pitch_enu =  roll_ned
      yaw_enu   = -yaw_ned + pi/2   (rotate CW -> CCW, North -> East offset)

    Then convert ENU euler angles to quaternion using ZYX intrinsic rotation.
    """
    roll_enu = pitch_ned
    pitch_enu = roll_ned
    yaw_enu = -yaw_ned + math.pi / 2.0

    # ZYX intrinsic (yaw, pitch, roll) to quaternion
    cy = math.cos(yaw_enu * 0.5)
    sy = math.sin(yaw_enu * 0.5)
    cp = math.cos(pitch_enu * 0.5)
    sp = math.sin(pitch_enu * 0.5)
    cr = math.cos(roll_enu * 0.5)
    sr = math.sin(roll_enu * 0.5)

    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class BridgeConfig:
    """Runtime configuration for the MAVLink–Foxglove bridge."""

    # Transport – exactly one of serial_port / udp_address should be set.
    serial_port: str | None = DEFAULT_SERIAL_PORT
    baud: int = DEFAULT_BAUD
    udp_address: str | None = None

    # Foxglove server
    foxglove_host: str = DEFAULT_FOXGLOVE_HOST
    foxglove_port: int = DEFAULT_FOXGLOVE_PORT

    # Resilience
    reconnect_delay_s: float = DEFAULT_RECONNECT_DELAY_S
    serial_timeout_s: float = DEFAULT_SERIAL_TIMEOUT_S

    @property
    def connection_string(self) -> str:
        """Return the pymavlink connection string for the configured transport."""
        if self.udp_address is not None:
            return f"udpin:{self.udp_address}"
        assert self.serial_port is not None
        return self.serial_port

    @property
    def transport_label(self) -> str:
        if self.udp_address is not None:
            return f"UDP {self.udp_address}"
        return f"Serial {self.serial_port} @ {self.baud} baud"


# ---------------------------------------------------------------------------
# Stats
# ---------------------------------------------------------------------------

@dataclass
class BridgeStats:
    messages_received: int = 0
    messages_published: int = 0
    heartbeats: int = 0
    parse_errors: int = 0
    last_report_monotonic_s: float = 0.0

    def needs_report(self) -> bool:
        now = time.monotonic()
        if self.last_report_monotonic_s == 0.0:
            self.last_report_monotonic_s = now
            return False
        if now - self.last_report_monotonic_s < STATS_LOG_INTERVAL_S:
            return False
        self.last_report_monotonic_s = now
        return True


# ---------------------------------------------------------------------------
# Serial port helpers (same UX as reference)
# ---------------------------------------------------------------------------

def available_ports() -> list[list_ports.ListPortInfo]:
    return sorted(list_ports.comports(), key=lambda p: p.device)


def available_port_lines() -> list[str]:
    ports = available_ports()
    if not ports:
        return ["No serial ports detected."]
    return [f"{port.device}: {port.description}" for port in ports]


def prompt_port_selection() -> str | None:
    ports = available_ports()
    if not ports:
        print("No serial ports detected. Connect the device and retry.")
        return None

    print("Available serial ports:")
    for index, port in enumerate(ports, start=1):
        print(f"  [{index}] {port.device}: {port.description}")

    while True:
        try:
            raw = input(f"Select port [1-{len(ports)}]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return None

        if not raw:
            continue

        try:
            choice = int(raw)
        except ValueError:
            print(f"  Enter a number between 1 and {len(ports)}.")
            continue

        if 1 <= choice <= len(ports):
            selected = ports[choice - 1].device
            print(f"  Using {selected}")
            return selected

        print(f"  Enter a number between 1 and {len(ports)}.")


# ---------------------------------------------------------------------------
# Timestamp helper
# ---------------------------------------------------------------------------

def time_usec_to_timestamp(time_usec: int) -> Timestamp:
    """Convert a MAVLink microsecond timestamp to a Foxglove Timestamp."""
    return Timestamp(sec=time_usec // 1_000_000, nsec=(time_usec % 1_000_000) * 1_000)


def time_boot_ms_to_timestamp(time_boot_ms: int) -> Timestamp:
    """Convert a MAVLink boot-relative millisecond timestamp to a Foxglove Timestamp.

    We anchor boot time to wall-clock on first call so Foxglove can display it.
    """
    return Timestamp(sec=time_boot_ms // 1_000, nsec=(time_boot_ms % 1_000) * 1_000_000)


# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------

class MavlinkFoxgloveBridge:
    """Reads MAVLink from Serial or UDP and publishes to Foxglove."""

    def __init__(self, config: BridgeConfig) -> None:
        self._config = config
        self._stats = BridgeStats()

        # Well-known Foxglove channels (created once)
        self._pose_channel = PoseInFrameChannel("/uav/pose")
        self._tf_channel = FrameTransformChannel("/tf")
        self._gps_channel = LocationFixChannel("/uav/gps")

        # Raw JSON channels (created lazily)
        self._json_channels: dict[str, Channel] = {}

    # -- public entry point -------------------------------------------------

    def run(self) -> int:
        server = start_server(
            name="mavlink-foxglove-bridge",
            host=self._config.foxglove_host,
            port=self._config.foxglove_port,
            supported_encodings=["json"],
            session_id=f"mavlink-bridge:{self._config.transport_label}",
        )

        LOGGER.info(
            "Foxglove WebSocket server listening on ws://%s:%d",
            self._config.foxglove_host,
            server.port,
        )
        app_url = server.app_url(open_in_desktop=True)
        if app_url is not None:
            LOGGER.info("Open Foxglove with: %s", app_url)

        try:
            self._reconnect_loop()
        finally:
            self._pose_channel.close()
            self._tf_channel.close()
            self._gps_channel.close()
            for ch in self._json_channels.values():
                ch.close()
            server.stop()

        return 0

    # -- reconnection -------------------------------------------------------

    def _reconnect_loop(self) -> None:
        while True:
            try:
                self._run_session()
            except Exception as exc:
                LOGGER.warning(
                    "MAVLink connection lost (%s): %s",
                    self._config.transport_label,
                    exc,
                )
                LOGGER.info(
                    "Retrying in %.1f seconds",
                    self._config.reconnect_delay_s,
                )
                time.sleep(self._config.reconnect_delay_s)

    # -- main read loop -----------------------------------------------------

    def _run_session(self) -> None:
        conn = mavutil.mavlink_connection(
            self._config.connection_string,
            baud=self._config.baud,
        )

        LOGGER.info("Connected via %s", self._config.transport_label)
        LOGGER.info("Waiting for heartbeat...")
        conn.wait_heartbeat()
        LOGGER.info(
            "Heartbeat received (system %d, component %d)",
            conn.target_system,
            conn.target_component,
        )

        while True:
            msg = conn.recv_match(
                type=list(MAVLINK_MESSAGES_OF_INTEREST),
                blocking=True,
                timeout=self._config.serial_timeout_s,
            )
            if msg is None:
                self._maybe_report_stats()
                continue

            msg_type = msg.get_type()
            self._stats.messages_received += 1

            try:
                self._dispatch(msg_type, msg)
                self._stats.messages_published += 1
            except Exception as exc:
                self._stats.parse_errors += 1
                LOGGER.warning("Failed to process %s: %s", msg_type, exc)

            self._maybe_report_stats()

    # -- message dispatch ---------------------------------------------------

    def _dispatch(self, msg_type: str, msg: object) -> None:
        if msg_type == "ATTITUDE":
            self._handle_attitude(msg)
        elif msg_type == "GLOBAL_POSITION_INT":
            self._handle_global_position(msg)
        elif msg_type == "GPS_RAW_INT":
            self._handle_gps_raw(msg)
        elif msg_type == "HEARTBEAT":
            self._handle_heartbeat(msg)

    # -- ATTITUDE -> PoseInFrame + FrameTransform ---------------------------

    def _handle_attitude(self, msg: object) -> None:
        ts = time_boot_ms_to_timestamp(msg.time_boot_ms)
        orientation = euler_ned_to_quaternion_enu(msg.roll, msg.pitch, msg.yaw)

        # Publish PoseInFrame for the 3D panel
        self._pose_channel.log(
            PoseInFrame(
                timestamp=ts,
                frame_id="uav",
                pose=Pose(
                    position=Vector3(x=0.0, y=0.0, z=0.0),
                    orientation=orientation,
                ),
            ),
        )

        # Publish FrameTransform so the 3D panel can render the UAV frame
        self._tf_channel.log(
            FrameTransform(
                timestamp=ts,
                parent_frame_id="world",
                child_frame_id="uav",
                translation=Vector3(x=0.0, y=0.0, z=0.0),
                rotation=orientation,
            ),
        )

    # -- GLOBAL_POSITION_INT -> LocationFix + raw JSON ----------------------

    def _handle_global_position(self, msg: object) -> None:
        ts = time_boot_ms_to_timestamp(msg.time_boot_ms)

        # Foxglove LocationFix for the Map panel
        self._gps_channel.log(
            LocationFix(
                timestamp=ts,
                latitude=msg.lat / 1e7,
                longitude=msg.lon / 1e7,
                altitude=msg.alt / 1e3,
            ),
        )

        # Also publish as flat JSON for plotting individual fields
        self._publish_json("mavlink/global_position", {
            "time_boot_ms": msg.time_boot_ms,
            "lat_deg": msg.lat / 1e7,
            "lon_deg": msg.lon / 1e7,
            "alt_m": msg.alt / 1e3,
            "relative_alt_m": msg.relative_alt / 1e3,
            "vx_m_s": msg.vx / 100.0,
            "vy_m_s": msg.vy / 100.0,
            "vz_m_s": msg.vz / 100.0,
            "hdg_deg": msg.hdg / 100.0 if msg.hdg != 65535 else None,
        })

    # -- GPS_RAW_INT -> raw JSON --------------------------------------------

    def _handle_gps_raw(self, msg: object) -> None:
        self._publish_json("mavlink/gps_raw", {
            "time_usec": msg.time_usec,
            "fix_type": msg.fix_type,
            "lat_deg": msg.lat / 1e7,
            "lon_deg": msg.lon / 1e7,
            "alt_m": msg.alt / 1e3,
            "eph": msg.eph / 100.0 if msg.eph != 65535 else None,
            "epv": msg.epv / 100.0 if msg.epv != 65535 else None,
            "vel_m_s": msg.vel / 100.0 if msg.vel != 65535 else None,
            "satellites_visible": msg.satellites_visible,
        })

    # -- HEARTBEAT -> raw JSON ----------------------------------------------

    def _handle_heartbeat(self, msg: object) -> None:
        self._stats.heartbeats += 1

        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        self._publish_json("mavlink/heartbeat", {
            "type": msg.type,
            "autopilot": msg.autopilot,
            "base_mode": msg.base_mode,
            "custom_mode": msg.custom_mode,
            "system_status": msg.system_status,
            "armed": armed,
        })

    # -- JSON publishing helper ---------------------------------------------

    def _publish_json(self, topic: str, message: dict[str, object]) -> None:
        channel = self._json_channels.get(topic)
        if channel is None:
            channel = Channel(topic, message_encoding="json")
            self._json_channels[topic] = channel
            LOGGER.info("Advertising JSON topic: %s", topic)
        channel.log(message)

    # -- stats reporting ----------------------------------------------------

    def _maybe_report_stats(self) -> None:
        if not self._stats.needs_report():
            return
        LOGGER.info(
            "msgs_rx=%d msgs_pub=%d heartbeats=%d errors=%d",
            self._stats.messages_received,
            self._stats.messages_published,
            self._stats.heartbeats,
            self._stats.parse_errors,
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read MAVLink messages from Serial or UDP and publish attitude, position, "
            "and status data to Foxglove using well-known schemas (PoseInFrame, "
            "LocationFix, FrameTransform) for native 3D and Map panel visualization."
        ),
    )

    # Transport group (mutually exclusive)
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument(
        "--port",
        help="Serial device path (e.g. /dev/tty.usbmodemXXXX)",
    )
    transport.add_argument(
        "--udp",
        metavar="HOST:PORT",
        help=f"UDP listen address (default: {DEFAULT_UDP_ADDRESS})",
        nargs="?",
        const=DEFAULT_UDP_ADDRESS,
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "--foxglove-host",
        default=DEFAULT_FOXGLOVE_HOST,
        help=f"Foxglove WebSocket bind host (default: {DEFAULT_FOXGLOVE_HOST})",
    )
    parser.add_argument(
        "--foxglove-port",
        type=int,
        default=DEFAULT_FOXGLOVE_PORT,
        help=f"Foxglove WebSocket bind port (default: {DEFAULT_FOXGLOVE_PORT})",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=DEFAULT_RECONNECT_DELAY_S,
        help=f"Delay before retrying a lost link in seconds (default: {DEFAULT_RECONNECT_DELAY_S})",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="Print available serial ports and exit",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Application log level",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> BridgeConfig:
    return BridgeConfig(
        serial_port=args.port,
        baud=args.baud,
        udp_address=args.udp,
        foxglove_host=args.foxglove_host,
        foxglove_port=args.foxglove_port,
        reconnect_delay_s=args.reconnect_delay,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if args.list_ports:
        for line in available_port_lines():
            print(line)
        return 0

    logging.basicConfig(level=args.log_level, format="%(asctime)s [%(levelname)s] %(message)s")
    set_log_level(args.log_level)

    # Resolve transport
    if args.udp is not None:
        LOGGER.info("Transport: UDP on %s", args.udp)
    elif args.port is not None:
        LOGGER.info("Transport: Serial on %s", args.port)
    else:
        # Default: prompt for serial port selection
        args.port = prompt_port_selection()
        if args.port is None:
            return 1

    bridge = MavlinkFoxgloveBridge(config_from_args(args))

    try:
        return bridge.run()
    except KeyboardInterrupt:
        LOGGER.info("Stopping bridge on user interrupt")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
