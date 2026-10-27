"""Forward original MAVLink frames and publish primary GPS data to ROS 2."""

import json
import math
import time

from pymavlink import mavutil
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64, UInt8

from airborne_telemetry.gps import navsat_fix


class AirborneBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("airborne_bridge", **kwargs)
        defaults = {
            "pixhawk": "/dev/ttyACM0", "pixhawk_baud": 115200,
            "radio": "/dev/ttyUSB0", "radio_baud": 57600,
            "source_system": 245, "target_system": 1, "target_component": 1,
            "stream_rate": 10, "skip_stream_request": False,
            "reconnect_delay": 2.0, "link_timeout": 5.0, "gps_timeout": 3.0,
            "status_interval": 5.0, "frame_id": "gps_link",
            "debug_gps": False,
        }
        self.config = {
            name: self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)
            ).value for name, value in defaults.items()
        }
        for name in ("pixhawk_baud", "radio_baud", "stream_rate", "reconnect_delay",
                     "link_timeout", "gps_timeout", "status_interval"):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("source_system", "target_system", "target_component"):
            if not 1 <= self.config[name] <= 255:
                raise ValueError(f"{name} must be between 1 and 255")
        if self.config["stream_rate"] > 65535:
            raise ValueError("stream_rate must fit a MAVLink uint16")
        for name in ("pixhawk", "radio", "frame_id"):
            if not self.config[name]:
                raise ValueError(f"{name} must not be empty")
        # Reliable, volatile queues allow reliable and best-effort subscribers
        # without replaying an old RTK fix to a newly connected subscriber.
        self.fix_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.type_pub = self.create_publisher(UInt8, "gps/fix_type", 10)
        self.rtk_pub = self.create_publisher(Bool, "gps/rtk_fixed", 10)
        self.sat_pub = self.create_publisher(UInt8, "gps/satellites", 10)
        self.msl_pub = self.create_publisher(Float64, "gps/altitude_msl", 10)
        self.fresh_pub = self.create_publisher(Bool, "gps/fresh", 10)
        self.connections = {"pixhawk": None, "radio": None}
        self.retry_at = {"pixhawk": 0.0, "radio": 0.0}
        self.last_rx = time.monotonic()
        self.last_gps = None
        self.last_report = self.last_rx
        self.last_stale = 0.0
        self.requested = False
        self.forwarded = 0
        self.dropped = 0
        self.gps_debug_state = {}
        # Transport/watchdogs continue even when simulation time pauses.
        self.timer = self.create_timer(
            0.01, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def close_link(self, name):
        connection = self.connections[name]
        self.connections[name] = None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

    def failed_link(self, name, error):
        self.get_logger().warning(f"{name}: {error}; retrying in {self.config['reconnect_delay']} s")
        self.close_link(name)
        self.retry_at[name] = time.monotonic() + self.config["reconnect_delay"]
        if name == "pixhawk":
            self.last_gps = None

    def open_links(self, now):
        for name in self.connections:
            if self.connections[name] is not None or now < self.retry_at[name]:
                continue
            try:
                self.connections[name] = mavutil.mavlink_connection(
                    self.config[name], baud=self.config[f"{name}_baud"],
                    source_system=self.config["source_system"], source_component=191,
                    autoreconnect=False,
                )
                if name == "pixhawk":
                    self.last_rx = now
                    self.requested = False
                self.get_logger().info(f"Opened {name}: {self.config[name]}")
            except Exception as error:
                self.failed_link(name, error)

    def publish_gps(self, message, now):
        self.last_gps = now
        fix = navsat_fix(message, self.get_clock().now().to_msg(), self.config["frame_id"])
        self.fix_pub.publish(fix)
        self.type_pub.publish(UInt8(data=int(message.fix_type)))
        self.rtk_pub.publish(Bool(data=message.fix_type == 6))
        self.sat_pub.publish(UInt8(data=int(message.satellites_visible)))
        altitude = message.alt / 1000.0 if message.fix_type in (3, 4, 5, 6, 7, 8) else math.nan
        self.msl_pub.publish(Float64(data=altitude))
        self.fresh_pub.publish(Bool(data=True))

    def publish_stale(self, now):
        if self.last_gps is not None and now - self.last_gps <= self.config["gps_timeout"]:
            return
        if now - self.last_stale < 1.0:
            return
        self.last_stale = now
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.config["frame_id"]
        fix.status.status = NavSatStatus.STATUS_NO_FIX
        fix.latitude = fix.longitude = fix.altitude = math.nan
        self.fix_pub.publish(fix)
        self.type_pub.publish(UInt8(data=0))
        self.rtk_pub.publish(Bool(data=False))
        self.sat_pub.publish(UInt8(data=255))
        self.msl_pub.publish(Float64(data=math.nan))
        self.fresh_pub.publish(Bool(data=False))

    def log_raw_gps(self, message, now):
        """Log decoded MAVLink fields before filtering or ROS conversion."""
        message_type = message.get_type()
        if not self.config["debug_gps"] or message_type not in (
            "GPS_RAW_INT", "GPS2_RAW", "GLOBAL_POSITION_INT"
        ):
            return
        system = message.get_srcSystem()
        component = message.get_srcComponent()
        key = (system, component, message_type)
        fields = message.to_dict()
        values = {name: value for name, value in fields.items()
                  if name not in ("time_usec", "time_boot_ms")}
        previous = self.gps_debug_state.get(key)
        count = previous[0] + 1 if previous else 1
        source_match = (system == self.config["target_system"] and
                        component == self.config["target_component"])
        record = {
            "message_type": message_type,
            "source_system": system,
            "source_component": component,
            "sequence": message.get_seq(),
            "rx_count": count,
            "rx_monotonic_s": round(now, 6),
            "rx_gap_ms": round((now - previous[1]) * 1000, 3) if previous else None,
            "values_changed": values != previous[2] if previous else None,
            "source_match": source_match,
            "gps_topic_input": source_match and message_type == "GPS_RAW_INT",
            "fields": fields,
        }
        self.gps_debug_state[key] = (count, now, values)
        self.get_logger().info("RAW_GPS " + json.dumps(record, separators=(",", ":")))

    def handle_message(self, message, now):
        self.log_raw_gps(message, now)
        # Never repack: preserve IDs, sequence, version and signatures.
        frame = message.get_msgbuf()
        if frame:
            radio = self.connections["radio"]
            if radio is None:
                self.dropped += 1
            else:
                try:
                    written = radio.write(frame)
                    if written is not None and written != len(frame):
                        raise IOError("incomplete radio write")
                    self.forwarded += 1
                except Exception as error:
                    self.dropped += 1
                    self.failed_link("radio", error)
        if (message.get_srcSystem() != self.config["target_system"] or
                message.get_srcComponent() != self.config["target_component"]):
            return
        if message.get_type() == "GPS_RAW_INT":
            self.publish_gps(message, now)
        elif message.get_type() == "HEARTBEAT" and not self.requested:
            if not self.config["skip_stream_request"]:
                self.connections["pixhawk"].mav.request_data_stream_send(
                    message.get_srcSystem(), message.get_srcComponent(),
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, self.config["stream_rate"], 1,
                )
            self.requested = True

    def poll(self):
        now = time.monotonic()
        self.open_links(now)
        pixhawk = self.connections["pixhawk"]
        if pixhawk is not None:
            try:
                # Bounded nonblocking reads keep the executor responsive.
                for _ in range(100):
                    message = pixhawk.recv_match(blocking=False)
                    if message is None:
                        break
                    if message.get_type() == "BAD_DATA":
                        continue
                    self.last_rx = time.monotonic()
                    self.handle_message(message, self.last_rx)
                if time.monotonic() - self.last_rx > self.config["link_timeout"]:
                    raise TimeoutError("no valid MAVLink telemetry")
            except Exception as error:
                self.failed_link("pixhawk", error)
        now = time.monotonic()
        self.publish_stale(now)
        if now - self.last_report >= self.config["status_interval"]:
            self.get_logger().info(f"Forwarded {self.forwarded} frames; radio unavailable/dropped: {self.dropped}")
            self.last_report = now

    def destroy_node(self):
        self.timer.cancel()
        for name in self.connections:
            self.close_link(name)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AirborneBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
