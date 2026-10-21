"""Publish Pixhawk telemetry in ROS 2 and forward unchanged MAVLink to ground."""

import json
import math
import time

from pymavlink import mavutil
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


def json_value(value):
    """Keep MAVLink byte arrays and unknown (NaN) values valid in JSON."""
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, bytes, bytearray)):
        return [json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def open_connection(endpoint, baud):
    return mavutil.mavlink_connection(
        endpoint, baud=baud, source_system=245, source_component=191,
        autoreconnect=False,
    )


class AirborneBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("airborne_bridge", **kwargs)
        defaults = {
            "pixhawk": "/dev/ttyACM0", "pixhawk_baud": 115200,
            "radio": "/dev/ttyUSB0", "radio_baud": 57600,
            "stream_rate": 10, "skip_stream_request": False,
            "status_interval": 5.0, "reconnect_delay": 2.0,
            "heartbeat_timeout": 10.0, "poll_interval": 0.01,
            "max_messages_per_poll": 100,
        }
        # Restart to change connection settings; reject misleading live updates.
        for name, default in defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        try:
            self._validate_settings()
        except ValueError:
            super().destroy_node()
            raise

        self.telemetry_publisher = self.create_publisher(String, "telemetry/mavlink", 10)
        self.raw_publisher = self.create_publisher(UInt8MultiArray, "telemetry/mavlink_raw", 10)
        self.pixhawk = None
        self.radio = None
        self.ready = False
        self.next_connect = 0.0
        self.last_heartbeat = 0.0
        self.last_report = time.monotonic()
        self.forwarded_messages = 0
        self.forwarded_bytes = 0
        # Keep processing serial data even when simulated ROS time is paused.
        self.timer = self.create_timer(
            self.settings["poll_interval"], self._poll,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _validate_settings(self):
        for name in ("pixhawk", "radio"):
            if not self.settings[name].strip():
                raise ValueError(f"{name} must not be empty")
        for name in (
            "pixhawk_baud", "radio_baud", "stream_rate", "status_interval",
            "reconnect_delay", "heartbeat_timeout", "poll_interval",
            "max_messages_per_poll",
        ):
            value = self.settings[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if self.settings["stream_rate"] > 65535:
            raise ValueError("stream_rate must fit the MAVLink uint16 field")

    def _connect(self):
        self.get_logger().info(
            f"Opening Pixhawk {self.settings['pixhawk']} and radio "
            f"{self.settings['radio']}; waiting for heartbeat"
        )
        self.pixhawk = open_connection(self.settings["pixhawk"], self.settings["pixhawk_baud"])
        self.radio = open_connection(self.settings["radio"], self.settings["radio_baud"])
        # Bound serial writes as well as reads to keep shutdown responsive.
        for connection in (self.pixhawk, self.radio):
            port = getattr(connection, "port", None)
            if hasattr(port, "write_timeout"):
                port.write_timeout = 0.1
        self.last_heartbeat = time.monotonic()
        self.ready = False

    def _poll(self):
        if time.monotonic() < self.next_connect:
            return
        try:
            if self.pixhawk is None:
                self._connect()
            # A busy link cannot monopolize the executor. Reads never wait for data.
            for _ in range(self.settings["max_messages_per_poll"]):
                message = self.pixhawk.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_type() == "BAD_DATA":
                    continue
                if message.get_type() == "HEARTBEAT":
                    self.last_heartbeat = time.monotonic()
                    if not self.ready:
                        if not self.settings["skip_stream_request"]:
                            self.pixhawk.mav.request_data_stream_send(
                                message.get_srcSystem(), message.get_srcComponent(),
                                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                self.settings["stream_rate"], 1,
                            )
                        self.ready = True
                        self.get_logger().info("Pixhawk heartbeat received; bridge running")
                if self.ready:
                    self._forward(message)

            now = time.monotonic()
            if now - self.last_heartbeat >= self.settings["heartbeat_timeout"]:
                raise TimeoutError("Pixhawk heartbeat timed out")
            if now - self.last_report >= self.settings["status_interval"]:
                self.get_logger().info(
                    f"Forwarded {self.forwarded_messages} messages "
                    f"({self.forwarded_bytes} bytes)"
                )
                self.last_report = now
        except Exception as error:
            self.get_logger().error(
                f"Bridge error: {error}; reconnecting in "
                f"{self.settings['reconnect_delay']} seconds"
            )
            self._close_connections()
            self.next_connect = time.monotonic() + self.settings["reconnect_delay"]

    def _forward(self, message):
        frame = bytes(message.get_msgbuf())
        if not frame:
            return
        # Ground receives MAVLink bytes, without ROS serialization or repacking.
        written = self.radio.write(frame)
        if written is not None and written != len(frame):
            raise OSError(f"Incomplete radio write: {written}/{len(frame)} bytes")
        self.forwarded_messages += 1
        self.forwarded_bytes += len(frame)

        stamp = self.get_clock().now().to_msg()
        payload = {
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
            "system_id": message.get_srcSystem(),
            "component_id": message.get_srcComponent(),
            "sequence": message.get_seq(),
            "message": json_value(message.to_dict()),
        }
        self.telemetry_publisher.publish(
            String(data=json.dumps(payload, allow_nan=False, separators=(",", ":")))
        )
        self.raw_publisher.publish(UInt8MultiArray(data=list(frame)))

    def _close_connections(self):
        for connection in (self.pixhawk, self.radio):
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
        self.pixhawk = None
        self.radio = None
        self.ready = False

    def destroy_node(self):
        self.timer.cancel()
        self._close_connections()
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
