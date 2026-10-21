import json
from pathlib import Path
import socket
import subprocess
import sys
import time
from unittest.mock import Mock

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, UInt8MultiArray

from airborne import bridge


@pytest.fixture
def make_node():
    context = Context()
    rclpy.init(context=context)
    nodes = []

    def create(**settings):
        node = bridge.AirborneBridge(
            context=context,
            parameter_overrides=[Parameter(name, value=value) for name, value in settings.items()],
        )
        nodes.append(node)
        return node

    yield create
    for node in reversed(nodes):
        node.destroy_node()
    context.shutdown()


def packet(dialect, kind="HEARTBEAT", signed=False):
    encoder = dialect.MAVLink(None, srcSystem=42, srcComponent=1)
    encoder.seq = 17
    if signed:
        encoder.signing.secret_key = b"a" * 32
        encoder.signing.link_id = 3
        encoder.signing.timestamp = 123456
        encoder.signing.sign_outgoing = True
    if kind == "HEARTBEAT":
        message = encoder.heartbeat_encode(2, 3, 0, 0, 4)
    else:
        message = encoder.attitude_encode(123, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0)
    frame = message.pack(encoder)
    parser = dialect.MAVLink(None)
    return parser.parse_char(frame), bytes(frame)


def connect_fakes(monkeypatch, messages):
    pixhawk, radio = Mock(), Mock()
    pixhawk.recv_match.side_effect = list(messages) + [None]
    radio.write.side_effect = len
    opener = Mock(side_effect=[pixhawk, radio])
    monkeypatch.setattr(bridge, "open_connection", opener)
    return pixhawk, radio, opener


@pytest.mark.parametrize("dialect,signed", [(mavlink1, False), (mavlink2, False), (mavlink2, True)])
def test_original_frames_and_stream_request(make_node, monkeypatch, dialect, signed):
    heartbeat, heartbeat_frame = packet(dialect, signed=signed)
    attitude, attitude_frame = packet(dialect, "ATTITUDE", signed=signed)
    bad = Mock()
    bad.get_type.return_value = "BAD_DATA"
    pixhawk, radio, _ = connect_fakes(monkeypatch, [attitude, bad, heartbeat, attitude, heartbeat])
    node = make_node()
    node._poll()
    assert [call.args[0] for call in radio.write.call_args_list] == [
        heartbeat_frame, attitude_frame, heartbeat_frame,
    ]
    pixhawk.mav.request_data_stream_send.assert_called_once_with(
        42, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
    )
    assert node.forwarded_messages == 3
    assert node.forwarded_bytes == len(heartbeat_frame) * 2 + len(attitude_frame)
    assert all(call.kwargs == {"blocking": False} for call in pixhawk.recv_match.call_args_list)


def test_skip_stream_request(make_node, monkeypatch):
    heartbeat, _ = packet(mavlink1)
    pixhawk, radio, _ = connect_fakes(monkeypatch, [heartbeat])
    node = make_node(skip_stream_request=True)
    node._poll()
    pixhawk.mav.request_data_stream_send.assert_not_called()
    assert radio.write.call_count == 1


def test_poll_budget_and_read_only_settings(make_node, monkeypatch):
    heartbeat, _ = packet(mavlink1)
    _, radio, _ = connect_fakes(monkeypatch, [heartbeat] * 5)
    node = make_node(max_messages_per_poll=2)
    node._poll()
    assert radio.write.call_count == 2
    node._poll()
    assert radio.write.call_count == 4
    result = node.set_parameters([Parameter("radio", value="/dev/another-radio")])
    assert not result[0].successful


def test_write_failure_closes_and_reconnects(make_node, monkeypatch):
    heartbeat, _ = packet(mavlink1)
    pixhawk, radio, opener = connect_fakes(monkeypatch, [heartbeat])
    radio.write.side_effect = OSError("radio unplugged")
    node = make_node()
    node._poll()
    pixhawk.close.assert_called_once()
    radio.close.assert_called_once()
    assert node.pixhawk is None
    assert node.forwarded_messages == 0
    node._poll()
    assert opener.call_count == 2  # Reconnect delay has not expired.

    recovered_input, recovered_radio = Mock(), Mock()
    recovered_input.recv_match.side_effect = [heartbeat, None]
    recovered_radio.write.side_effect = len
    opener.side_effect = [recovered_input, recovered_radio]
    node.next_connect = 0.0
    node._poll()
    assert node.ready
    assert node.forwarded_messages == 1


def test_heartbeat_timeout_and_partial_open_cleanup(make_node, monkeypatch):
    pixhawk, radio, _ = connect_fakes(monkeypatch, [])
    node = make_node()
    node._poll()
    node.last_heartbeat = time.monotonic() - 11
    pixhawk.recv_match.side_effect = None
    pixhawk.recv_match.return_value = None
    node._poll()
    assert node.pixhawk is None
    pixhawk.close.assert_called_once()
    radio.close.assert_called_once()

    partial = Mock()
    monkeypatch.setattr(bridge, "open_connection", Mock(side_effect=[partial, OSError("missing radio")]))
    node.next_connect = 0.0
    node._poll()
    partial.close.assert_called_once()
    assert node.pixhawk is None


def test_partial_write_is_failure(make_node, monkeypatch):
    heartbeat, _ = packet(mavlink1)
    _, radio, _ = connect_fakes(monkeypatch, [heartbeat])
    radio.write.side_effect = None
    radio.write.return_value = 1
    node = make_node()
    node._poll()
    assert node.forwarded_messages == 0
    assert node.pixhawk is None


@pytest.mark.parametrize("settings", [
    {"poll_interval": 0.0}, {"reconnect_delay": -1.0},
    {"heartbeat_timeout": float("nan")}, {"pixhawk": " "},
    {"max_messages_per_poll": 0}, {"stream_rate": 65536},
])
def test_invalid_parameters(make_node, settings):
    with pytest.raises(ValueError):
        make_node(**settings)


def test_json_unknowns():
    result = bridge.json_value({"unknown": float("nan"), "values": [float("inf"), b"ab"]})
    assert json.loads(json.dumps(result, allow_nan=False)) == {
        "unknown": None, "values": [None, [97, 98]],
    }


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_live_ros_simulator_to_python_ground(make_node, tmp_path):
    root = Path(__file__).resolve().parents[2]
    input_port, ground_port = free_udp_port(), free_udp_port()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.setblocking(False)
        mission_port = receiver.getsockname()[1]
        node = make_node(
            pixhawk=f"udpin:127.0.0.1:{input_port}",
            radio=f"udpout:127.0.0.1:{ground_port}",
            skip_stream_request=True,
        )
        subscriber = Node("test_telemetry_consumer", context=node.context)
        decoded, raw_frames, ground_frames = [], [], []
        subscriber.create_subscription(String, "telemetry/mavlink", lambda msg: decoded.append(json.loads(msg.data)), 100)
        subscriber.create_subscription(UInt8MultiArray, "telemetry/mavlink_raw", lambda msg: raw_frames.append(bytes(msg.data)), 100)
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        executor.add_node(subscriber)
        processes = []
        try:
            deadline = time.monotonic() + 10
            while (node.telemetry_publisher.get_subscription_count() == 0
                   or node.raw_publisher.get_subscription_count() == 0):
                assert time.monotonic() < deadline, "ROS subscribers were not discovered"
                executor.spin_once(timeout_sec=0.02)

            with (tmp_path / "ground.log").open("w") as log:
                processes.append(subprocess.Popen([
                    sys.executable, "-u", str(root / "ground/ground_bridge.py"),
                    "--radio", f"udpin:127.0.0.1:{ground_port}",
                    "--mission-planner", f"udpout:127.0.0.1:{mission_port}",
                    "--status-interval", "0.1",
                ], stdout=log, stderr=subprocess.STDOUT))
                deadline = time.monotonic() + 5
                while "Ground bridge is running." not in (tmp_path / "ground.log").read_text():
                    assert processes[0].poll() is None, (tmp_path / "ground.log").read_text()
                    assert time.monotonic() < deadline, "Ground failed to start"
                    executor.spin_once(timeout_sec=0.02)

            processes.append(subprocess.Popen([
                sys.executable, str(root / "sim/simulate_pixhawk.py"),
                "--output", f"udpout:127.0.0.1:{input_port}",
            ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
            expected = {"HEARTBEAT", "ATTITUDE", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "SYS_STATUS"}
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                while True:
                    try:
                        ground_frames.append(receiver.recv(65535))
                    except BlockingIOError:
                        break
                types = {item["message"]["mavpackettype"] for item in decoded}
                parser = mavlink1.MAVLink(None)
                ground_types = {parser.parse_char(frame).get_type() for frame in ground_frames}
                # DDS callbacks can arrive after the matching UDP packet.
                if (expected <= types and expected <= ground_types
                        and len(ground_frames) >= 20
                        and set(ground_frames) <= set(raw_frames)):
                    break
            assert expected <= types
            assert expected <= ground_types
            # Every frame arriving at Mission Planner also reached ROS byte for byte.
            assert set(ground_frames) <= set(raw_frames)
            assert all(item["system_id"] == 1 and item["component_id"] == 1 for item in decoded)
            assert all("stamp" in item and "sequence" in item for item in decoded)
            assert "--- Ground Telemetry ---" in (tmp_path / "ground.log").read_text()
        finally:
            for process in reversed(processes):
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                if process.stderr is not None:
                    process.stderr.close()
            executor.shutdown()
            subscriber.destroy_node()
