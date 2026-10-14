import argparse
import math
import socket
import time

from pymavlink import mavutil


def open_connection(endpoint, baud, source_system=246):
    return mavutil.mavlink_connection(
        endpoint,
        baud=baud,
        source_system=source_system,
        source_component=191,
        autoreconnect=False,
    )


def open_udp_output(endpoint):
    prefix = "udpout:"
    if not endpoint.startswith(prefix):
        raise ValueError(
            "Mission Planner endpoint must use udpout:HOST:PORT"
        )

    address = endpoint[len(prefix):]
    try:
        host, port = address.rsplit(":", 1)
        destination = (host, int(port))
    except ValueError as error:
        raise ValueError(
            "Mission Planner endpoint must use udpout:HOST:PORT"
        ) from error

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return udp_socket, destination


def close_connection(connection):
    if connection is not None:
        try:
            connection.close()
        except Exception:
            pass


def update_telemetry(state, message):
    message_type = message.get_type()

    if message_type == "HEARTBEAT":
        state["mode"] = mavutil.mode_string_v10(message)
        state["system_status"] = message.system_status
    elif message_type == "ATTITUDE":
        state["roll"] = math.degrees(message.roll)
        state["pitch"] = math.degrees(message.pitch)
        state["yaw"] = math.degrees(message.yaw)
    elif message_type == "GLOBAL_POSITION_INT":
        state["latitude"] = message.lat / 1e7
        state["longitude"] = message.lon / 1e7
        state["altitude_msl"] = message.alt / 1000.0
        state["relative_altitude"] = message.relative_alt / 1000.0
    elif message_type == "GPS_RAW_INT":
        state["fix_type"] = message.fix_type
        state["satellites"] = message.satellites_visible

        if state["latitude"] is None:
            state["latitude"] = message.lat / 1e7
            state["longitude"] = message.lon / 1e7
            state["altitude_msl"] = message.alt / 1000.0
    elif message_type == "SYS_STATUS":
        if message.voltage_battery != 65535:
            state["voltage"] = message.voltage_battery / 1000.0
        if message.current_battery >= 0:
            state["current"] = message.current_battery / 100.0
        if message.battery_remaining >= 0:
            state["battery_remaining"] = message.battery_remaining


def format_value(value, format_spec, suffix=""):
    if value is None:
        return "unknown"
    return f"{value:{format_spec}}{suffix}"


def display_telemetry(state, message_count, byte_count):
    print("\n--- Ground Telemetry ---")
    print(f"Packets: {message_count} | Bytes: {byte_count}")
    print(
        "GPS: "
        f"{format_value(state['latitude'], '.7f')}, "
        f"{format_value(state['longitude'], '.7f')} | "
        f"MSL altitude: {format_value(state['altitude_msl'], '.1f', ' m')} | "
        "Relative altitude: "
        f"{format_value(state['relative_altitude'], '.1f', ' m')}"
    )
    print(
        f"GPS fix: {state['fix_type']} | "
        f"Satellites: {state['satellites']}"
    )
    print(
        "Attitude: "
        f"roll={format_value(state['roll'], '.1f', ' deg')}, "
        f"pitch={format_value(state['pitch'], '.1f', ' deg')}, "
        f"yaw={format_value(state['yaw'], '.1f', ' deg')}"
    )
    print(
        "Battery: "
        f"{format_value(state['voltage'], '.2f', ' V')}, "
        f"{format_value(state['current'], '.2f', ' A')}, "
        f"remaining={format_value(state['battery_remaining'], 'd', '%')}"
    )
    print(
        f"Mode: {state['mode']} | "
        f"System status: {state['system_status']}"
    )


def run_bridge(args):
    while True:
        radio = None
        mission_planner = None
        mission_planner_address = None

        try:
            print(f"Opening ground radio input: {args.radio}")
            radio = open_connection(args.radio, args.radio_baud)

            print(f"Opening Mission Planner output: {args.mission_planner}")
            mission_planner, mission_planner_address = open_udp_output(
                args.mission_planner
            )

            state = {
                "latitude": None,
                "longitude": None,
                "altitude_msl": None,
                "relative_altitude": None,
                "fix_type": None,
                "satellites": None,
                "roll": None,
                "pitch": None,
                "yaw": None,
                "voltage": None,
                "current": None,
                "battery_remaining": None,
                "mode": "unknown",
                "system_status": None,
            }

            message_count = 0
            byte_count = 0
            last_display = time.monotonic()

            print("Ground bridge is running.")

            while True:
                message = radio.recv_match(blocking=True, timeout=1)

                if message is None or message.get_type() == "BAD_DATA":
                    continue

                frame = message.get_msgbuf()
                if not frame:
                    continue

                # Mission Planner receives the original Pixhawk MAVLink frame.
                mission_planner.sendto(frame, mission_planner_address)
                message_count += 1
                byte_count += len(frame)
                update_telemetry(state, message)

                now = time.monotonic()
                if now - last_display >= args.status_interval:
                    display_telemetry(state, message_count, byte_count)
                    last_display = now

        except KeyboardInterrupt:
            print("\nGround bridge stopped.")
            return
        except Exception as error:
            print(f"Ground bridge error: {error}")
            print(f"Reconnecting in {args.reconnect_delay} seconds...")
            time.sleep(args.reconnect_delay)
        finally:
            close_connection(radio)
            close_connection(mission_planner)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Forward MAVLink telemetry from a ground radio to "
            "Mission Planner while displaying selected values."
        )
    )
    parser.add_argument(
        "--radio",
        default="COM5",
        help=(
            "Ground radio serial device or MAVLink endpoint, such as "
            "COM5 or udpin:127.0.0.1:14610"
        ),
    )
    parser.add_argument(
        "--radio-baud",
        type=int,
        default=57600,
        help="Ground radio serial baud rate",
    )
    parser.add_argument(
        "--mission-planner",
        default="udpout:127.0.0.1:14550",
        help="Mission Planner MAVLink output endpoint",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=1.0,
        help="Seconds between console telemetry updates",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=2.0,
        help="Seconds to wait after a connection failure",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_bridge(parse_arguments())
