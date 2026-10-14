import argparse
import time

from pymavlink import mavutil


def open_connection(endpoint, baud, source_system=245):
    return mavutil.mavlink_connection(
        endpoint,
        baud=baud,
        source_system=source_system,
        source_component=191,
        autoreconnect=False,
    )


def close_connection(connection):
    if connection is not None:
        try:
            connection.close()
        except Exception:
            pass


def run_bridge(args):
    while True:
        pixhawk = None
        radio = None

        try:
            print(f"Opening Pixhawk input: {args.pixhawk}")
            pixhawk = open_connection(args.pixhawk, args.pixhawk_baud)

            print(f"Opening radio output: {args.radio}")
            radio = open_connection(args.radio, args.radio_baud)

            print("Waiting for Pixhawk heartbeat...")

            heartbeat = None
            while heartbeat is None:
                heartbeat = pixhawk.recv_match(
                    type="HEARTBEAT",
                    blocking=True,
                    timeout=1,
                )

            target_system = heartbeat.get_srcSystem()
            target_component = heartbeat.get_srcComponent()

            print(
                f"Heartbeat received from system {target_system}, "
                f"component {target_component}"
            )

            # Forward the heartbeat consumed while establishing the connection.
            heartbeat_frame = heartbeat.get_msgbuf()
            if heartbeat_frame:
                radio.write(heartbeat_frame)

            if not args.skip_stream_request:
                print(f"Requesting MAVLink streams at {args.stream_rate} Hz")
                pixhawk.mav.request_data_stream_send(
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    args.stream_rate,
                    1,
                )

            forwarded_messages = 1
            forwarded_bytes = len(heartbeat_frame or b"")
            last_report = time.monotonic()

            print("Airborne bridge is running.")

            while True:
                message = pixhawk.recv_match(blocking=True, timeout=1)

                if message is None or message.get_type() == "BAD_DATA":
                    continue

                frame = message.get_msgbuf()
                if not frame:
                    continue

                radio.write(frame)
                forwarded_messages += 1
                forwarded_bytes += len(frame)

                now = time.monotonic()
                if now - last_report >= args.status_interval:
                    print(
                        f"Forwarded {forwarded_messages} messages "
                        f"({forwarded_bytes} bytes), "
                        f"last={message.get_type()}"
                    )
                    last_report = now

        except KeyboardInterrupt:
            print("\nAirborne bridge stopped.")
            return
        except Exception as error:
            print(f"Airborne bridge error: {error}")
            print(f"Reconnecting in {args.reconnect_delay} seconds...")
            time.sleep(args.reconnect_delay)
        finally:
            close_connection(pixhawk)
            close_connection(radio)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Forward MAVLink telemetry from a Pixhawk connection "
            "to an airborne telemetry radio."
        )
    )
    parser.add_argument(
        "--pixhawk",
        default="/dev/ttyACM0",
        help=(
            "Pixhawk serial device or MAVLink endpoint, such as "
            "/dev/ttyACM0 or udpin:127.0.0.1:14600"
        ),
    )
    parser.add_argument(
        "--pixhawk-baud",
        type=int,
        default=115200,
        help="Pixhawk serial baud rate",
    )
    parser.add_argument(
        "--radio",
        default="/dev/ttyUSB0",
        help=(
            "Radio serial device or MAVLink endpoint, such as "
            "/dev/ttyUSB0 or udpout:127.0.0.1:14610"
        ),
    )
    parser.add_argument(
        "--radio-baud",
        type=int,
        default=57600,
        help="Airborne radio serial baud rate",
    )
    parser.add_argument(
        "--stream-rate",
        type=int,
        default=10,
        help="Requested Pixhawk telemetry stream rate in Hz",
    )
    parser.add_argument(
        "--skip-stream-request",
        action="store_true",
        help="Do not send REQUEST_DATA_STREAM to the Pixhawk",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=5.0,
        help="Seconds between forwarding status messages",
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
