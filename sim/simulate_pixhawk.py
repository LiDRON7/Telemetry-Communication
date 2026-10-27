import argparse
import math
import time

from pymavlink import mavutil


def simulated_position(elapsed):
    center_latitude = 38.8895
    center_longitude = -77.0353

    latitude = center_latitude + 0.00020 * math.sin(elapsed / 12.0)
    longitude = center_longitude + 0.00020 * math.cos(elapsed / 12.0)
    altitude_msl = 100.0 + 5.0 * math.sin(elapsed / 8.0)
    relative_altitude = 20.0 + 3.0 * math.sin(elapsed / 8.0)

    return latitude, longitude, altitude_msl, relative_altitude


def simulated_attitude(elapsed):
    roll = math.radians(12.0 * math.sin(elapsed / 2.0))
    pitch = math.radians(7.0 * math.sin(elapsed / 3.0))

    heading_degrees = (90.0 + elapsed * 4.0) % 360.0
    yaw = math.radians(heading_degrees)
    if yaw > math.pi:
        yaw -= 2.0 * math.pi

    roll_speed = math.radians(6.0 * math.cos(elapsed / 2.0))
    pitch_speed = math.radians((7.0 / 3.0) * math.cos(elapsed / 3.0))
    yaw_speed = math.radians(4.0)

    return (
        roll,
        pitch,
        yaw,
        roll_speed,
        pitch_speed,
        yaw_speed,
        heading_degrees,
    )


def run_simulator(args):
    print(f"Opening simulated Pixhawk output: {args.output}")
    connection = mavutil.mavlink_connection(
        args.output,
        source_system=args.system_id,
        source_component=1,
    )

    start_time = time.monotonic()
    next_heartbeat = start_time
    next_attitude = start_time
    next_position = start_time
    next_gps = start_time
    next_system_status = start_time
    next_report = start_time + 5.0
    sent_messages = 0

    print("Simulated Pixhawk is running.")

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start_time
            boot_time_ms = int(elapsed * 1000) & 0xFFFFFFFF

            latitude, longitude, altitude_msl, relative_altitude = (
                simulated_position(elapsed)
            )
            (
                roll,
                pitch,
                yaw,
                roll_speed,
                pitch_speed,
                yaw_speed,
                heading_degrees,
            ) = simulated_attitude(elapsed)

            latitude_int = int(latitude * 1e7)
            longitude_int = int(longitude * 1e7)
            altitude_mm = int(altitude_msl * 1000)
            relative_altitude_mm = int(relative_altitude * 1000)
            heading_centidegrees = int(heading_degrees * 100) % 36000

            if now >= next_heartbeat:
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    mavutil.mavlink.MAV_AUTOPILOT_PX4,
                    (
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                        | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    ),
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                    3,
                )
                sent_messages += 1
                next_heartbeat += 1.0

            if now >= next_attitude:
                connection.mav.attitude_send(
                    boot_time_ms,
                    roll,
                    pitch,
                    yaw,
                    roll_speed,
                    pitch_speed,
                    yaw_speed,
                )
                sent_messages += 1
                next_attitude += 0.1

            if now >= next_position:
                north_velocity = int(200.0 * math.cos(elapsed / 12.0))
                east_velocity = int(200.0 * math.sin(elapsed / 12.0))
                down_velocity = int(-40.0 * math.cos(elapsed / 8.0))

                connection.mav.global_position_int_send(
                    boot_time_ms,
                    latitude_int,
                    longitude_int,
                    altitude_mm,
                    relative_altitude_mm,
                    north_velocity,
                    east_velocity,
                    down_velocity,
                    heading_centidegrees,
                )
                sent_messages += 1
                next_position += 0.2

            if now >= next_gps:
                connection.mav.gps_raw_int_send(
                    int(time.time() * 1_000_000),
                    args.gps_fix_type,
                    latitude_int,
                    longitude_int,
                    altitude_mm,
                    80,
                    120,
                    200,
                    heading_centidegrees,
                    14,
                )
                sent_messages += 1
                next_gps += 1.0

            if now >= next_system_status:
                battery_remaining = max(20, int(95 - elapsed / 60.0))
                voltage_mv = int(16_400 - (95 - battery_remaining) * 45)
                current_centiamp = int(
                    520 + 80 * math.sin(elapsed / 4.0)
                )

                connection.mav.sys_status_send(
                    0,
                    0,
                    0,
                    250,
                    voltage_mv,
                    current_centiamp,
                    battery_remaining,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                sent_messages += 1
                next_system_status += 1.0

            if now >= next_report:
                print(
                    f"Simulator sent {sent_messages} MAVLink messages | "
                    f"position={latitude:.7f},{longitude:.7f} | "
                    f"altitude={altitude_msl:.1f} m"
                )
                next_report += 5.0

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nSimulated Pixhawk stopped.")
    finally:
        connection.close()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Generate simulated Pixhawk MAVLink telemetry."
    )
    parser.add_argument(
        "--output",
        default="udpout:127.0.0.1:14600",
        help="MAVLink output endpoint",
    )
    parser.add_argument(
        "--gps-fix-type",
        type=int,
        choices=range(9),
        default=3,
        help="MAVLink GPS fix: 3=3D, 5=RTK float, 6=RTK fixed (simulated only)",
    )
    parser.add_argument(
        "--system-id",
        type=int,
        default=1,
        help="Simulated Pixhawk MAVLink system ID",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_simulator(parse_arguments())
