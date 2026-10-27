# Drone telemetry

Airborne runs on ROS 2 Jazzy and reads telemetry from the Pixhawk. Ground stays
in Python and forwards MAVLink to Mission Planner. Run commands from the
repository root.

## 1. Build airborne on the drone computer

Requires ROS 2 Jazzy, colcon, and the Python dependencies (`pymavlink` and
`pyserial`) installed for the Python interpreter used by ROS.

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

## 2. Configure airborne

Edit `airborne/config/airborne.yaml`:

| Parameter | Default | Set to |
| --- | --- | --- |
| `pixhawk` | `/dev/ttyACM0` | Pixhawk serial port |
| `pixhawk_baud` | `115200` | Pixhawk baud rate |
| `radio` | `/dev/ttyUSB0` | Airborne radio serial port |
| `radio_baud` | `57600` | Radio baud rate |
| `target_system` | `1` | Pixhawk MAVLink system ID |
| `target_component` | `1` | Autopilot component ID |

Ensure your user can access the serial ports. If another process already uses
the Pixhawk port, configure a separate MAVLink UDP output for this bridge.

After editing the YAML, run `colcon build` again to update the installed
configuration before starting airborne.

## 3. Start ground and Mission Planner

On the ground computer, install dependencies and start ground in PowerShell.
Replace `COM4` with your ground radio port.

```powershell
python -m pip install -r requirements.txt
python ground\ground_bridge.py `
  --radio COM4 `
  --radio-baud 57600 `
  --mission-planner udpout:127.0.0.1:14550 `
  --status-interval 1
```

In Mission Planner, select UDP and listen on port **14550**. If it runs on a
different computer, replace `127.0.0.1` with that computer's IP.

## 4. Start airborne

On the drone computer:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch airborne_telemetry airborne.launch.py
```

Restart airborne after changing its configuration. This bridge sends telemetry
to ground; it does not relay Mission Planner commands or provide RTK corrections.

## 5. Read ROS topics

In another terminal on the drone computer:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list
ros2 topic echo /gps/fix
```

Press Ctrl+C to stop an echo, then inspect another topic:

```bash
ros2 topic echo /gps/fix_type
ros2 topic echo /gps/rtk_fixed
```

| Topic | Message type | Data |
| --- | --- | --- |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | Latitude/longitude in degrees; ellipsoid altitude in metres, or NaN if unavailable |
| `/gps/fix_type` | `std_msgs/msg/UInt8` | GPS fix type; **5 = RTK float**, **6 = RTK fixed** |
| `/gps/rtk_fixed` | `std_msgs/msg/Bool` | True only for a fresh RTK fixed solution |
| `/gps/fresh` | `std_msgs/msg/Bool` | GPS data received within the timeout; does not guarantee a valid fix |
| `/gps/satellites` | `std_msgs/msg/UInt8` | Visible satellites; 255 means unknown |
| `/gps/altitude_msl` | `std_msgs/msg/Float64` | Altitude above mean sea level in metres |

Fix types: **0** no GPS, **1** no fix, **2** 2D, **3** 3D, **4** DGPS/SBAS,
**5** RTK float, **6** RTK fixed, **7** static, **8** PPP.
Use `fix_type` to distinguish float from fixed; `NavSatFix.status` cannot.

Topics describe the primary GPS (`GPS_RAW_INT`) from the configured autopilot.
`/gps/fix` uses ROS receipt timestamps and unknown covariance. Publishers use
reliable, volatile QoS with depth 10; topic names follow the node namespace.

After `gps_timeout` (default **3 seconds**) without GPS data, `fresh` and
`rtk_fixed` become false, `fix_type` becomes 0, and coordinates/altitudes become
NaN. Autonomy nodes should also monitor message age to detect a stopped bridge.

Current hardware test status: build and startup succeeded with `colcon build`;
pitch, roll, yaw, and relative altitude were reported working. GPS was reported
as 0 indoors; the cause is not yet confirmed. Outdoor GPS/RTK testing is pending.
Check `/gps/fresh`, `/gps/fix_type`, and `/gps/satellites` during that test:
`fresh=false` means GPS packets are missing or stale, rather than confirming
poor satellite reception.

## Debug GPS before ROS conversion

Stop the running airborne node first, then rebuild and run with raw GPS logging:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 run airborne_telemetry airborne_bridge --ros-args \
  --params-file airborne/config/airborne.yaml -p debug_gps:=true
```

The airborne terminal prints one `RAW_GPS` JSON record per received
`GPS_RAW_INT`, `GPS2_RAW`, or `GLOBAL_POSITION_INT`, before source filtering
and ROS conversion. These are the decoded MAVLink fields received from the
Pixhawk, without replacing invalid coordinates with NaN.

- `fields`: original coordinates, fix type, satellites and message timestamp;
  `lat`/`lon` are degrees multiplied by 10,000,000, and `alt` is MSL millimetres.
- `rx_count` / `rx_gap_ms`: packet count and arrival interval for each
  source/message type. Increasing counts confirm packets arrive, even if
  their values stay zero.
- `values_changed`: whether fields changed since the previous packet, ignoring
  `time_usec`/`time_boot_ms`; `null` means this is the first packet.
- `source_match`: whether system/component IDs match the configured autopilot.
- `gps_topic_input`: whether this message feeds the GPS ROS topics. Only a
  matching `GPS_RAW_INT` does; secondary GPS and fused position are logged
  for comparison.

Compare `fields.fix_type`, `fields.lat`, and `fields.lon` against the ROS topics
in another terminal. Timestamp changes alone do not prove a new valid GPS fix.
No `RAW_GPS` lines means none of these three message types has arrived.
Logging is off by default; restart without the override to disable it.

## Optional: run without hardware

Complete step 1 first. Open three terminals on the same Linux computer.
In each, run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Then start these commands in order, one per terminal:

**Terminal 1 — ground:**

```bash
python3 ground/ground_bridge.py --radio udpin:127.0.0.1:14610 \
  --mission-planner udpout:127.0.0.1:14550
```

**Terminal 2 — airborne:**

```bash
ros2 run airborne_telemetry airborne_bridge --ros-args \
  -p pixhawk:=udpin:127.0.0.1:14600 \
  -p radio:=udpout:127.0.0.1:14610 \
  -p skip_stream_request:=true
```

**Terminal 3 — simulated Pixhawk:**

```bash
python3 sim/simulate_pixhawk.py --gps-fix-type 6
```

Use `5` for simulated RTK float or `3` for a regular 3D fix. Inspect topics
using step 5; stop the simulator to check GPS timeout behavior.
