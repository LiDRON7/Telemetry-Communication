# LiDRON telemetry communication

`airborne/` is a ROS 2 Python package. `ground/` remains a standalone Python
application and does not need ROS 2.

```text
Pixhawk --MAVLink--> airborne ROS 2 node --MAVLink/radio--> ground Python --> Mission Planner
                            |
                            +--> ROS 2 topics --> onboard algorithms
```

The airborne node forwards the original MAVLink frames, including the first
heartbeat, without changing IDs, sequence numbers, checksums, or signatures.
ROS messages travel only between onboard ROS nodes. The radio and Mission Planner
continue to receive MAVLink. Forwarding is one way, from Pixhawk to ground;
algorithm outputs and commands to the aircraft are not routed by this bridge.

## Build airborne

ROS 2 Jazzy on Ubuntu 24.04 is the tested environment. Source your installed
ROS distribution, then run these commands from this repository's root:

```bash
source /opt/ros/jazzy/setup.bash
# Use system Python so the virtual environment can import the installed ROS packages.
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
# Install a colcon executable that uses this virtual environment's Python.
python -m pip install --force-reinstall --no-deps colcon-core
touch .venv/COLCON_IGNORE
# Invoke colcon with the environment's Python so installed executables use it.
python -m colcon build --symlink-install --packages-select airborne
source install/setup.bash
```

The host needs `python3-venv`, ROS development tools (`colcon`), and access to
serial devices (typically membership in `dialout`). The root `requirements.txt`
is shared by airborne, ground, and the simulator; ROS packages come from the ROS
installation rather than pip. Keep the virtual environment in place after building.

With `.venv` activated, `command -v colcon` should point to `.venv/bin/colcon`.
The system `/usr/bin/colcon` uses system Python even when a virtual environment
is active; rebuilding with it can cause `ModuleNotFoundError: pymavlink`.
Use `python -m colcon build` to explicitly select the active Python interpreter.

## Run airborne

With defaults (`/dev/ttyACM0` at 115200 baud and `/dev/ttyUSB0` at 57600 baud):

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch airborne airborne.launch.py
```

For specific devices or UDP endpoints:

```bash
ros2 run airborne airborne_bridge --ros-args \
  -p pixhawk:=/dev/ttyACM0 -p pixhawk_baud:=115200 \
  -p radio:=/dev/ttyUSB0 -p radio_baud:=57600 \
  -p stream_rate:=4
```

Use stable `/dev/serial/by-id/...` paths on the aircraft. All settings are ROS
parameters; the old `--pixhawk`/`--radio` options are replaced by `--ros-args -p`.
The compatibility script `python airborne/airborne_bridge.py` also accepts ROS
arguments after sourcing ROS and activating the environment.

| Parameter | Default | Purpose |
| --- | --- | --- |
| `pixhawk` | `/dev/ttyACM0` | Pixhawk serial device or MAVLink endpoint |
| `pixhawk_baud` | `115200` | Pixhawk serial baud rate |
| `radio` | `/dev/ttyUSB0` | Airborne radio device or MAVLink endpoint |
| `radio_baud` | `57600` | Radio serial baud rate |
| `stream_rate` | `10` | Requested MAVLink stream rate, Hz |
| `skip_stream_request` | `false` | Do not send `REQUEST_DATA_STREAM` |
| `status_interval` | `5.0` | Seconds between ROS log reports |
| `reconnect_delay` | `2.0` | Seconds before reopening failed connections |
| `heartbeat_timeout` | `10.0` | Seconds without a Pixhawk heartbeat before reconnecting |
| `poll_interval` | `0.01` | Seconds between nonblocking receive polls |
| `max_messages_per_poll` | `100` | Bound processing per executor callback |

Parameters are read only after startup; restart the node to apply changes.
For launch configuration, copy `airborne/config/airborne.yaml`, edit it, and run:

```bash
ros2 launch airborne airborne.launch.py params_file:=/absolute/path/airborne.yaml
```

The bridge waits for a heartbeat before forwarding, requests streams once per
connection, skips malformed input, and closes/reopens both connections after
I/O failures or heartbeat loss. Telemetry is live: disconnected periods are not
buffered or replayed. A serial radio's availability is determined by local write
errors; this one-way link cannot confirm remote reception.

## Use telemetry in ROS 2 algorithms

Both topics use reliable, volatile QoS with depth 10 and support ROS namespace
and topic remapping. They publish once per forwarded MAVLink frame:

| Topic | ROS type | Payload |
| --- | --- | --- |
| `/telemetry/mavlink` | `std_msgs/msg/String` | JSON containing receive time, source IDs, sequence, and decoded MAVLink fields |
| `/telemetry/mavlink_raw` | `std_msgs/msg/UInt8MultiArray` | Complete original MAVLink frame in `data` |

The JSON envelope has `stamp: {sec, nanosec}`, `system_id`, `component_id`,
`sequence`, and `message`. `message.mavpackettype` identifies the MAVLink message
(e.g. `ATTITUDE`, `GLOBAL_POSITION_INT`, `GPS_RAW_INT`, `SYS_STATUS`, or
`HEARTBEAT`). Fields keep their MAVLink units and coordinate conventions:
attitude angles are radians, integer latitude/longitude are degrees times 1e7,
and `GLOBAL_POSITION_INT` altitudes are millimeters. This interface does not
convert MAVLink NED/FRD frames to ROS ENU/FLU frames. Unknown nonfinite floats
become JSON `null`; byte arrays become integer arrays. `stamp` is ROS receipt
time, while boot/GPS timestamps remain inside the message.

Example subscription inside an algorithm's ROS 2 node:

```python
import json
from std_msgs.msg import String

# In your Node's __init__:
self.telemetry_subscription = self.create_subscription(
    String, "/telemetry/mavlink", self.on_telemetry, 10
)

# Method on your Node:
def on_telemetry(self, msg):
    telemetry = json.loads(msg.data)
    packet = telemetry["message"]
    if packet["mavpackettype"] == "ATTITUDE":
        roll, pitch, yaw = packet["roll"], packet["pitch"], packet["yaw"]
        # Feed these values to your algorithm using MAVLink's frame conventions.
```

Inspect messages with `ros2 topic echo /telemetry/mavlink`.
These generic topics expose all decoded MAVLink message types without adding
MAVROS or a custom message build dependency. Algorithms requiring typed sensor
messages should adapt the fields, units, timestamps, and reference frames they use.

## Run ground (Python only)

On the ground computer, create a regular Python virtual environment and install
`requirements.txt`. No ROS installation or ROS environment is needed:

```bash
python -m pip install -r requirements.txt
python ground/ground_bridge.py --radio COM5 --radio-baud 57600 \
  --mission-planner udpout:127.0.0.1:14550
```

Select Mission Planner's UDP input on port 14550. On Linux, replace `COM5` with
the ground radio device, such as `/dev/ttyUSB0`. Ground continues to display GPS,
attitude, battery, and flight mode while relaying the original frames.

## Simulate without hardware

Start these in separate terminals from the repository root. For the airborne
terminal, source ROS, activate `.venv`, and source `install/setup.bash` first.
The other two terminals need only the Python dependencies.

```bash
# Ground: listens for airborne and forwards to Mission Planner.
python ground/ground_bridge.py --radio udpin:127.0.0.1:14610

# Airborne ROS 2 node.
ros2 run airborne airborne_bridge --ros-args \
  -p pixhawk:=udpin:127.0.0.1:14600 \
  -p radio:=udpout:127.0.0.1:14610 -p skip_stream_request:=true

# Simulator: generates heartbeat, attitude, position, GPS, and battery telemetry.
python sim/simulate_pixhawk.py --output udpout:127.0.0.1:14600
```

On the ROS machine, `ros2 topic echo /telemetry/mavlink` shows the same telemetry
that ground receives. ROS 2 does not need to be installed on the ground machine.

## Tests

After building and sourcing ROS and the virtual environment:

```bash
python -m pytest airborne/test -q
```

Tests exercise original-frame forwarding, stream requests, reconnects, malformed
input, parameter validation, and a live UDP path through the ROS node and the
unchanged Python ground application, with a ROS subscriber and Mission Planner
UDP receiver.

## Aircraft service

`airborne/airborne-bridge.service` now launches the installed ROS 2 executable.
Build in the repository root before using it. Review its user, workspace path,
`ROS_DISTRO`, and serial device IDs for the target aircraft. Its existing device
IDs and 4 Hz stream request are retained. After installing or updating the unit:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now airborne-bridge.service
journalctl -u airborne-bridge.service -f
```

The service assumes the repository is located at
`/home/lidron-blueberry/Telemetry-Communication` and that the ROS workspace was
built there with the Python dependencies available to the installed executable.
