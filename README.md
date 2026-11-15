# LiDRON Telemetry Communication

How the ground station talks to the drone, and the MAVLink bridges we wrote before the radio moved to the Pixhawk.

## How the link works now

The SiK radio on the drone plugs into Pixhawk TELEM1. The second SiK radio sits on the ground laptop's USB, and QGroundControl talks to PX4 through it directly. Nothing in this repo is in that path, so the link keeps working even if the Pi crashes.

The ground laptop also has the Tersus PRECIS RTK base station on USB. QGroundControl sends its RTCM corrections up the same SiK link, and PX4 passes them to the H-RTK F9P on the drone for an RTK fix.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/LiDRON7/Sensor-Control-Data-Gathering/main/docs/architecture/data-flow-dark.png">
  <img alt="Who talks to whom: sensors, the Pi, PX4, QGroundControl and the RTK base" src="https://raw.githubusercontent.com/LiDRON7/Sensor-Control-Data-Gathering/main/docs/architecture/data-flow-light.png">
</picture>

The full data and comms picture, with every topic, lives in [Sensor-Control-Data-Gathering](https://github.com/LiDRON7/Sensor-Control-Data-Gathering). Wiring is in [Drone-Construction](https://github.com/LiDRON7/Drone-Construction/tree/main/docs/architecture).

## Set up the ground station

1. Plug the 500 mW SiK radio into the laptop and open QGroundControl. It finds the drone on its own at 57600 baud.
2. RTK: set up the Tersus base on its tripod with a clear sky view. Connect the antenna before you turn it on (the note in the case says so too). Plug it into the laptop, then in QGroundControl go to Application Settings, RTK, and point it at the base's serial port. We haven't tested this with the Tersus yet: QGroundControl's RTK list names u-blox, Trimble and Septentrio bases, so if it doesn't show up, the fallback is an NTRIP caster running on the laptop.
3. Wait for "RTK fixed" in the GPS status before an autonomous flight.

## What is in this repo

| Folder | What it does | Used in flight? |
| --- | --- | --- |
| `ground/ground_bridge.py` | radio on a COM port to Mission Planner over UDP, one way | no, QGroundControl replaced Mission Planner |
| `airborne/airborne_bridge.py` | Pixhawk USB to the radio on the Pi | no, an older copy of `airborne/` in Sensor-Control-Data-Gathering |
| `lidron_mavlink/` | scripts we used to read MAVLink off the Pixhawk the first time | no |
| `sim/simulate_pixhawk.py` | fake Pixhawk for testing the bridges | no |

Whether to archive this repo is issue [#32](https://github.com/LiDRON7/Telemetry-Communication/issues/32). Telemetry has no authentication yet: anyone with a SiK radio on our net ID could send commands. MAVLink 2 signing in PX4 and QGroundControl is issue [#33](https://github.com/LiDRON7/Telemetry-Communication/issues/33).

## Run the old ground bridge

Only if you need Mission Planner for something. On Windows:

```powershell
python ground\ground_bridge.py `
  --radio COM4 `
  --radio-baud 57600 `
  --mission-planner udpout:127.0.0.1:14550 `
  --status-interval 1
```
