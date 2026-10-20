<div align="center">

# 🌾 Arkairo

### Autonomous dual-drone crop-disease survey & precision spraying on ROS 2

Drone-1 surveys the field and finds diseased plants. Drone-2 flies to each one and sprays it.
No ground operator in the loop. Geotags travel over the existing MAVLink telemetry radios.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)
![ArduPilot](https://img.shields.io/badge/ArduPilot-Copter-orange)
![MAVLink](https://img.shields.io/badge/MAVLink-MAVROS-blue)
![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-HSV%20detection-5C3EE8?logo=opencv&logoColor=white)
![Hardware](https://img.shields.io/badge/Cube_Orange%2B-Raspberry_Pi_5-informational)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

`ros2` `drone` `uav` `ardupilot` `mavros` `mavlink` `precision-agriculture` `crop-disease-detection` `computer-vision` `visual-servoing` `multi-drone` `autonomous-flight`

</div>

---

## Contents

[How it works](#how-it-works) · [Repository layout](#repository-layout) · [Nodes](#nodes) · [Quick start](#quick-start) · [Testing](#testing) · [Topics](#topic-reference) · [Design rules](#design-rules) · [Docs](#documentation) · [License](#license)

---

## How it works

```
 Drone-1 (Survey)            GCS laptop                Drone-2 (Sprayer)
┌─────────────────┐       ┌─────────────────┐       ┌─────────────────────┐
│ KML → lanes     │       │  GCS Forwarder  │       │ Idle until a geotag │
│ Auto arm/takeoff│ SiK A │  (pymavlink)    │ SiK B │ Arm → Navigate →    │
│ Survey + detect │══════►│  filter SYSID=1 │══════►│ Center → Spray →    │
│ Geotag → RTL    │       │  dedup + relay  │       │ Next target / RTL   │
└─────────────────┘       └─────────────────┘       └─────────────────────┘
   Cube Orange+              /dev/ttyUSB0,1             Cube Orange+
   TF-Luna avoidance is handled by ArduPilot, never by ROS.
```

### Geotag data flow

| # | Stage | What happens |
| - | ----- | ------------ |
| 1 | **Detect** (D1, Pi 5) | HSV threshold for yellow patches, green-context and sand filtering, contour checks, drone GPS as position, 3 m haversine dedup → `/drone1/disease_geotag` |
| 2 | **Encode** (D1 `telem_tx`) | `GEOTAG:10.048352,76.330968,0.0` as MAVLink `STATUSTEXT` (50-char limit) via `/mavros/statustext/send` |
| 3 | **Radio A** | Cube → SiK 915 MHz → ground radio on `/dev/ttyUSB0` |
| 4 | **Forward** (GCS) | pymavlink keeps `STATUSTEXT` from SYSID 1 starting with `GEOTAG:`, drops exact duplicates, resends on `/dev/ttyUSB1` |
| 5 | **Radio B** | Ground radio → Drone-2 Cube |
| 6 | **Decode** (D2 `telem_rx`) | Parses the string, validates coordinates → `/drone2/target_position` |
| 7 | **Spray** (D2) | Navigate → visual-servo centering → spray relay → wait 10 s for the next geotag → RTL on timeout |

A `NAMED_VALUE_FLOAT` fallback (`d_lat`, `d_lon`, `d_alt`) is supported; `STATUSTEXT` is preferred because it arrives as a single message.

---

## Repository layout

| Path | Contents |
| ---- | -------- |
| [`drone1_ws/`](drone1_ws/) | Survey drone, 7 packages: KML planner, navigation, image capture, detection and geotag, telemetry TX, messages, bringup |
| [`drone2_ws/`](drone2_ws/) | Sprayer drone, 6 packages: telemetry RX, unified navigation, merged detection and centering, sprayer control, messages, bringup |
| [`gcs_ws/`](gcs_ws/) | Ground station `gcs_forwarder` relay |
| [`terrain/`](terrain/) | SRTM terrain tiles for SITL |
| `launch_drone{1,2}_test.sh` | SITL test launchers |
| `autostart_drone1.sh` | Boot-time launcher for Drone-1 |
| `send_test_geotag.sh` | Inject a fake geotag for Drone-2 testing |
| [`DEPLOYMENT.md`](DEPLOYMENT.md) · [`SITL_TESTING_GUIDE.md`](SITL_TESTING_GUIDE.md) · [`HARDWARE_TESTING.md`](HARDWARE_TESTING.md) · [`TELEMETRY_HARDWARE_TEST.md`](TELEMETRY_HARDWARE_TEST.md) | Simulation, hardware and telemetry guides |

Drone-2 merged mission management into `drone2_navigation` and merged detection with centering, which cut it from 9 to 6 packages and roughly halved inter-node latency.

---

## Nodes

### Drone-1: survey

| Node | Role | Dev guide |
| ---- | ---- | --------- |
| **KML Lane Planner** | Watches `missions/` for a KML boundary, converts WGS84 to ENU, builds a boustrophedon lane pattern from the corner nearest home, publishes `LaneSegmentArray` | [guide](drone1_ws/developers_debug/02_KML_LANE_PLANNER_NODE.md) |
| **Navigation** | Auto-arm, takeoff (default 6.7 m / 22 ft), pure-pursuit path following via MAVROS, auto-RTL. `IDLE → ARMING → TAKING_OFF → NAVIGATING → MISSION_COMPLETE → RETURNING_HOME` | [guide](drone1_ws/developers_debug/01_DRONE1_NAVIGATION_NODE.md) |
| **Image Capture** | USB camera interface | [guide](drone1_ws/developers_debug/03_IMAGE_CAPTURE_NODE.md) |
| **Detection & Geotag** | HSV disease detection, CSV log, dedup | [guide](drone1_ws/developers_debug/04_DETECTION_AND_GEOTAG_NODE.md) |
| **Telemetry TX** | Encodes geotags into MAVLink for the radio | [guide](drone1_ws/developers_debug/05_TELEM_TX_NODE.md) |

### Drone-2: sprayer

| Node | Role | Dev guide |
| ---- | ---- | --------- |
| **Telemetry RX** | Decodes geotags, validates, dispatches targets | [guide](drone2_ws/developers_debug/01_TELEM_RX_NODE.md) |
| **Navigation** (unified) | `IDLE → ARM → TAKEOFF → NAVIGATE → ARRIVED → WAIT_SPRAY → WAIT_FOR_NEXT → RTL`, haversine arrival check, timeout RTL | [guide](drone2_ws/developers_debug/02_DRONE2_NAVIGATION_NODE.md) |
| **Detection & Centering** (merged) | Local HSV detection plus PID visual servoing; signals spray-ready when centered | [guide](drone2_ws/developers_debug/03_DETECTION_CENTERING_NODE.md) |
| **Sprayer Control** | Safety checks, then `MAV_CMD_DO_SET_RELAY` on the flight controller; publishes `/drone2/spray_done` | [guide](drone2_ws/developers_debug/04_SPRAYER_CONTROL_NODE.md) |

### GCS

| Node | Role | Dev guide |
| ---- | ---- | --------- |
| **GCS Forwarder** | pymavlink relay D1 → D2 with filtering and dedup | [guide](gcs_ws/developers_debug/01_GCS_FORWARDER_NODE.md) |

Sprayer relay setup on the Cube:

```
RELAY1_PIN = 54        # AUX5
RELAY1_DEFAULT = 0
RELAY1_FUNCTION = 1
```

---

## Quick start

Full instructions: [DEPLOYMENT.md](DEPLOYMENT.md).

```bash
# Dependencies (Ubuntu / ROS 2 Jazzy)
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras \
                 ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-vision-msgs \
                 ros-${ROS_DISTRO}-geographic-msgs
pip3 install scipy geopy pymavlink

# Build each workspace
cd drone1_ws && colcon build --symlink-install && source install/setup.bash
cd ../drone2_ws && colcon build --symlink-install && source install/setup.bash

# Launch (one per vehicle) and run the ground relay on the laptop
ros2 launch drone1_bringup drone1_survey.launch.py
ros2 launch drone2_bringup drone2_sprayer.launch.py
```

Start a mission by dropping a KML boundary into `drone1_ws/missions/`. Drone-1 arms, takes off, surveys, transmits geotags and returns home. Drone-2 waits for the first geotag, then arms, flies, sprays and moves to the next target or returns home.

---

## Testing

SITL needs ArduPilot (`sim_vehicle.py`), MAVROS, and built workspaces.

```bash
./launch_drone1_test.sh     # Drone-1 SITL stack
./launch_drone2_test.sh     # Drone-2 SITL stack
./send_test_geotag.sh       # inject a geotag for Drone-2
```

In SITL only, `param set ARMING_CHECK 0` in MAVProxy disables arming checks. **Never do this on real hardware.**

| Symptom | Fix |
| ------- | --- |
| "FCU timeout" | Start MAVROS first and wait for `FCU: ArduCopter` |
| "No waypoints" | Restart the KML planner after navigation is running |
| "Takeoff timeout" | Wait ~30 s after SITL starts before launching navigation |
| Drone not moving | Check `ros2 topic list \| grep mavros` |

More in [SITL_TESTING_GUIDE.md](SITL_TESTING_GUIDE.md) and [HARDWARE_TESTING.md](HARDWARE_TESTING.md).

---

## Topic reference

### Drone-2 internal

| Topic | Type | Publisher → Subscriber |
| ----- | ---- | ---------------------- |
| `/drone2/target_position` | NavSatFix | telem_rx → drone2_navigation |
| `/drone2/arrival_status` | Bool | drone2_navigation → detection_centering |
| `/drone2/spray_ready` | Bool | detection_centering → sprayer_control |
| `/drone2/spray_done` | Bool | sprayer_control → drone2_navigation |
| `/drone2/navigation_status` | String | drone2_navigation → monitoring |
| `/drone2/detection_status` | Bool | detection_centering → monitoring |

### Cross-drone

| Channel | Type | Notes |
| ------- | ---- | ----- |
| `/drone1/disease_geotag` | GeoPointStamped | Detection output |
| `/mavros/statustext/send` · `/recv` | StatusText | Primary `GEOTAG:lat,lon,alt` |
| `/mavros/debug_value/send` · `/recv` | DebugValue | Fallback `d_lat`, `d_lon`, `d_alt` |

---

## Design rules

| Rule | Reason |
| ---- | ------ |
| ROS never touches the TF-Luna | The flight controller owns sensors |
| No UDP/DDS bridges between drones | No network dependency |
| MAVROS is the only FC interface | Standardized MAVLink |
| MAVLink for inter-drone comms | Reuses the telemetry link |
| One node, one responsibility | Clean architecture |

Algorithms in use: pure-pursuit path following, boustrophedon coverage, ENU/WGS84 conversion, HSV detection with green-context validation, PID visual servoing, haversine distance for arrival and dedup.

---

## Documentation

Every node has a developer guide covering logic, pub/sub, parameters, key functions, dependencies, error handling and test checklists:
[`drone1_ws/developers_debug/`](drone1_ws/developers_debug/) · [`drone2_ws/developers_debug/`](drone2_ws/developers_debug/) · [`gcs_ws/developers_debug/`](gcs_ws/developers_debug/). New to the project? Read the Drone-1 navigation guide first, then detection, then Drone-2 navigation.

---

## License

Apache 2.0, see [LICENSE](LICENSE).

**Made by Shaan Shoukath** · built for precision agriculture with safety-first autonomous drone coordination.
