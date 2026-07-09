# MAVLink → Foxglove Bridge

Reads MAVLink messages from a Serial or UDP connection and publishes them to
[Foxglove](https://foxglove.dev) using the Python SDK with well-known schemas
for native 3D and Map panel visualization.

## Features

- **Serial** (default) or **UDP** transport — configurable via CLI
- **Multi-UAV Support** via automatic MAVLink System ID detection:
  - Dynamically discovers system IDs on the stream.
  - Namespaces topics by System ID to keep streams separated in Foxglove.
- **Foxglove well-known schemas** for zero-config visualization:
  - `PoseInFrame` on `/uav_{sys_id}/pose` — attitude and position in 3D panel.
  - `FrameTransform` on `/tf` — transforms for each `uav_{sys_id}` relative to `world`.
  - `LocationFix` on `/uav_{sys_id}/gps` — GPS position in Map panel.
- **GPS-to-Local ENU Projection**: Projects each UAV's absolute GPS coordinates into local `(x, y, z)` coordinates in meters relative to the first received GPS coordinate, allowing them to be visualized in their correct relative 3D positions without overlapping at the origin.
- **Raw JSON topics** for detailed plotting:
  - `mavlink/uav_{sys_id}/global_position` — lat, lon, alt, velocities, heading
  - `mavlink/uav_{sys_id}/gps_raw` — fix type, satellites, HDOP/VDOP
  - `mavlink/uav_{sys_id}/heartbeat` — armed state, flight mode, system status
- **NED → ENU** coordinate conversion for Foxglove's 3D panel
- Auto serial port detection and reconnection on link loss

## MAVLink Messages

| MAVLink Message        | Foxglove Output                                                         |
| :--------------------- | :---------------------------------------------------------------------- |
| `ATTITUDE`             | `PoseInFrame` + `FrameTransform` (for `/uav_{sys_id}/pose` & `/tf`)     |
| `GLOBAL_POSITION_INT`  | `LocationFix` (for `/uav_{sys_id}/gps`) + Local ENU Translation + JSON  |
| `GPS_RAW_INT`          | JSON topic `mavlink/uav_{sys_id}/gps_raw`                               |
| `HEARTBEAT`            | JSON topic `mavlink/uav_{sys_id}/heartbeat`                             |

## Quick Start

### Prerequisites

- Python ≥ 3.12
- [uv](https://docs.astral.sh/uv/) package manager

### List available serial ports

```sh
uv run main.py --list-ports
```

### Connect via Serial (default)

```sh
# Auto-detect port (interactive prompt)
uv run main.py

# Specify port explicitly
uv run main.py --port /dev/tty.usbmodemXXXX --baud 115200
```

### Connect via UDP (e.g. SITL)

```sh
# Default: 127.0.0.1:14550
uv run main.py --udp

# Custom address
uv run main.py --udp 192.168.1.100:14550
```

### All options

```sh
uv run main.py --help
```

```
usage: main.py [-h] [--port PORT | --udp [HOST:PORT]] [--baud BAUD]
               [--foxglove-host FOXGLOVE_HOST] [--foxglove-port FOXGLOVE_PORT]
               [--reconnect-delay RECONNECT_DELAY] [--list-ports]
               [--log-level {DEBUG,INFO,WARNING,ERROR}]
```

## Foxglove Setup

1. Start the bridge.
2. Open [Foxglove](https://app.foxglove.dev) (or the desktop app).
3. Connect via **Open connection** → **Foxglove WebSocket** → `ws://127.0.0.1:8765`.
4. Add panels:
   - **3D** panel → Set the Display Frame (e.g. to `world`) and add the TF / Pose layers. You'll see frame axes or models for each active UAV (`uav_1`, `uav_2`, etc.) positioned relative to `world` according to their projected GPS meters.
   - **Map** panel → Under the topic list, enable `/uav_{sys_id}/gps` for each UAV to see their real-time positions on the map.
   - **Plot** panel → Subscribe to `mavlink/uav_{sys_id}/*` topics (e.g., `mavlink/uav_1/global_position/relative_alt_m`) to plot flight telemetry fields.
