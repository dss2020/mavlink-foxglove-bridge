# MAVLink → Foxglove Bridge

Reads MAVLink messages from a Serial or UDP connection and publishes them to
[Foxglove](https://foxglove.dev) using the Python SDK with well-known schemas
for native 3D and Map panel visualization.

## Features

- **Serial** (default) or **UDP** transport — configurable via CLI
- **Foxglove well-known schemas** for zero-config visualization:
  - `PoseInFrame` on `/uav/pose` — attitude in the 3D panel
  - `FrameTransform` on `/tf` — UAV frame relative to world
  - `LocationFix` on `/uav/gps` — GPS position in the Map panel
- **Raw JSON topics** for detailed plotting:
  - `mavlink/global_position` — lat, lon, alt, velocities, heading
  - `mavlink/gps_raw` — fix type, satellites, HDOP/VDOP
  - `mavlink/heartbeat` — armed state, flight mode, system status
- **NED → ENU** coordinate conversion for Foxglove's 3D panel
- Auto serial port detection and reconnection on link loss

## MAVLink Messages

| MAVLink Message        | Foxglove Output                               |
| :--------------------- | :-------------------------------------------- |
| `ATTITUDE`             | `PoseInFrame` + `FrameTransform` (3D panel)   |
| `GLOBAL_POSITION_INT`  | `LocationFix` (Map panel) + JSON              |
| `GPS_RAW_INT`          | JSON topic `mavlink/gps_raw`                  |
| `HEARTBEAT`            | JSON topic `mavlink/heartbeat`                |

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

1. Start the bridge
2. Open [Foxglove](https://app.foxglove.dev) (or the desktop app)
3. Connect via **Open connection** → **Foxglove WebSocket** → `ws://127.0.0.1:8765`
4. Add panels:
   - **3D** panel → set display frame to `uav` to see attitude
   - **Map** panel → auto-detects `/uav/gps` for position
   - **Plot** panel → subscribe to `mavlink/*` topics for raw data
