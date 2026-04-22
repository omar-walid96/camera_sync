# cam_sync_bench

ROS 2 Humble benchmark and viewer for inter-camera timestamp synchronisation between a UVC webcam and an Intel RealSense D435I.

## What it does

### `sync_bench_node`

Measures the timestamp spread between two `sensor_msgs/Image` streams using `message_filters::ApproximateTime`. Every 2 seconds it prints a rolling p50/p95/p99/max plus **bias and jitter** to the console. All matched pairs are buffered in memory and written to CSV on exit (no I/O on the executor thread).

**Target:** p99 < 34 ms.

Console output format:
```
[12.0s] matched=355 (+60) a=360 b=360 | spread ms p50=6.12 p95=8.40 p99=9.24 max=9.24 | bias=6.12 stddev=1.83
```

- **bias** — mean of signed `(t_a − t_b)`. A consistent non-zero value is a fixed pipeline offset (V4L2 kernel-arrival lag vs RealSense hardware timestamp), not fixable by software sync.
- **stddev** — the actual jitter. This is the number that describes real synchronisation quality.

### `cam_viewer_node`

Side-by-side live viewer for three streams — UVC color, RealSense color, RealSense depth — synchronised with the same ApproximateTime policy. Each panel overlays:

- Camera label, rolling FPS, and bandwidth (MB/s) over a 2-second window
- Hardware timestamp (seconds since epoch)
- Three spread readings colour-coded against the 34 ms budget:
  - **UVC ↔ RS Color** — main sync metric
  - **RS Color ↔ RS Depth** — internal RealSense hardware sync (typically < 1 ms on D435I)
  - **UVC ↔ RS Depth** — full pipeline spread

Depth is rendered with the INFERNO colormap (dark = near, bright = far, 0–8 m range).

Spread colour key: **green** < 17 ms · **yellow** < 34 ms · **red** ≥ 34 ms.

### `scripts/analyze_sync.py`

Offline analysis of the CSV output. Separates bias from jitter and prints a verdict.

```bash
python3 scripts/analyze_sync.py                       # default: /tmp/sync_bench.csv
python3 scripts/analyze_sync.py /tmp/sync_bench.csv
```

Output includes signed-delta describe(), bias, jitter (stddev), spread percentiles with ✓/✗ against the 34 ms budget, and a one-line verdict. A `positive: ~100%` reading means the spread is dominated by a fixed pipeline bias rather than random jitter.

## Hardware tested

| Device | Interface | Topic |
|--------|-----------|-------|
| Integrated UVC webcam | USB via v4l2_camera | `/cam_uvc/image_raw` |
| Intel RealSense D435I | USB 3.2 via realsense2_camera v4.57 | `/camera/camera/color/image_raw` |
| RealSense depth stream | same device | `/camera/camera/depth/image_rect_raw` |

> Both devices share USB controller `0000:00:14.0`. If p99 exceeds budget, try putting them on separate controllers.

## Prerequisites

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-v4l2-camera \
                 ros-humble-realsense2-camera \
                 ros-humble-rmw-cyclonedds-cpp

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml

pip install pandas numpy   # for analyze_sync.py
```

## Build

```bash
cd ~/camera_sync_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cam_sync_bench --symlink-install
source install/setup.bash
```

## Run

**Terminal 1** — iceoryx shared-memory daemon (required by CycloneDDS SHM transport):
```bash
iox-roudi
```

**Terminal 2** — cameras + benchmark:
```bash
ros2 launch cam_sync_bench bench.launch.py uvc_device:=/dev/video0
```

**Terminal 3** — live viewer:
```bash
ros2 run cam_sync_bench cam_viewer_node
```

Press `q` in the viewer window to quit.

**After the run** — analyse the CSV:
```bash
python3 ~/camera_sync_ws/src/cam_sync_bench/scripts/analyze_sync.py
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `uvc_device` | `/dev/video0` | V4L2 device path for the UVC webcam |
| `width` | `640` | UVC capture width |
| `height` | `480` | UVC capture height |
| `fps` | `30` | UVC target frame rate |
| `slop_s` | `0.034` | ApproximateTime max interval (seconds) |
| `csv_path` | `/tmp/sync_bench.csv` | Output CSV path |

## CSV output

`/tmp/sync_bench.csv` — written on node shutdown, one row per matched pair:

```
t_wall_ns,t_a_ns,t_b_ns,delta_ms,spread_ms
```

- `delta_ms` — signed `(t_a − t_b)`. Consistently positive means UVC timestamps are later than RealSense (expected: V4L2 stamps kernel arrival, not exposure).
- `spread_ms` — `|delta_ms|`, used for percentile budget checks.

## Interpreting results

| p99 | Verdict | Likely fix |
|-----|---------|------------|
| < 34 ms | Within budget | — |
| 34–80 ms | Over budget | `echo 1000 \| sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`; check USB topology with `lsusb -t` |
| > 80 ms or no matches | Broken | Check `ros2 topic info -v /camera/camera/color/image_raw` for QoS mismatch |
| matched << min(a, b) | Clock drift | Raise `slop_s:=0.050` to confirm; if matches increase, cameras need hardware sync |

A low p99 with a high bias and low stddev (e.g. `p99=9 ms, bias=8 ms, stddev=1.8 ms`) means the cameras are well-synchronised — the 8 ms is a fixed pipeline offset that can be subtracted, not jitter.
