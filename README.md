# cam_sync_bench

ROS 2 Humble benchmark and viewer for inter-camera timestamp synchronisation between a UVC webcam and an Intel RealSense D435I.

## What it does

### `sync_bench_node`

Measures the timestamp spread across **N ∈ {2, 3, 4}** `sensor_msgs/Image` streams using `message_filters::ApproximateTime`. N is selected at runtime from the length of the `topics` parameter — no recompilation needed to add a third or fourth camera.

Every 2 seconds it prints a rolling window report to the console and writes matched pairs to CSV via a **threaded writer** (no disk I/O on the executor thread).

**Target:** p99 < 34 ms.

Console output format:
```
[12.0s] matched=355 (+60) cam[0]=360 cam[1]=360 | max-pair spread ms
        p50=6.12 p95=8.40 p99=9.24 max=9.24 mean=6.12 stddev=1.83 | dropped_csv=0
    pair[0,1] mean=6.12 ms
```

- **mean / stddev** — separate pipeline bias from true jitter. A run with `mean=8 ms, stddev=1.8 ms` has only 1.8 ms of real jitter; the 8 ms is a fixed V4L2 kernel-arrival offset that software sync cannot fix.
- **dropped_csv** — rows discarded by the CSV writer queue (target: 0). Non-zero means disk I/O is the bottleneck.

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

Offline analysis of the CSV output. Auto-detects the number of pairs from the column names, separates bias from jitter, and prints a verdict.

```bash
python3 scripts/analyze_sync.py                       # default: /tmp/sync_bench.csv
python3 scripts/analyze_sync.py /tmp/sync_bench.csv
```

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

**Terminal 2** — cameras + benchmark (2-camera default):
```bash
ros2 launch cam_sync_bench bench.launch.py uvc_device_0:=/dev/video0
```

**4-camera example** (2 UVC + 2 RealSense):
```bash
ros2 launch cam_sync_bench bench.launch.py \
  uvc_device_0:=/dev/video0 \
  uvc_device_1:=/dev/video8 \
  rs_serial_0:=052622073756 \
  rs_serial_1:=<second_serial>
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
| `uvc_device_0` | `/dev/video0` | V4L2 device path for UVC camera 0 |
| `uvc_device_1` | *(empty — skip)* | V4L2 device path for UVC camera 1 |
| `rs_serial_0` | *(empty — auto-discover)* | Serial number for RealSense 0 |
| `rs_serial_1` | *(empty — skip)* | Serial number for RealSense 1 |
| `slop_s` | `0.034` | ApproximateTime max interval (seconds) |
| `csv_path` | `/tmp/sync_bench.csv` | Output CSV path |
| `topics` | *(auto-built from active devices)* | Override topic list (YAML, 2–4 entries) |

Topics are derived automatically from whichever devices are configured. Pass `topics:='[...]'` only if you need a custom subset (e.g. depth stream instead of color).

### Multi-device usage

**2-camera default** (1 UVC + 1 RealSense, auto-discover RealSense):
```bash
ros2 launch cam_sync_bench bench.launch.py uvc_device_0:=/dev/video0
```

**2-camera with explicit RealSense serial** (required when two RealSense devices are connected):
```bash
ros2 launch cam_sync_bench bench.launch.py \
  uvc_device_0:=/dev/video0 \
  rs_serial_0:=052622073756
```

**4-camera** (2 UVC + 2 RealSense):
```bash
ros2 launch cam_sync_bench bench.launch.py \
  uvc_device_0:=/dev/video0 \
  uvc_device_1:=/dev/video8 \
  rs_serial_0:=052622073756 \
  rs_serial_1:=<second_serial>
```

Active topics for the 4-camera case:
- `/cam_uvc_0/image_raw`
- `/camera_0/camera/color/image_raw`
- `/cam_uvc_1/image_raw`
- `/camera_1/camera/color/image_raw`

To find RealSense serial numbers: `rs-enumerate-devices | grep "Serial Number"`

## CSV output

`/tmp/sync_bench.csv` — written on node shutdown via the threaded CsvWriter, one row per matched N-tuple:

```
t_wall_ns, t_0_ns, …, t_{N-1}_ns, delta_0_1_ms, [delta_0_2_ms, …]
```

- `delta_I_J_ms` — signed `(t_I − t_J)`. Consistently positive means stream I timestamps are later than stream J.
- With N=2: 1 delta column. N=3: 3 columns. N=4: 6 columns.

## Interpreting results

| p99 | Verdict | Likely fix |
|-----|---------|------------|
| < 34 ms | Within budget | — |
| 34–80 ms | Over budget | `echo 1000 \| sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`; check USB topology with `lsusb -t` |
| > 80 ms or no matches | Broken | Check `ros2 topic info -v /camera/camera/color/image_raw` for QoS mismatch |
| matched << min(counts) | Clock drift | Raise `slop_s:=0.050` to confirm; if matches increase, cameras need hardware sync |

A result like `p99=9 ms, mean=8 ms, stddev=1.8 ms` is actually well-synced — the 8 ms is a fixed V4L2 pipeline offset, not jitter. The stddev is the number that describes real synchronisation quality.

## Design notes

**Why N is compile-time, topics are runtime:** `message_filters::ApproximateTime` requires the message type list at compile time. N ∈ {2, 3, 4} are pre-instantiated in `main()`. To support N=5 or N=6, add new `PolicyHelper<5>` specialisation and a `case 5:` in `main()` — do not replace ApproximateTime with a hand-rolled matcher.

**Why CSV writes are threaded:** At 30 Hz × 4 topics, an `std::ofstream` flush on the executor thread adds ~0.1–2 ms of latency per callback, which perturbs the very timestamps being measured. The `CsvWriter` queue (cap 10000) absorbs bursts; `dropped_csv` in the report surfaces any backpressure.
