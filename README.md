# cam_sync_bench

ROS 2 Humble benchmark and live viewer for inter-camera timestamp synchronisation across up to 4 cameras (2 UVC + 2 Intel RealSense), including depth streams.

## What it does

### `sync_bench_node`

Measures the timestamp spread across **N ∈ {2, 3, 4}** `sensor_msgs/Image` streams using `message_filters::ApproximateTime`. N is selected at runtime from the length of the `topics` parameter — no recompilation needed.

Every 2 seconds it prints a rolling-window report and writes matched tuples to CSV via a **threaded writer** (no disk I/O on the executor thread). Starts after a configurable delay (`sync_delay_s`, default 10 s) so cameras have time to initialise before matching begins.

On every match it also publishes a `cam_sync_msgs/SyncedFrames` bundle on `/synced_frames` containing all N frames, all pairwise signed deltas, and the topic labels — so downstream consumers need only one subscription.

**Target:** p99 < 34 ms.

Console output format:
```
[12.0s] matched=355 (+60) cam[0]=360 cam[1]=360 | max-pair spread ms
        p50=6.12 p95=8.40 p99=9.24 max=9.24 mean=6.12 stddev=1.83 | dropped_csv=0
    pair[0,1] mean=6.12 ms
```

- **mean / stddev** — separate pipeline bias from true jitter. A run with `mean=8 ms, stddev=1.8 ms` has only 1.8 ms of real jitter; the 8 ms is a fixed pipeline offset.
- **dropped_csv** — rows discarded by the CSV writer queue (target: 0). Non-zero means disk I/O is the bottleneck.

### `cam_viewer_node`

Live **2×3 grid** viewer for 6 streams — synchronised independently with `ApproximateTime` across all 6 individual topics:

```
[ UVC 0     ] [ RS 0 Color ] [ RS 0 Depth ]
[ UVC 1     ] [ RS 1 Color ] [ RS 1 Depth ]
```

Each panel overlays camera label, rolling FPS, bandwidth (MB/s), hardware timestamp, and spread vs cam[0] colour-coded against the 34 ms budget.

Depth panels are rendered with the **INFERNO** colormap (dark = near, bright = far, 0–8 m range).

### `synced_viewer_node`

Live viewer that subscribes to a single `/synced_frames` topic instead of 6 individual topics. Sync is handled upstream by `sync_bench_node` — this node only renders.

Grid layout adapts to N: **1×N** for N≤3, **2×2** for N=4. Depth panels are auto-detected from `frame.encoding`.

Each panel overlays the **signed delta vs every other camera** decoded from `pairwise_spreads_ms` — positive means this camera's timestamp is later than the peer. This exposes cross-camera relationships that an anchor-only display would hide.

```
[ cam 0                   ] [ cam 1                   ]
  anchor                     vs cam 0  +6.1 ms
                              vs cam 2  -2.3 ms
[ cam 2                   ] [ cam 3                   ]
  vs cam 0  +3.0 ms           vs cam 0 +11.4 ms
  vs cam 1  -2.3 ms           vs cam 1  +5.3 ms
```

Spread colour key: **green** < 17 ms · **yellow** < 34 ms · **red** ≥ 34 ms.

ROS spins on a background thread; OpenCV GUI runs on the main thread.

| Key | Action |
|-----|--------|
| `Space` | Pause — freezes the current frame and stamps **PAUSED** in the centre; ROS keeps running in the background |
| `Space` again | Resume live view |
| `q` | Quit |

### `scripts/opencv_cam_node.py`

Python ROS 2 node that uses `cv2.VideoCapture` (CAP_V4L2) to publish `sensor_msgs/Image` with `bgr8` encoding. Required for devices that only support **YU12/I420** (e.g. DroidCam via v4l2loopback), because `v4l2_camera_node` crashes on those formats. Does not use `cv_bridge` — the Image message is built directly from `frame.tobytes()` to avoid NumPy ABI conflicts. Timestamps use `get_clock().now()` (CLOCK_REALTIME) immediately after `cap.read()`.

### `scripts/analyze_sync.py`

Offline analysis of the CSV output. Auto-detects the number of pairs from the column names, separates bias from jitter, and prints a verdict.

```bash
python3 scripts/analyze_sync.py                       # default: /tmp/sync_bench.csv
python3 scripts/analyze_sync.py /tmp/sync_bench.csv
```

## Hardware tested

| Device | Driver | Interface | Topics |
|--------|--------|-----------|--------|
| Integrated UVC webcam | `v4l2_camera_node` (YUYV) | USB | `/cam_uvc_0/image_raw` |
| DroidCam / v4l2loopback | `opencv_cam_node` (YU12) | USB | `/cam_uvc_1/image_raw` |
| Intel RealSense D435I | `realsense2_camera` v4.x | USB 3.2 | `/camera_0/camera/color/image_raw`, `/camera_0/camera/depth/image_rect_raw` |
| Intel RealSense D415 | `realsense2_camera` v4.x | USB 3.2 | `/camera_1/camera/color/image_raw`, `/camera_1/camera/depth/image_rect_raw` |

> If a RealSense is on USB 2.1, disable its depth stream with `rs_enable_depth_N:=false` — color + depth at 640×480×30 (~46 MB/s) saturates USB 2.1 bandwidth.

## Timestamp clock alignment

UVC cameras (V4L2 / opencv_cam_node) use **CLOCK_REALTIME** (system time). RealSense cameras default to a hardware clock that is unrelated to system time. Without alignment, all cross-camera deltas are meaningless.

`bench.launch.py` sets `enable_global_time_sync: True` on every RealSense node, which enables the SDK's continuous linear regression between hardware clock ticks and CLOCK_REALTIME samples. After this correction, RealSense `header.stamp` values are in the same domain as UVC timestamps and can be compared directly.

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

`cam_sync_msgs` (custom message package) must be built first.

```bash
cd ~/camera_sync_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cam_sync_msgs cam_sync_bench --symlink-install
source install/setup.bash
```

> Always re-run `source install/setup.bash` after building `cam_sync_msgs` — its shared libraries must be on `LD_LIBRARY_PATH` before launching any node that uses `SyncedFrames`.

> `--symlink-install` applies to Python launch files but **not** to `install(PROGRAMS ...)` entries (e.g. `opencv_cam_node`). Script changes to `opencv_cam_node.py` require a rebuild.

## Run

**Terminal 1** — iceoryx shared-memory daemon (required by CycloneDDS SHM transport):
```bash
iox-roudi
```

**Terminal 2** — cameras + benchmark:
```bash
# 2-camera: 1 UVC + 1 RealSense
ros2 launch cam_sync_bench bench.launch.py \
  uvc_device_0:=/dev/video0 \
  rs_serial_0:=052622073756

# 4-camera: 2 UVC + 2 RealSense (all 6 streams including depth)
ros2 launch cam_sync_bench bench.launch.py \
  uvc_device_0:=/dev/video0 \
  uvc_device_1:=/dev/video8 \
  rs_serial_0:=052622073756 \
  rs_serial_1:=145522067777
```

> `uvc_use_opencv_1` defaults to `true` — DroidCam on `/dev/video8` is handled by `opencv_cam_node` automatically.
> `sync_bench_node` starts 10 s after the cameras (configurable via `sync_delay_s`).

**Terminal 3** — viewer (choose one):
```bash
# Original: subscribes to 6 individual topics, always shows 2×3 grid
ros2 run cam_sync_bench cam_viewer_node

# Decoupled: subscribes to /synced_frames, shows all pairwise spreads
ros2 run cam_sync_bench synced_viewer_node
```

**After the run** — analyse the CSV:
```bash
python3 ~/camera_sync_ws/src/cam_sync_bench/scripts/analyze_sync.py
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `uvc_device_0` | `/dev/video0` | V4L2 device path for UVC camera 0 |
| `uvc_device_1` | *(empty — skip)* | V4L2 device path for UVC camera 1 |
| `uvc_pixel_format_0` | `YUYV` | V4L2 pixel format for UVC camera 0 (ignored when `use_opencv=true`) |
| `uvc_pixel_format_1` | `YU12` | V4L2 pixel format for UVC camera 1 (ignored when `use_opencv=true`) |
| `uvc_use_opencv_0` | `false` | Use `opencv_cam_node` for UVC camera 0 (set `true` for YU12/I420 sources) |
| `uvc_use_opencv_1` | `true` | Use `opencv_cam_node` for UVC camera 1 (default `true` — DroidCam only supports YU12) |
| `rs_serial_0` | *(empty — skip)* | Serial number for RealSense 0 |
| `rs_serial_1` | *(empty — skip)* | Serial number for RealSense 1 |
| `rs_enable_depth_0` | `true` | Enable depth stream for RealSense 0 |
| `rs_enable_depth_1` | `true` | Enable depth stream for RealSense 1 (set `false` for USB 2.1 devices) |
| `sync_delay_s` | `10.0` | Seconds before `sync_bench_node` starts (0 = immediate) |
| `slop_s` | `0.034` | ApproximateTime max interval (seconds) |
| `csv_path` | `/tmp/sync_bench.csv` | Output CSV path |
| `topics` | *(auto-built from active devices)* | Override topic list (YAML, 2–4 entries) |

Topics for `sync_bench_node` are auto-derived from whichever color streams are active. Pass `topics:='[...]'` only when you need a custom subset.

To find RealSense serial numbers: `rs-enumerate-devices | grep "Serial Number"`

## Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/cam_uvc_0/image_raw` | `sensor_msgs/Image` | `opencv_cam_node` / `v4l2_camera_node` | UVC camera 0 |
| `/cam_uvc_1/image_raw` | `sensor_msgs/Image` | `opencv_cam_node` | UVC camera 1 |
| `/camera_0/camera/color/image_raw` | `sensor_msgs/Image` | `realsense2_camera` | RealSense 0 color |
| `/camera_0/camera/depth/image_rect_raw` | `sensor_msgs/Image` | `realsense2_camera` | RealSense 0 depth |
| `/camera_1/camera/color/image_raw` | `sensor_msgs/Image` | `realsense2_camera` | RealSense 1 color |
| `/camera_1/camera/depth/image_rect_raw` | `sensor_msgs/Image` | `realsense2_camera` | RealSense 1 depth |
| `/synced_frames` | `cam_sync_msgs/SyncedFrames` | `sync_bench_node` | Bundled N-tuple after each match |

The extra `camera/` segment in RealSense topics is a realsense2_camera v4.x naming convention (namespace + camera_name both default to `camera`).

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
| > 80 ms or no matches | Broken | Check `ros2 topic info -v <topic>` for QoS mismatch |
| matched << min(counts) | Clock drift | Raise `slop_s:=0.050` to confirm; if matches increase, cameras need hardware sync |
| Deltas don't match physical stopwatch | Clock domain mismatch | Verify `enable_global_time_sync: True` is active on RealSense nodes |

A result like `p99=9 ms, mean=8 ms, stddev=1.8 ms` is actually well-synced — the 8 ms is a fixed pipeline offset, not jitter. The stddev is the number that describes real synchronisation quality.

## Design notes

**Why a separate `cam_sync_msgs` package:** ROS 2 message definitions must live in their own package when consumed by nodes in other packages (e.g. a future Python LeRobot recorder). Keeping interfaces separate avoids circular dependencies and lets any downstream package depend on `cam_sync_msgs` without pulling in the full benchmark stack.

**Why `sync_bench_node` publishes `/synced_frames`:** Downstream consumers (Python dataset recorders, ML pipelines) need synchronised frames but should not re-implement synchronisation logic. Publishing a pre-assembled bundle decouples the sync algorithm (C++ / `message_filters`) from consumption (Python / arbitrary processing). The bundle includes `pairwise_spreads_ms` for diagnostic logging without requiring subscribers to recompute timestamps.

**Why N is compile-time, topics are runtime:** `message_filters::ApproximateTime` requires the message type list at compile time. N ∈ {2, 3, 4} are pre-instantiated in `main()`. To support N=5 or N=6, add a new `PolicyHelper<5>` specialisation and a `case 5:` in `main()` — do not replace ApproximateTime with a hand-rolled matcher.

**Why the viewer uses a fixed 6-topic policy:** The viewer hardcodes `ApproxPolicy6` for all 6 streams (4 color + 2 depth). Topic names and depth rendering are configurable via `topic_N`, `label_N`, and `is_depth_N` parameters.

**Why CSV writes are threaded:** At 30 Hz × 4 topics, an `std::ofstream` flush on the executor thread adds ~0.1–2 ms of latency per callback, which perturbs the very timestamps being measured. The `CsvWriter` queue (cap 10000) absorbs bursts; `dropped_csv` in the report surfaces any backpressure.

**Why opencv_cam_node avoids cv_bridge:** The Humble `cv_bridge` Python binding was compiled against NumPy 1.x and raises `AttributeError: _ARRAY_API not found` on NumPy 2.x systems. The node builds `sensor_msgs/Image` directly with no cv_bridge dependency.

**Why opencv_cam_node uses `array.array('B', ...)` for msg.data:** rclpy's generated setter for `uint8[]` fields has two paths — for `array.array` input it does a direct assignment, but for plain `bytes` input it runs `all(isinstance(v, int) for v in value)` which iterates all 921,600 bytes in Python (~50 ms at 640×480), cutting fps from 30 to ~18. Wrapping with `array.array('B', frame.tobytes())` takes the fast path.

**Why opencv_cam_node does not call `rclpy.spin()`:** This node has no subscriptions or timers. Calling `spin()` in a second thread competes with the capture loop for the Python GIL, halving throughput. The capture loop runs on the main thread in a tight `cap.read()` → `publish()` loop.

**Why sync_bench_node starts with a delay:** The `ApproximateTime` synchronizer fills its internal queue from the moment it starts. If cameras are still initialising, partial-startup frames occupy queue slots and reduce match rate. A 10-second `TimerAction` delay gives all cameras time to reach steady-state first.

**Why `enable_global_time_sync: True` is required:** The RealSense SDK defaults to `RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK` — a camera-internal counter unrelated to system time. UVC cameras get `CLOCK_REALTIME` timestamps from the V4L2 kernel subsystem. Without alignment, cross-camera deltas are the difference between two unrelated clocks and cannot be interpreted as synchronisation error.
