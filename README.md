# exterior_camera_rbnx

V4L2 USB-camera primitive for the **exterior** view in the
`piper_grasp_deploy` Robonix VLA pipeline.

A thin robonix wrapper around a single `rclpy` publisher — it opens
a V4L2 device and publishes `sensor_msgs/Image`. Owns the
`robonix/primitive/camera_exterior/*` namespace.

## Lifecycle

Standard primitive driver shape — same as `piper_ctl_rbnx`:

| Phase | What happens |
|---|---|
| `CMD_INIT` | Light cfg validation. No I/O. |
| `CMD_ACTIVATE` | Spawn the camera publisher subprocess, wait for the first `sensor_msgs/Image` on the configured topic (proves the V4L2 device is alive), atlas-declare `image`. |
| `CMD_DEACTIVATE` | SIGTERM the subprocess. Idempotent. |
| `CMD_SHUTDOWN` | Last-chance kill. Idempotent. |

## Capability surface

| Contract | Mode | Transport | Notes |
|---|---|---|---|
| `robonix/primitive/camera_exterior/driver` | rpc | gRPC | auto-declared by framework |
| `robonix/primitive/camera_exterior/image`  | topic_out | ROS 2 | `sensor_msgs/Image`, RELIABLE QoS |

Both contracts are defined at PACKAGE level (see
`capabilities/primitive/camera_exterior/*.v1.toml`). They use a
different namespace from `OrbbecSDK_rbnx` (which owns
`primitive/camera/*`) because the exterior camera is a separate
physical device serving a different consumer (vla_client's
"full image" input).

## Config

Delivered via `Driver(CMD_INIT, config_json)` from the manifest's
per-package `config:` block. **NOT** via env (env-based config was
the v0.1 shape; v0.2 is fully driver-managed).

| Key | Default | Notes |
|---|---|---|
| `device`             | `/dev/video11` | V4L2 device path |
| `topic`              | `/exterior_camera/color/image_raw` | Where the publisher publishes; declared on atlas as `camera_exterior/image` |
| `frame_id`           | `exterior_camera` | TF frame stamped on the message header |
| `fps`                | `30.0` | Publish rate target (V4L2 driver may cap lower) |
| `width`              | `0` | `0` = use device default |
| `height`             | `0` | `0` = use device default |
| `encoding`           | `bgr8` | `bgr8` (OpenCV native) or `rgb8`. `vla_client_rbnx` decodes both. |
| `buffer_size`        | `1` | V4L2 internal buffer count; `1` minimizes capture latency |
| `sentinel_timeout_s` | `10.0` | Max time `on_activate` waits for the first frame |

Example deploy manifest snippet:

```yaml
- name: exterior_camera
  url: https://github.com/lhw2002426/exterior_camera_rbnx
  branch: main
  config:
    device:   "/dev/video11"
    topic:    "/exterior_camera/color/image_raw"
    frame_id: "exterior_camera"
    fps:      30
    encoding: "bgr8"
    sentinel_timeout_s: 10.0
```

## Wiring with `vla_client_rbnx`

`vla_client_rbnx`'s `full_image_topic` defaults to
`/camera/color/image_raw` (Orbbec). To consume the exterior camera
instead, override:

```yaml
- name: vla_client
  url: https://github.com/lhw2002426/vla_client_rbnx
  branch: binary-gripper-v3
  config:
    full_image_topic:  /exterior_camera/color/image_raw
    wrist_image_topic: /wrist_camera/color/image_raw
    ...
```

## Boot ordering

Independent of every other primitive (no TF deps, no CAN deps, no
joint-stream deps). Can appear anywhere in the `primitive:` list.

## Runtime layout

```
exterior_camera_rbnx/
├── package_manifest.yaml
├── README.md
├── capabilities/primitive/camera_exterior/
│   ├── driver.v1.toml          # lifecycle/srv/Driver.srv
│   └── image.v1.toml           # sensor_msgs/Image (topic_out)
├── scripts/
│   ├── build.sh                # rbnx codegen
│   └── start.sh                # source ROS + python3 -m exterior_camera.main
└── exterior_camera/
    ├── __init__.py
    ├── main.py                 # Primitive driver — lifecycle + atlas declare
    └── camera_node.py          # standalone rclpy publisher (spawned by main.py)
```

## Standalone test (no rbnx)

The publisher node accepts CLI flags so you can run it directly:

```bash
source /opt/ros/humble/setup.bash
cd ~/packages/exterior_camera_rbnx
PYTHONPATH="$PWD:${PYTHONPATH:-}" \
python3 -u -m exterior_camera.camera_node \
    --device /dev/video11 \
    --topic /exterior_camera/color/image_raw \
    --fps 30 \
    --encoding bgr8
```

Then in another shell:

```bash
ros2 topic hz /exterior_camera/color/image_raw   # ~30 Hz
ros2 run rqt_image_view rqt_image_view           # visual check
```

## QoS / encoding notes

- The publisher uses RELIABLE QoS. `vla_client_rbnx` subscribes with
  BEST_EFFORT — DDS QoS matching: publisher must be at least as
  strict as subscriber, so RELIABLE → BEST_EFFORT is compatible.
- The `encoding` field is set to `bgr8` by default. `vla_client_rbnx`'s
  `_decode_ros_image()` handles both `bgr8` and `rgb8`.
- Header `frame_id` defaults to `exterior_camera`; not joined to any
  TF tree by this package. Add a static transform elsewhere
  (e.g. `easy_handeye2_rbnx`) if you need camera→robot extrinsics.
