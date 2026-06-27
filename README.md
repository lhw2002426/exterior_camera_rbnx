# exterior_camera_rbnx

V4L2 USB-camera publisher for the **exterior** camera in the
`piper_grasp_deploy` VLA pipeline.

This is a thin robonix wrapper around a single `rclpy` node — it opens
a V4L2 device and publishes `sensor_msgs/Image` on a single topic. It
is the minimum-viable primitive package shape: no Driver(CMD_INIT),
no MCP tools, no routed atlas contracts. The package just calls
`RegisterPrimitive` on atlas so `rbnx boot` proceeds, then runs the
camera node and heartbeats every 30s.

Sibling to `OrbbecSDK_rbnx` (which provides the wrist/scene camera +
depth) — use this package for any standard USB color camera that
should also feed the VLA.

## Topic surface

| Topic | Type | QoS | Encoding |
|---|---|---|---|
| `/exterior_camera/color/image_raw` (default, configurable) | `sensor_msgs/Image` | RELIABLE / KEEP_LAST(10) / VOLATILE | `bgr8` (default) or `rgb8` |

No depth, no `camera_info` — this camera is only consumed by
`vla_client_rbnx`, which does not need either.

## Config (via env, not Driver(CMD_INIT))

This package has **no** `*/driver` capability, so the per-package
`config:` block in the deploy manifest is ignored. Use the top-level
`env:` block instead, or export before `rbnx boot`.

| Env | Default | Notes |
|---|---|---|
| `EXTERIOR_CAMERA_DEVICE`   | `/dev/video11` | V4L2 device path |
| `EXTERIOR_CAMERA_TOPIC`    | `/exterior_camera/color/image_raw` | ROS topic name |
| `EXTERIOR_CAMERA_FRAME_ID` | `exterior_camera` | TF frame id stamped on the message header |
| `EXTERIOR_CAMERA_FPS`      | `30.0` | Publish rate target (driver may cap it lower) |
| `EXTERIOR_CAMERA_WIDTH`    | `0`    | `0` = use device default |
| `EXTERIOR_CAMERA_HEIGHT`   | `0`    | `0` = use device default |
| `EXTERIOR_CAMERA_ENCODING` | `bgr8` | One of `bgr8`, `rgb8` |

`vla_client_rbnx`'s `full_image_topic` defaults to
`/camera/color/image_raw`. If you want it to consume this camera
instead, override its config to point at the topic this package
publishes:

```yaml
config:
  full_image_topic: /exterior_camera/color/image_raw
```

## Boot ordering

This package is independent of every other primitive — TF
subtrees, CAN buses, joint streams, none of it is touched. It can
appear anywhere in the `primitive:` list. Conventionally we keep
cameras at the top of the list so they have time to warm up before
downstream services come online.

## Runtime layout

```
exterior_camera_rbnx/
├── package_manifest.yaml
├── scripts/
│   ├── build.sh                       # codegen only
│   ├── start.sh                       # source ROS + exec wrapper
│   └── atlas_register_and_launch.py   # atlas.RegisterPrimitive + spawn node
└── exterior_camera/
    ├── __init__.py
    └── camera_node.py                 # rclpy publisher
```

## Standalone test (no rbnx)

```bash
source /opt/ros/humble/setup.bash
cd /Users/howenliu/lab/packages/exterior_camera_rbnx
EXTERIOR_CAMERA_DEVICE=/dev/video11 \
EXTERIOR_CAMERA_TOPIC=/exterior_camera/color/image_raw \
PYTHONPATH="$PWD:${PYTHONPATH:-}" \
python3 -u -m exterior_camera.camera_node
```

Then in another shell:

```bash
ros2 topic hz /exterior_camera/color/image_raw
ros2 run rqt_image_view rqt_image_view  # optional visual check
```

## Compatibility notes

- The publisher uses `RELIABLE` QoS. `vla_client_rbnx` subscribes with
  `BEST_EFFORT`, which is the *less strict* end and therefore
  compatible (DDS QoS matching rule).
- The message `encoding` field is set to `bgr8` by default. `vla_client_rbnx`'s
  `_decode_ros_image()` handles both `bgr8` and `rgb8`, so no
  client-side change is needed.
- Header `frame_id` is `exterior_camera` by default; it is **not**
  joined to any TF tree by this package. Add a static transform
  elsewhere (e.g. `easy_handeye2_rbnx`) if you need camera→robot
  extrinsics.

## Why a separate package and not just `fake_camera_pub`?

`fake_camera_pub` ships a static image to validate downstream
pipelines without real hardware. This package is the **real** camera
counterpart — it owns a V4L2 device and publishes live frames at
a target rate, designed to be a peer of `OrbbecSDK_rbnx` /
`realsense_camera_rbnx` in a deploy manifest.
