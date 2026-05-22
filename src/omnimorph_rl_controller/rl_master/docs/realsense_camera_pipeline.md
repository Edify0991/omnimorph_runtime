# RealSense Camera Pipeline

## 1. Runtime roles

For a physical RealSense camera, you need two ROS programs:

1. `realsense2_camera` driver:
   opens the USB device and publishes the native camera topics.
2. `realsense_camera_bridge.py`:
   republishes those topics into the OmniMorph runtime's standardized topic set,
   resizes frames, emits compressed streams, and publishes a compact
   `camera_features` vector for `external_observations`.

Connecting the camera alone is not enough. Without the driver process, there is
no ROS image stream to bridge.

## 2. Standardized topics

The bridge normalizes the runtime-facing camera topics to:

- `/omnimorph/camera/color/image_raw`
- `/omnimorph/camera/color/camera_info`
- `/omnimorph/camera/color/image_raw/compressed`
- `/omnimorph/camera/depth/image_raw`
- `/omnimorph/camera/depth/camera_info`
- `/omnimorph/camera/depth/image_raw/compressed`
- `/omnimorph/camera/features`

Current default stream contract:

- color: `rgb8`, JPEG compressed
- depth: `16UC1`, PNG compressed
- default size: `424x240`
- default fps: `15`
- feature vector: 8 dims
  - color mean R/G/B
  - grayscale contrast std
  - center depth
  - mean valid depth
  - valid depth ratio
  - near-depth ratio

## 3. Startup

Driver only:

```bash
./script/start_realsense_driver.sh
```

Bridge only:

```bash
./script/start_realsense_bridge.sh
```

Driver + bridge together:

```bash
./script/start_realsense_stack.sh
```

If your deployment requires root for device access, you can run the same script
with `sudo`:

```bash
sudo -E ./script/start_realsense_stack.sh
```

## 4. Entering `external_observations`

The current AMP full-body profile already declares:

```yaml
external_observations:
  - name: camera_features
    dim: 8
    topic: /omnimorph/camera/features
```

That means the runtime already does this:

`/omnimorph/camera/features`
-> `SolverDdsBridge` subscriber
-> `ExternalObservationProvider`
-> `ObservationFeatureContext.named_features["camera_features"]`

## 5. Entering the policy input

There are two distinct cases:

1. Append the camera feature into the observation vector:
   use a manifest term like:

```yaml
- name: feature
  source: camera_features
  count: 8
```

2. Feed the camera feature as a separate ONNX input:
   declare a `policy_io.onnx_inputs` item with `source: feature`.

The current `Jingchu01-AMP-Flat_model_18500.onnx` profile is still configured
for `obs_dim: 93`, so you should not directly append the 8-dim camera feature
to that active manifest unless the ONNX was retrained/exported for the new
input size.

For a camera-conditioned observation example, see:

- `config/observation_manifest_amp_mjlab_jc01_full_body_camera.yaml`
