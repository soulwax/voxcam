# voxcam

Pure Dart Cinemachine-style virtual camera brain and procedural camera rig toolkit.

## Features

- **Virtual Camera Priority Stack (`CameraDirectorBrain`)**: Automatically selects the highest-priority active `VirtualCameraRig` and cross-blends seamlessly across pose, FOV, clip planes, and Dutch roll.
- **Per-Pair Blend Curves (`CameraBlendCurve`, `CameraTransitionSpec`)**: Supports `cut`, `linear`, `smoothstep`, and `easeInOutCubic` curves, custom transition overrides, and pop-free mid-blend interruptions.
- **Screen Dead-Zone & Soft-Zone Framing (`CameraFramingComposer`)**: Keeps targets comfortably framed inside configurable normalized dead/soft zones with damped look-ahead motion prediction.
- **Obstacle-Aware Orbit Rig (`CameraOrbitRig`)**: Bounded spherical yaw/pitch/distance rig with smooth terrain/wall raycast pull-in via user-supplied probes.
- **Multi-Source Trauma Impulse (`CameraImpulseSource`)**: Quadratic trauma-based 3D offset & rotational tremor plus directional kick impulses and distance attenuation.
