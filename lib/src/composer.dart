import 'dart:math' as math;

import 'math.dart';

/// Easing curves for virtual camera transitions.
enum CameraBlendCurve {
  cut,
  linear,
  smoothstep,
  easeInOutCubic;

  /// Evaluates the blend curve in normalized time `[0..1]`.
  double evaluate(double t) {
    final u = t.clamp(0.0, 1.0);
    switch (this) {
      case CameraBlendCurve.cut:
        return u >= 1.0 ? 1.0 : 0.0;
      case CameraBlendCurve.linear:
        return u;
      case CameraBlendCurve.smoothstep:
        return u * u * (3.0 - 2.0 * u);
      case CameraBlendCurve.easeInOutCubic:
        return u < 0.5 ? 4.0 * u * u * u : 1.0 - math.pow(-2.0 * u + 2.0, 3) * 0.5;
    }
  }
}

/// Transition specification between virtual cameras.
final class CameraTransitionSpec {
  final double durationSeconds;
  final CameraBlendCurve curve;

  const CameraTransitionSpec({
    this.durationSeconds = 0.6,
    this.curve = CameraBlendCurve.smoothstep,
  });

  const CameraTransitionSpec.cut()
      : durationSeconds = 0.0,
        curve = CameraBlendCurve.cut;
}

/// Screen dead-zone & soft-zone framing composer with velocity look-ahead.
final class CameraFramingComposer {
  /// World-space offset applied relative to the tracked target when no orbital rig drives position.
  CamVec3 followOffset;

  /// Local offset applied on top of [targetPosition] (e.g. character shoulder or chest height).
  CamVec3 trackedTargetOffset;

  /// Horizontal normalized dead-zone half-width (`0..1` NDC fraction from center).
  /// While the projected target remains inside `[-deadZoneHalfWidth, +deadZoneHalfWidth]`,
  /// lateral framing stays locked to ignore target micro-jitter.
  double deadZoneHalfWidth;

  /// Vertical normalized dead-zone half-height (`0..1` NDC fraction from center).
  double deadZoneHalfHeight;

  /// Horizontal hard limit (`0..1`). Target is immediately clamped inside this boundary.
  double softZoneHalfWidth;

  /// Vertical hard limit (`0..1`). Target is immediately clamped inside this boundary.
  double softZoneHalfHeight;

  /// Exponential damping speed for soft-zone recovery and follow trailing (`0` = instant).
  double damping;

  /// Seconds of velocity look-ahead prediction applied to focus point.
  double lookAheadTime;

  /// Exponential smoothing speed for estimated target velocity.
  double lookAheadSmoothing;

  /// Desired distance bounds along camera forward direction.
  double minFollowDistance;
  double maxFollowDistance;

  /// Assumed viewport aspect ratio when projecting target into camera screen bounds.
  double aspectRatio;

  /// When `true`, rotates the camera to look smoothly toward the framed target.
  bool aimAtTarget;

  CamVec3? _lastTargetPos;
  CamVec3 _smoothedVelocity = CamVec3.zero;

  CameraFramingComposer({
    this.followOffset = const CamVec3(0.0, 2.5, 8.0),
    this.trackedTargetOffset = const CamVec3(0.0, 1.0, 0.0),
    this.deadZoneHalfWidth = 0.12,
    this.deadZoneHalfHeight = 0.12,
    this.softZoneHalfWidth = 0.45,
    this.softZoneHalfHeight = 0.45,
    this.damping = 8.0,
    this.lookAheadTime = 0.2,
    this.lookAheadSmoothing = 6.0,
    this.minFollowDistance = 1.5,
    this.maxFollowDistance = 60.0,
    this.aspectRatio = 16.0 / 9.0,
    this.aimAtTarget = true,
  });

  void resetState() {
    _lastTargetPos = null;
    _smoothedVelocity = CamVec3.zero;
  }

  /// Evaluates updated camera pose tracking [targetPosition].
  CamPose evaluate(
    CamPose currentPose,
    CamVec3 targetPosition,
    double dt, {
    CamVec3? explicitVelocity,
  }) {
    final instantVel = explicitVelocity ??
        ((_lastTargetPos != null && dt > 1e-5)
            ? (targetPosition - _lastTargetPos!) / dt
            : CamVec3.zero);
    _lastTargetPos = targetPosition;

    final velAlpha = camDampFactor(lookAheadSmoothing, dt);
    _smoothedVelocity = CamVec3.lerp(_smoothedVelocity, instantVel, velAlpha);

    final focus = targetPosition + trackedTargetOffset + (_smoothedVelocity * lookAheadTime);

    var nextRot = currentPose.rotation;
    if (aimAtTarget) {
      final lookDir = focus - currentPose.position;
      if (lookDir.lengthSquared > 1e-8) {
        final desiredRot = CamQuat.lookRotation(lookDir);
        final rotAlpha = camDampFactor(damping * 1.25, dt);
        nextRot = CamQuat.slerp(currentPose.rotation, desiredRot, rotAlpha);
      }
    }

    final camRight = nextRot.right;
    final camUp = nextRot.up;
    final camFwd = nextRot.forward;

    final toTarget = focus - currentPose.position;
    final depth = toTarget.dot(camFwd);

    // If the target is behind the camera or framing dead-zone is zeroed, blend straight toward followOffset.
    if (depth <= 0.1 || (deadZoneHalfWidth <= 0.0 && deadZoneHalfHeight <= 0.0)) {
      final desiredPos = focus + followOffset;
      final alpha = camDampFactor(damping, dt);
      return currentPose.copyWith(
        position: CamVec3.lerp(currentPose.position, desiredPos, alpha),
        rotation: nextRot,
      );
    }

    final halfV = depth * math.tan(currentPose.fovYRadians * 0.5);
    final halfH = halfV * math.max(0.1, aspectRatio);

    final rx = toTarget.dot(camRight);
    final ry = toTarget.dot(camUp);
    final nx = halfH > 1e-6 ? rx / halfH : 0.0;
    final ny = halfV > 1e-6 ? ry / halfV : 0.0;

    final softW = math.max(deadZoneHalfWidth, softZoneHalfWidth);
    final softH = math.max(deadZoneHalfHeight, softZoneHalfHeight);

    final dampAlpha = camDampFactor(damping, dt);
    var shiftX = 0.0;
    var shiftY = 0.0;
    var shiftFwd = 0.0;

    if (nx.abs() > softW) {
      // Immediate clamp at soft-zone edge so target cannot escape framing envelope.
      final excessNorm = nx - nx.sign * softW;
      shiftX = excessNorm * halfH;
    } else if (nx.abs() > deadZoneHalfWidth) {
      final excessNorm = nx - nx.sign * deadZoneHalfWidth;
      shiftX = excessNorm * halfH * dampAlpha;
    }

    if (ny.abs() > softH) {
      final excessNorm = ny - ny.sign * softH;
      shiftY = excessNorm * halfV;
    } else if (ny.abs() > deadZoneHalfHeight) {
      final excessNorm = ny - ny.sign * deadZoneHalfHeight;
      shiftY = excessNorm * halfV * dampAlpha;
    }

    if (depth < minFollowDistance) {
      shiftFwd = (depth - minFollowDistance);
    } else if (depth > maxFollowDistance) {
      shiftFwd = (depth - maxFollowDistance) * dampAlpha;
    }

    final newPos = currentPose.position +
        (camRight * shiftX) +
        (camUp * shiftY) +
        (camFwd * shiftFwd);

    return currentPose.copyWith(
      position: newPos,
      rotation: nextRot,
    );
  }
}

/// Callback returning the nearest obstacle distance along a ray `[origin -> origin + direction * maxDistance]`.
typedef CameraObstacleProbe = double? Function(
  CamVec3 origin,
  CamVec3 direction,
  double maxDistance,
);

/// Third-person spherical orbit rig with pitch/distance bounds and raycast obstacle pull-in.
final class CameraOrbitRig {
  double yawRadians;
  double pitchRadians;
  double distance;

  double minPitchRadians;
  double maxPitchRadians;
  double minDistance;
  double maxDistance;

  CamVec3 targetOffset;
  double wallPullInPadding;
  double pullOutDamping;

  double _currentDistance;

  CameraOrbitRig({
    this.yawRadians = 0.0,
    this.pitchRadians = 0.35,
    this.distance = 6.0,
    this.minPitchRadians = -1.2,
    this.maxPitchRadians = 1.35,
    this.minDistance = 1.2,
    this.maxDistance = 25.0,
    this.targetOffset = const CamVec3(0.0, 1.2, 0.0),
    this.wallPullInPadding = 0.25,
    this.pullOutDamping = 6.0,
  }) : _currentDistance = distance;

  double get effectiveDistance => _currentDistance;

  void rotateBy(double deltaYawRadians, double deltaPitchRadians) {
    yawRadians += deltaYawRadians;
    pitchRadians = (pitchRadians + deltaPitchRadians).clamp(minPitchRadians, maxPitchRadians);
  }

  void zoomBy(double deltaDistance) {
    distance = (distance + deltaDistance).clamp(minDistance, maxDistance);
  }

  CamPose evaluate(
    CamPose basePose,
    CamVec3 targetPosition,
    double dt, {
    CameraObstacleProbe? obstacleProbe,
  }) {
    final clampedPitch = pitchRadians.clamp(minPitchRadians, maxPitchRadians);
    final desiredDist = distance.clamp(minDistance, maxDistance);

    // Spherical offset from target anchor to camera eye (at yaw=0, pitch=0 -> +Z).
    final cosP = math.cos(clampedPitch);
    final sinP = math.sin(clampedPitch);
    final sinY = math.sin(yawRadians);
    final cosY = math.cos(yawRadians);
    final rayDir = CamVec3(sinY * cosP, sinP, cosY * cosP).normalized;
    final anchor = targetPosition + targetOffset;

    var targetDist = desiredDist;
    final hitDist = obstacleProbe?.call(anchor, rayDir, desiredDist);
    if (hitDist != null && hitDist < targetDist) {
      targetDist = math.max(0.2, hitDist - wallPullInPadding);
      if (targetDist < _currentDistance) {
        // Immediate pull-in so walls never clip the camera view.
        _currentDistance = targetDist;
      }
    }
    if (targetDist >= _currentDistance) {
      final recoverAlpha = camDampFactor(pullOutDamping, dt);
      _currentDistance += (targetDist - _currentDistance) * recoverAlpha;
    }

    final eye = anchor + (rayDir * _currentDistance);
    final lookDir = (anchor - eye).normalized;
    final rot = CamQuat.lookRotation(lookDir);

    return basePose.copyWith(
      position: eye,
      rotation: rot,
    );
  }
}

/// Multi-octave trauma & directional kick impulse source for camera shake.
final class CameraImpulseSource {
  final String id;
  double decayRate;
  double frequencyHz;
  CamVec3 maxOffset;
  double maxAngleRadians;
  double kickDecayRate;

  double _trauma = 0.0;
  double _phase = 0.0;
  CamVec3 _directionalKick = CamVec3.zero;

  CameraImpulseSource({
    this.id = 'default',
    this.decayRate = 1.6,
    this.frequencyHz = 21.0,
    this.maxOffset = const CamVec3(0.32, 0.32, 0.14),
    this.maxAngleRadians = 0.065,
    this.kickDecayRate = 10.0,
  });

  double get trauma => _trauma;
  bool get isActive => _trauma > 1e-4 || _directionalKick.lengthSquared > 1e-7;

  /// Adds [traumaAmount] clamped in `[0..1]` plus optional directional punch [kick].
  void trigger({
    double traumaAmount = 0.5,
    CamVec3 kick = CamVec3.zero,
  }) {
    if (traumaAmount > 0.0) {
      _trauma = (_trauma + traumaAmount).clamp(0.0, 1.0);
    }
    _directionalKick = _directionalKick + kick;
  }

  /// Triggers an explosion/impact impulse attenuated quadratically by distance from [epicenter].
  void triggerAt(
    CamVec3 epicenter,
    CamVec3 listenerPos, {
    double traumaAmount = 0.7,
    double maxRadius = 25.0,
    double kickStrength = 0.0,
  }) {
    if (maxRadius <= 0.0) return;
    final delta = listenerPos - epicenter;
    final dist = delta.length;
    if (dist >= maxRadius) return;
    final factor = 1.0 - (dist / maxRadius);
    final falloff = factor * factor;
    final dir = dist > 1e-5 ? delta / dist : CamVec3.up;
    trigger(
      traumaAmount: traumaAmount * falloff,
      kick: dir * (kickStrength * falloff),
    );
  }

  void clear() {
    _trauma = 0.0;
    _directionalKick = CamVec3.zero;
    _phase = 0.0;
  }

  /// Advances decay timers and returns world-offset + local rotation shake offsets.
  ({CamVec3 positionOffset, CamQuat rotationOffset}) sample(double dt) {
    if (dt > 0.0) {
      _phase += dt * frequencyHz * (math.pi * 2.0);
      _trauma = math.max(0.0, _trauma - decayRate * dt);
      final kickDecay = math.exp(-kickDecayRate * dt);
      _directionalKick = _directionalKick * kickDecay;
      if (_directionalKick.lengthSquared < 1e-8) {
        _directionalKick = CamVec3.zero;
      }
    }

    if (!isActive) {
      return (positionOffset: CamVec3.zero, rotationOffset: CamQuat.identity);
    }

    final shake = _trauma * _trauma;
    final ox = math.sin(_phase * 1.13 + 0.4) * 0.65 + math.sin(_phase * 2.31) * 0.35;
    final oy = math.cos(_phase * 1.47 + 1.1) * 0.65 + math.sin(_phase * 2.77) * 0.35;
    final oz = math.sin(_phase * 0.91 + 2.3) * 0.65 + math.cos(_phase * 2.09) * 0.35;

    final posOffset = CamVec3(
          ox * maxOffset.x * shake,
          oy * maxOffset.y * shake,
          oz * maxOffset.z * shake,
        ) +
        _directionalKick;

    final rx = math.sin(_phase * 1.61 + 0.7) * maxAngleRadians * shake;
    final ry = math.cos(_phase * 1.29 + 1.9) * maxAngleRadians * shake;
    final rz = math.sin(_phase * 1.97 + 3.1) * maxAngleRadians * shake;

    final rotOffset = CamQuat.fromEuler(
      pitchRadians: rx,
      yawRadians: ry,
      rollRadians: rz,
    );

    return (positionOffset: posOffset, rotationOffset: rotOffset);
  }
}
