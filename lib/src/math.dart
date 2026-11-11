import 'dart:math' as math;

/// Lightweight immutable 3D vector used by `voxcam`.
final class CamVec3 {
  final double x;
  final double y;
  final double z;

  const CamVec3(this.x, this.y, this.z);

  static const CamVec3 zero = CamVec3(0.0, 0.0, 0.0);
  static const CamVec3 one = CamVec3(1.0, 1.0, 1.0);
  static const CamVec3 right = CamVec3(1.0, 0.0, 0.0);
  static const CamVec3 up = CamVec3(0.0, 1.0, 0.0);
  static const CamVec3 forward = CamVec3(0.0, 0.0, -1.0);

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  CamVec3 get normalized {
    final len = length;
    if (len < 1e-9) return zero;
    final inv = 1.0 / len;
    return CamVec3(x * inv, y * inv, z * inv);
  }

  CamVec3 operator +(CamVec3 other) => CamVec3(x + other.x, y + other.y, z + other.z);
  CamVec3 operator -(CamVec3 other) => CamVec3(x - other.x, y - other.y, z - other.z);
  CamVec3 operator -() => CamVec3(-x, -y, -z);
  CamVec3 operator *(double scalar) => CamVec3(x * scalar, y * scalar, z * scalar);
  CamVec3 operator /(double scalar) => CamVec3(x / scalar, y / scalar, z / scalar);

  double dot(CamVec3 other) => x * other.x + y * other.y + z * other.z;

  CamVec3 cross(CamVec3 other) => CamVec3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x,
      );

  double distanceTo(CamVec3 other) => (this - other).length;

  static CamVec3 lerp(CamVec3 a, CamVec3 b, double t) => CamVec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
      );

  @override
  String toString() =>
      'CamVec3(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)}, ${z.toStringAsFixed(3)})';
}

/// Unit quaternion using right-handed `-Z` forward, `+Y` up convention.
final class CamQuat {
  final double x;
  final double y;
  final double z;
  final double w;

  const CamQuat(this.x, this.y, this.z, this.w);

  static const CamQuat identity = CamQuat(0.0, 0.0, 0.0, 1.0);

  factory CamQuat.fromAxisAngle(CamVec3 axis, double angleRadians) {
    final n = axis.normalized;
    if (n.lengthSquared < 1e-12) return identity;
    final half = angleRadians * 0.5;
    final s = math.sin(half);
    return CamQuat(n.x * s, n.y * s, n.z * s, math.cos(half));
  }

  /// Creates an orientation whose local `-Z` points along [forwardDir] and
  /// local `+Y` aligns as closely as possible with [worldUp].
  factory CamQuat.lookRotation(CamVec3 forwardDir, [CamVec3 worldUp = CamVec3.up]) {
    if (forwardDir.lengthSquared < 1e-12) return identity;
    final z = -forwardDir.normalized;
    var xAxis = worldUp.cross(z);
    if (xAxis.lengthSquared < 1e-10) {
      final fallbackUp = z.y.abs() < 0.99 ? CamVec3.up : CamVec3.right;
      xAxis = fallbackUp.cross(z);
    }
    xAxis = xAxis.normalized;
    final yAxis = z.cross(xAxis);

    final m00 = xAxis.x, m01 = yAxis.x, m02 = z.x;
    final m10 = xAxis.y, m11 = yAxis.y, m12 = z.y;
    final m20 = xAxis.z, m21 = yAxis.z, m22 = z.z;
    final trace = m00 + m11 + m22;
    if (trace > 0.0) {
      final s = math.sqrt(trace + 1.0) * 2.0;
      return CamQuat(
        (m21 - m12) / s,
        (m02 - m20) / s,
        (m10 - m01) / s,
        0.25 * s,
      ).normalized;
    }
    if (m00 > m11 && m00 > m22) {
      final s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0;
      return CamQuat(
        0.25 * s,
        (m01 + m10) / s,
        (m02 + m20) / s,
        (m21 - m12) / s,
      ).normalized;
    }
    if (m11 > m22) {
      final s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0;
      return CamQuat(
        (m01 + m10) / s,
        0.25 * s,
        (m12 + m21) / s,
        (m02 - m20) / s,
      ).normalized;
    }
    final s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0;
    return CamQuat(
      (m02 + m20) / s,
      (m12 + m21) / s,
      0.25 * s,
      (m10 - m01) / s,
    ).normalized;
  }

  /// Creates a camera orientation from intrinsic Y-X-Z Euler angles (radians).
  factory CamQuat.fromEuler({
    double pitchRadians = 0.0,
    double yawRadians = 0.0,
    double rollRadians = 0.0,
  }) {
    final qYaw = CamQuat.fromAxisAngle(CamVec3.up, yawRadians);
    final qPitch = CamQuat.fromAxisAngle(CamVec3.right, pitchRadians);
    final qRoll = CamQuat.fromAxisAngle(CamVec3.forward, rollRadians);
    return (qYaw * qPitch * qRoll).normalized;
  }

  double get lengthSquared => x * x + y * y + z * z + w * w;

  CamQuat get normalized {
    final lenSq = lengthSquared;
    if (lenSq < 1e-12) return identity;
    final inv = 1.0 / math.sqrt(lenSq);
    return CamQuat(x * inv, y * inv, z * inv, w * inv);
  }

  CamQuat get inverse => CamQuat(-x, -y, -z, w);

  double dot(CamQuat other) => x * other.x + y * other.y + z * other.z + w * other.w;

  CamQuat operator *(CamQuat other) => CamQuat(
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w,
        w * other.w - x * other.x - y * other.y - z * other.z,
      );

  CamVec3 rotate(CamVec3 v) {
    final u = CamVec3(x, y, z);
    final uv = u.cross(v);
    final uuv = u.cross(uv);
    return v + (uv * (2.0 * w)) + (uuv * 2.0);
  }

  CamVec3 get forward => rotate(CamVec3.forward);
  CamVec3 get right => rotate(CamVec3.right);
  CamVec3 get up => rotate(CamVec3.up);

  static CamQuat slerp(CamQuat a, CamQuat b, double t) {
    final cl = t.clamp(0.0, 1.0);
    var bx = b.x, by = b.y, bz = b.z, bw = b.w;
    var cosHalf = a.x * bx + a.y * by + a.z * bz + a.w * bw;
    if (cosHalf < 0.0) {
      bx = -bx;
      by = -by;
      bz = -bz;
      bw = -bw;
      cosHalf = -cosHalf;
    }
    if (cosHalf > 0.9995) {
      return CamQuat(
        a.x + (bx - a.x) * cl,
        a.y + (by - a.y) * cl,
        a.z + (bz - a.z) * cl,
        a.w + (bw - a.w) * cl,
      ).normalized;
    }
    final halfAngle = math.acos(cosHalf.clamp(-1.0, 1.0));
    final sinHalf = math.sqrt(1.0 - cosHalf * cosHalf);
    if (sinHalf.abs() < 1e-6) {
      return CamQuat(
        a.x + (bx - a.x) * cl,
        a.y + (by - a.y) * cl,
        a.z + (bz - a.z) * cl,
        a.w + (bw - a.w) * cl,
      ).normalized;
    }
    final ratioA = math.sin((1.0 - cl) * halfAngle) / sinHalf;
    final ratioB = math.sin(cl * halfAngle) / sinHalf;
    return CamQuat(
      a.x * ratioA + bx * ratioB,
      a.y * ratioA + by * ratioB,
      a.z * ratioA + bz * ratioB,
      a.w * ratioA + bw * ratioB,
    ).normalized;
  }
}

/// Complete camera lens and transform snapshot blended across virtual cameras.
final class CamPose {
  final CamVec3 position;
  final CamQuat rotation;
  final double fovYDegrees;
  final double near;
  final double far;
  final double dutchRollRadians;

  const CamPose({
    this.position = CamVec3.zero,
    this.rotation = CamQuat.identity,
    this.fovYDegrees = 60.0,
    this.near = 0.1,
    this.far = 200.0,
    this.dutchRollRadians = 0.0,
  });

  /// Vertical field of view converted to radians for render APIs.
  double get fovYRadians => fovYDegrees * (math.pi / 180.0);

  /// World-space rotation including [dutchRollRadians] around the local forward axis.
  CamQuat get composedRotation {
    if (dutchRollRadians.abs() < 1e-8) return rotation;
    return (rotation * CamQuat.fromAxisAngle(CamVec3.forward, dutchRollRadians)).normalized;
  }

  CamPose copyWith({
    CamVec3? position,
    CamQuat? rotation,
    double? fovYDegrees,
    double? near,
    double? far,
    double? dutchRollRadians,
  }) =>
      CamPose(
        position: position ?? this.position,
        rotation: rotation ?? this.rotation,
        fovYDegrees: fovYDegrees ?? this.fovYDegrees,
        near: near ?? this.near,
        far: far ?? this.far,
        dutchRollRadians: dutchRollRadians ?? this.dutchRollRadians,
      );

  static CamPose lerp(CamPose a, CamPose b, double t) {
    final cl = t.clamp(0.0, 1.0);
    return CamPose(
      position: CamVec3.lerp(a.position, b.position, cl),
      rotation: CamQuat.slerp(a.rotation, b.rotation, cl),
      fovYDegrees: a.fovYDegrees + (b.fovYDegrees - a.fovYDegrees) * cl,
      near: a.near + (b.near - a.near) * cl,
      far: a.far + (b.far - a.far) * cl,
      dutchRollRadians: a.dutchRollRadians + (b.dutchRollRadians - a.dutchRollRadians) * cl,
    );
  }
}

/// Frame-rate independent exponential smoothing factor `(1 - exp(-rate * dt))`.
double camDampFactor(double rate, double dt) {
  if (dt <= 0.0) return 0.0;
  if (rate <= 0.0) return 1.0;
  return 1.0 - math.exp(-rate * dt);
}
