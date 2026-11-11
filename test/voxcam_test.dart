import 'package:test/test.dart';
import 'package:voxcam/voxcam.dart';

void main() {
  group('voxcam Virtual Camera Director', () {
    test('CameraDirectorBrain blends by priority, honors custom curves & handles interruption smoothly', () {
      final brain = CameraDirectorBrain(
        defaultTransition: const CameraTransitionSpec(
          durationSeconds: 1.0,
          curve: CameraBlendCurve.linear,
        ),
      );

      final vcamA = VirtualCameraRig(
        id: 'cam_a',
        priority: 10,
        pose: const CamPose(
          position: CamVec3(0, 2, 10),
          fovYDegrees: 60.0,
        ),
      );
      final vcamB = VirtualCameraRig(
        id: 'cam_b',
        priority: 20,
        pose: const CamPose(
          position: CamVec3(10, 4, 10),
          fovYDegrees: 40.0,
          dutchRollRadians: 0.2,
        ),
      );
      final vcamC = VirtualCameraRig(
        id: 'cam_c',
        priority: 5,
        pose: const CamPose(
          position: CamVec3(0, 10, 0),
          fovYDegrees: 80.0,
        ),
      );

      brain.addRig(vcamA);
      brain.addRig(vcamC);

      // Initial evaluation starts directly on highest priority (cam_a).
      var pose = brain.evaluate(0.0);
      expect(brain.activeCamera?.id, 'cam_a');
      expect(brain.isBlending, isFalse);
      expect(pose.position.x, closeTo(0.0, 1e-6));

      // Add cam_b with higher priority -> starts 1s blend A -> B.
      brain.addRig(vcamB);
      pose = brain.evaluate(0.5);
      expect(brain.activeCamera?.id, 'cam_b');
      expect(brain.isBlending, isTrue);
      expect(brain.blendProgress, closeTo(0.5, 1e-6));
      expect(pose.position.x, closeTo(5.0, 1e-5));
      expect(pose.fovYDegrees, closeTo(50.0, 1e-5));
      expect(pose.dutchRollRadians, closeTo(0.1, 1e-5));

      // Mid-blend interruption: boost cam_c to priority 30 with custom cut transition.
      brain.setTransition('cam_b', 'cam_c', const CameraTransitionSpec(
        durationSeconds: 1.0,
        curve: CameraBlendCurve.smoothstep,
      ));
      vcamC.priority = 30;
      pose = brain.evaluate(0.0);
      // Immediately capturing live pose ensures zero jump at interruption start (x remains 5.0)!
      expect(brain.activeCamera?.id, 'cam_c');
      expect(pose.position.x, closeTo(5.0, 1e-5));

      // Complete blend -> arrives cleanly at cam_c.
      pose = brain.evaluate(1.0);
      expect(brain.isBlending, isFalse);
      expect(pose.position.y, closeTo(10.0, 1e-5));
      expect(pose.fovYDegrees, closeTo(80.0, 1e-5));

      // Cut transition pair snaps instantly.
      brain.setTransition('cam_c', 'cam_a', const CameraTransitionSpec.cut());
      vcamA.priority = 50;
      pose = brain.evaluate(0.0);
      expect(brain.isBlending, isFalse);
      expect(pose.position.x, closeTo(0.0, 1e-6));
    });

    test('CameraFramingComposer keeps jitter inside dead-zone steady and tracks beyond threshold', () {
      final composer = CameraFramingComposer(
        deadZoneHalfWidth: 0.25,
        deadZoneHalfHeight: 0.25,
        softZoneHalfWidth: 0.5,
        softZoneHalfHeight: 0.5,
        damping: 0.0, // instant when outside dead-zone for crisp assertion
        lookAheadTime: 0.0,
        trackedTargetOffset: CamVec3.zero,
        aimAtTarget: false,
      );

      var pose = const CamPose(
        position: CamVec3(0, 0, 10),
        rotation: CamQuat.identity,
        fovYDegrees: 60.0,
      );

      // Small lateral target displacement inside dead-zone -> camera must remain stationary.
      pose = composer.evaluate(pose, const CamVec3(0.4, 0.2, 0), 0.1);
      expect(pose.position.x, closeTo(0.0, 1e-6));
      expect(pose.position.y, closeTo(0.0, 1e-6));

      // Large displacement well outside dead-zone & soft-zone -> camera shifts right to track target.
      pose = composer.evaluate(pose, const CamVec3(8.0, 0.0, 0), 0.1);
      expect(pose.position.x, greaterThan(2.0));
    });

    test('CameraOrbitRig respects pitch/distance bounds and pulls inward on obstacles', () {
      final orbit = CameraOrbitRig(
        yawRadians: 0.0,
        pitchRadians: 0.0,
        distance: 10.0,
        targetOffset: CamVec3.zero,
        wallPullInPadding: 0.5,
      );
      orbit.rotateBy(0.0, 5.0);
      expect(orbit.pitchRadians, closeTo(orbit.maxPitchRadians, 1e-6));

      orbit.pitchRadians = 0.0;
      final base = const CamPose();

      // Raycast probe reports a wall 4.0m behind target -> immediate pull-in to 3.5m.
      final occludedPose = orbit.evaluate(
        base,
        CamVec3.zero,
        0.016,
        obstacleProbe: (_, __, ___) => 4.0,
      );
      expect(orbit.effectiveDistance, closeTo(3.5, 1e-5));
      expect(occludedPose.position.z, closeTo(3.5, 1e-5));

      // Once obstacle clears, camera pulls smoothly outward toward 10.0m without popping.
      final recoveringPose = orbit.evaluate(base, CamVec3.zero, 0.05);
      expect(recoveringPose.position.z, greaterThan(3.5));
      expect(recoveringPose.position.z, lessThan(10.0));
    });

    test('CameraImpulseSource applies trauma shake and decays deterministically', () {
      final impulse = CameraImpulseSource(decayRate: 1.0);
      impulse.triggerAt(
        const CamVec3(0, 0, 0),
        const CamVec3(5, 0, 0),
        traumaAmount: 1.0,
        maxRadius: 10.0,
        kickStrength: 0.5,
      );
      // Linear falloff 0.5 -> quadratic trauma = 0.25
      expect(impulse.trauma, closeTo(0.25, 1e-6));

      final sample1 = impulse.sample(0.02);
      expect(sample1.positionOffset.length, greaterThan(0.0));

      // After sufficient time, trauma and directional kick decay cleanly to zero.
      for (int i = 0; i < 40; i++) {
        impulse.sample(0.1);
      }
      final settled = impulse.sample(0.1);
      expect(impulse.isActive, isFalse);
      expect(settled.positionOffset.length, closeTo(0.0, 1e-7));
    });
  });
}
