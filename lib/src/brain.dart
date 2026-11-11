import 'composer.dart';
import 'math.dart';

/// Represents a virtual camera shot/rig managed by [CameraDirectorBrain].
final class VirtualCameraRig {
  final String id;
  int priority;
  bool enabled;

  /// Base camera pose used directly when neither [orbitRig] nor [framingComposer] overrides it,
  /// or updated incrementally by composers.
  CamPose pose;

  /// Optional screen dead-zone framing composer.
  CameraFramingComposer? framingComposer;

  /// Optional spherical orbit rig.
  CameraOrbitRig? orbitRig;

  /// Optional live world-space target coordinates pushed each frame.
  CamVec3? targetPosition;
  CamVec3? targetVelocity;

  /// Optional aim target (rotates pose toward [lookAtTarget] when [orbitRig] and [framingComposer] are absent).
  CamVec3? lookAtTarget;

  /// Optional transition spec when switching onto this virtual camera.
  CameraTransitionSpec? transitionIn;

  VirtualCameraRig({
    required this.id,
    this.priority = 10,
    this.enabled = true,
    this.pose = const CamPose(),
    this.framingComposer,
    this.orbitRig,
    this.targetPosition,
    this.targetVelocity,
    this.lookAtTarget,
    this.transitionIn,
  });

  /// Evaluates the rig's pose for a timestep [dt].
  CamPose evaluate(double dt, {CameraObstacleProbe? obstacleProbe}) {
    var evaluated = pose;
    final target = targetPosition;
    if (orbitRig != null && target != null) {
      evaluated = orbitRig!.evaluate(evaluated, target, dt, obstacleProbe: obstacleProbe);
    } else if (framingComposer != null && target != null) {
      evaluated = framingComposer!.evaluate(
        evaluated,
        target,
        dt,
        explicitVelocity: targetVelocity,
      );
    } else if (lookAtTarget != null) {
      final dir = lookAtTarget! - evaluated.position;
      if (dir.lengthSquared > 1e-8) {
        evaluated = evaluated.copyWith(rotation: CamQuat.lookRotation(dir));
      }
    }
    pose = evaluated;
    return evaluated;
  }
}

/// Cinemachine-style virtual camera brain managing priorities, smooth blends, and shake impulses.
final class CameraDirectorBrain {
  CameraTransitionSpec defaultTransition;

  final List<VirtualCameraRig> _rigs = [];
  final Map<String, CameraTransitionSpec> _pairTransitions = {};
  final List<CameraImpulseSource> _impulses = [];

  VirtualCameraRig? _activeRig;
  VirtualCameraRig? _previousRig;
  CamPose _blendFromPose = const CamPose();
  CamPose _currentUnshakenPose = const CamPose();
  CamPose _currentPose = const CamPose();

  bool _isBlending = false;
  double _blendElapsed = 0.0;
  CameraTransitionSpec _activeSpec = const CameraTransitionSpec();
  bool _hasEvaluated = false;

  CameraDirectorBrain({
    this.defaultTransition = const CameraTransitionSpec(),
  });

  List<VirtualCameraRig> get rigs => List.unmodifiable(_rigs);
  List<CameraImpulseSource> get impulseSources => List.unmodifiable(_impulses);

  VirtualCameraRig? get activeCamera => _activeRig;
  VirtualCameraRig? get previousCamera => _previousRig;
  bool get isBlending => _isBlending;

  double get blendProgress {
    if (!_isBlending) return 1.0;
    if (_activeSpec.durationSeconds <= 1e-6) return 1.0;
    return (_blendElapsed / _activeSpec.durationSeconds).clamp(0.0, 1.0);
  }

  /// Most recently evaluated camera pose including active [CameraImpulseSource] shake offsets.
  CamPose get currentPose => _currentPose;

  /// Pose prior to additive camera shake impulses.
  CamPose get unshakenPose => _currentUnshakenPose;

  void addRig(VirtualCameraRig rig) {
    if (!_rigs.contains(rig)) {
      _rigs.add(rig);
    }
  }

  bool removeRig(VirtualCameraRig rig) {
    final removed = _rigs.remove(rig);
    if (identical(_activeRig, rig)) {
      _activeRig = null;
    }
    if (identical(_previousRig, rig)) {
      _previousRig = null;
    }
    return removed;
  }

  VirtualCameraRig? findRig(String id) {
    for (final rig in _rigs) {
      if (rig.id == id) return rig;
    }
    return null;
  }

  void setTransition(String fromId, String toId, CameraTransitionSpec spec) {
    _pairTransitions['$fromId->$toId'] = spec;
  }

  void addImpulseSource(CameraImpulseSource source) {
    if (!_impulses.contains(source)) {
      _impulses.add(source);
    }
  }

  CameraImpulseSource ensureDefaultImpulseSource() {
    if (_impulses.isNotEmpty) return _impulses.first;
    final src = CameraImpulseSource();
    _impulses.add(src);
    return src;
  }

  /// Triggers shake trauma through the primary impulse source (creating one if needed).
  void triggerImpulse({
    double traumaAmount = 0.5,
    CamVec3 kick = CamVec3.zero,
  }) {
    ensureDefaultImpulseSource().trigger(traumaAmount: traumaAmount, kick: kick);
  }

  /// Forces the brain to snap immediately to the highest-priority enabled rig without blending.
  void snapToActive() {
    final target = _selectHighestPriorityRig();
    _previousRig = null;
    _activeRig = target;
    _isBlending = false;
    _blendElapsed = 0.0;
    if (target != null) {
      final pose = target.evaluate(0.0);
      _currentUnshakenPose = pose;
      _currentPose = pose;
      _hasEvaluated = true;
    }
  }

  VirtualCameraRig? _selectHighestPriorityRig() {
    VirtualCameraRig? best;
    for (final rig in _rigs) {
      if (!rig.enabled) continue;
      if (best == null || rig.priority > best.priority) {
        best = rig;
      }
    }
    return best;
  }

  CameraTransitionSpec _resolveTransition(VirtualCameraRig? from, VirtualCameraRig to) {
    if (from != null) {
      final pair = _pairTransitions['${from.id}->${to.id}'];
      if (pair != null) return pair;
    }
    return to.transitionIn ?? defaultTransition;
  }

  /// Advances all enabled virtual cameras, updates priority blending, and samples active shake impulses.
  CamPose evaluate(double dt, {CameraObstacleProbe? obstacleProbe}) {
    // Evaluate enabled rigs so composers/orbits track smoothly even when not currently primary.
    for (final rig in _rigs) {
      if (rig.enabled) {
        rig.evaluate(dt, obstacleProbe: obstacleProbe);
      }
    }

    final winner = _selectHighestPriorityRig();
    if (winner == null) {
      return _applyImpulses(_currentUnshakenPose, dt);
    }

    if (!_hasEvaluated) {
      _activeRig = winner;
      _hasEvaluated = true;
      _currentUnshakenPose = winner.pose;
      _currentPose = _applyImpulses(_currentUnshakenPose, dt);
      return _currentPose;
    }

    if (!identical(winner, _activeRig)) {
      final prev = _activeRig;
      final spec = _resolveTransition(prev, winner);
      if (spec.curve == CameraBlendCurve.cut || spec.durationSeconds <= 1e-6) {
        _previousRig = prev;
        _activeRig = winner;
        _isBlending = false;
        _blendElapsed = 0.0;
        _currentUnshakenPose = winner.pose;
      } else {
        // Capture current live blended pose as the new blend origin to guarantee zero pop on interruption.
        _previousRig = prev;
        _activeRig = winner;
        _blendFromPose = _currentUnshakenPose;
        _activeSpec = spec;
        _blendElapsed = 0.0;
        _isBlending = true;
      }
    }

    if (_isBlending && _activeRig != null) {
      _blendElapsed += dt;
      final rawT = _activeSpec.durationSeconds > 1e-6
          ? (_blendElapsed / _activeSpec.durationSeconds).clamp(0.0, 1.0)
          : 1.0;
      final curvedT = _activeSpec.curve.evaluate(rawT);
      _currentUnshakenPose = CamPose.lerp(_blendFromPose, _activeRig!.pose, curvedT);
      if (rawT >= 1.0) {
        _isBlending = false;
        _currentUnshakenPose = _activeRig!.pose;
      }
    } else if (_activeRig != null) {
      _currentUnshakenPose = _activeRig!.pose;
    }

    _currentPose = _applyImpulses(_currentUnshakenPose, dt);
    return _currentPose;
  }

  CamPose _applyImpulses(CamPose base, double dt) {
    if (_impulses.isEmpty) return base;
    var offsetSum = CamVec3.zero;
    var rotTotal = CamQuat.identity;
    for (final src in _impulses) {
      final s = src.sample(dt);
      offsetSum = offsetSum + s.positionOffset;
      rotTotal = (rotTotal * s.rotationOffset).normalized;
    }
    if (offsetSum.lengthSquared < 1e-10 && (1.0 - rotTotal.w.abs()) < 1e-8) {
      return base;
    }
    return base.copyWith(
      position: base.position + offsetSum,
      rotation: (base.rotation * rotTotal).normalized,
    );
  }
}
