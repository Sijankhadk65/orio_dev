import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// A thumb pad. While held it reports (x, y) in -1..1 every [period] (x right,
/// y forward); on release it reports (0, 0) once. That steady stream is what
/// the robot's deadman watches: if it stops, the wheels stop.
class Joystick extends StatefulWidget {
  final void Function(double x, double y) onChanged;
  final bool enabled;
  final double size;
  final Duration period;

  const Joystick({
    super.key,
    required this.onChanged,
    this.enabled = true,
    this.size = 160,
    this.period = const Duration(milliseconds: 100), // ~10 Hz, per the plan
  });

  @override
  State<Joystick> createState() => _JoystickState();
}

class _JoystickState extends State<Joystick> {
  Offset _knob = Offset.zero; // -1..1 each axis, y down (screen)
  Timer? _timer;

  double get _radius => widget.size / 2;

  void _move(Offset local) {
    var v = (local - Offset(_radius, _radius)) / _radius;
    if (v.distance > 1) v = v / v.distance;
    setState(() => _knob = v);
  }

  void _start(Offset local) {
    if (!widget.enabled) return;
    HapticFeedback.selectionClick();
    _move(local);
    _emit();
    _timer?.cancel();
    _timer = Timer.periodic(widget.period, (_) => _emit());
  }

  /// Positions this close to the centre are sent as (0, 0). Without it the
  /// first touch, a pixel off centre, reads as a direction — often backward,
  /// which on the robot is a blind reverse.
  static const deadZone = 0.15;

  void _emit() {
    final k = _knob.distance < deadZone ? Offset.zero : _knob;
    widget.onChanged(k.dx, -k.dy);
  }

  void _end() {
    if (_timer == null) return;
    _timer?.cancel();
    _timer = null;
    setState(() => _knob = Offset.zero);
    widget.onChanged(0, 0);
  }

  @override
  void didUpdateWidget(Joystick old) {
    super.didUpdateWidget(old);
    if (!widget.enabled) _end();
  }

  @override
  void dispose() {
    _timer?.cancel();
    if (_timer != null) widget.onChanged(0, 0);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    final knobR = widget.size * 0.2;
    return Opacity(
      opacity: widget.enabled ? 1 : 0.4,
      child: GestureDetector(
        onPanStart: (d) => _start(d.localPosition),
        onPanUpdate: (d) {
          if (_timer != null) _move(d.localPosition);
        },
        onPanEnd: (_) => _end(),
        onPanCancel: _end,
        child: SizedBox.square(
          dimension: widget.size,
          child: CustomPaint(painter: _PadPainter(_knob, knobR, scheme, active: _timer != null)),
        ),
      ),
    );
  }
}

class _PadPainter extends CustomPainter {
  final Offset knob;
  final double knobR;
  final ColorScheme scheme;
  final bool active;

  _PadPainter(this.knob, this.knobR, this.scheme, {required this.active});

  @override
  void paint(Canvas canvas, Size size) {
    final c = size.center(Offset.zero);
    final r = size.width / 2;
    canvas.drawCircle(c, r, Paint()..color = Colors.black.withValues(alpha: 0.35));
    canvas.drawCircle(
      c,
      r - 1,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2
        ..color = Colors.white.withValues(alpha: 0.6),
    );
    final cross = Paint()..color = Colors.white.withValues(alpha: 0.25);
    canvas.drawLine(c - Offset(r, 0), c + Offset(r, 0), cross);
    canvas.drawLine(c - Offset(0, r), c + Offset(0, r), cross);
    final travel = r - knobR;
    final at = c + knob * travel;
    canvas.drawCircle(
      at,
      knobR,
      Paint()..color = active ? scheme.primary : Colors.white.withValues(alpha: 0.8),
    );
    if (active) {
      final mag = min(1.0, knob.distance);
      canvas.drawCircle(
        at,
        knobR + 4 * mag,
        Paint()
          ..style = PaintingStyle.stroke
          ..color = scheme.primary.withValues(alpha: 0.5),
      );
    }
  }

  @override
  bool shouldRepaint(_PadPainter old) => old.knob != knob || old.active != active;
}
