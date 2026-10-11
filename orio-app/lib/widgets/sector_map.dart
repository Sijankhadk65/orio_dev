import 'dart:math';

import 'package:flutter/material.dart';

import '../connection/robot_session.dart';
import '../protocol/messages.dart';
import 'status_panel.dart';

class SectorMapCard extends StatelessWidget {
  final RobotSession session;
  final bool expand; // fill the parent's height (tablet) instead of a fixed one

  const SectorMapCard({super.key, required this.session, this.expand = false});

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final map = session.status?.sectors;
    Widget body;
    if (session.hello != null && !session.has(Feature.status)) {
      body = const NotYet('The sector map');
    } else if (map == null || map.distancesM.isEmpty) {
      body = const Center(child: Text('No sector map yet'));
    } else {
      body = SectorMapView(map: map);
    }
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Text('Sector map', style: theme.textTheme.titleSmall),
                const SizedBox(width: 12),
                if (map != null) Expanded(child: _Legend(map: map)),
              ],
            ),
            const SizedBox(height: 8),
            if (expand) Expanded(child: body) else SizedBox(height: 180, child: body),
          ],
        ),
      ),
    );
  }
}

class _Legend extends StatelessWidget {
  final SectorMap map;

  const _Legend({required this.map});

  @override
  Widget build(BuildContext context) {
    final style = Theme.of(context).textTheme.labelSmall;
    Widget dot(Color c, String t) => Padding(
      padding: const EdgeInsets.only(left: 10),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 10,
            height: 10,
            decoration: BoxDecoration(color: c, shape: BoxShape.circle),
          ),
          const SizedBox(width: 4),
          Text(t, style: style),
        ],
      ),
    );
    return Wrap(
      alignment: WrapAlignment.end,
      runSpacing: 4,
      children: [
        dot(SectorColors.clear, '>${map.clearM} m'),
        dot(SectorColors.caution, 'near'),
        dot(SectorColors.stop, '<${map.stopM} m'),
        dot(SectorColors.unknown, 'unknown'),
      ],
    );
  }
}

class SectorColors {
  static const clear = Color(0xFF3FA66B);
  static const caution = Color(0xFFE0A030);
  static const stop = Color(0xFFD64545);
  static const unknown = Color(0xFF8A8F98);

  static Color of(double? d, SectorMap map) {
    if (d == null) return unknown;
    if (d < map.stopM) return stop;
    if (d < map.clearM) return caution;
    return clear;
  }
}

/// The forward fan, robot at the bottom centre, sectors left to right, each
/// wedge as long as the nearest thing in it. Unknown sectors are drawn full
/// length in grey: the robot treats unknown as blocked, never as clear.
class SectorMapView extends StatelessWidget {
  final SectorMap map;
  final double rangeM;

  const SectorMapView({super.key, required this.map, this.rangeM = 2.0});

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    return CustomPaint(
      size: Size.infinite,
      painter: _SectorPainter(map, rangeM, scheme, Theme.of(context).textTheme.labelSmall!),
    );
  }
}

class _SectorPainter extends CustomPainter {
  final SectorMap map;
  final double rangeM;
  final ColorScheme scheme;
  final TextStyle labelStyle;

  _SectorPainter(this.map, this.rangeM, this.scheme, this.labelStyle);

  @override
  void paint(Canvas canvas, Size size) {
    final n = map.distancesM.length;
    if (n == 0) return;
    final fov = map.fovDeg * pi / 180;
    final origin = Offset(size.width / 2, size.height - 4);
    // Largest radius whose fan still fits the box.
    final maxR = min(size.height - 8, (size.width / 2 - 4) / sin(fov / 2));
    final pxPerM = maxR / rangeM;
    final start = -pi / 2 - fov / 2; // leftmost edge, measured from +x
    final step = fov / n;

    // Range rings, including the stop and clear thresholds.
    final ring = Paint()
      ..style = PaintingStyle.stroke
      ..color = scheme.outlineVariant;
    for (final m in [0.5, 1.0, 1.5, 2.0]) {
      if (m > rangeM) continue;
      canvas.drawArc(Rect.fromCircle(center: origin, radius: m * pxPerM), start, fov, false, ring);
      _label(
        canvas,
        '${m.toStringAsFixed(1)} m',
        origin + _polar(start + fov, m * pxPerM) + const Offset(4, -6),
      );
    }

    for (var i = 0; i < n; i++) {
      final d = map.distancesM[i];
      final r = ((d ?? rangeM).clamp(0, rangeM)) * pxPerM;
      final a0 = start + i * step + 0.01;
      final sweep = step - 0.02;
      final color = SectorColors.of(d, map);
      final path = Path()
        ..moveTo(origin.dx, origin.dy)
        ..arcTo(Rect.fromCircle(center: origin, radius: r), a0, sweep, false)
        ..close();
      canvas.drawPath(path, Paint()..color = color.withValues(alpha: d == null ? 0.25 : 0.55));
      canvas.drawPath(
        path,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.5
          ..color = color,
      );
      if (d != null && r > 24) {
        _label(canvas, d.toStringAsFixed(2), origin + _polar(a0 + sweep / 2, r - 14), center: true);
      }
    }

    for (final (m, c) in [(map.stopM, SectorColors.stop), (map.clearM, SectorColors.caution)]) {
      canvas.drawArc(
        Rect.fromCircle(center: origin, radius: m * pxPerM),
        start,
        fov,
        false,
        Paint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.5
          ..color = c,
      );
    }

    // The robot.
    canvas.drawCircle(origin, 6, Paint()..color = scheme.primary);
  }

  static Offset _polar(double a, double r) => Offset(cos(a), sin(a)) * r;

  void _label(Canvas canvas, String text, Offset at, {bool center = false}) {
    final tp = TextPainter(
      text: TextSpan(
        text: text,
        style: labelStyle.copyWith(color: scheme.onSurface),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, center ? at - Offset(tp.width / 2, tp.height / 2) : at);
  }

  @override
  bool shouldRepaint(_SectorPainter old) => true; // status arrives at ~5 Hz anyway
}
