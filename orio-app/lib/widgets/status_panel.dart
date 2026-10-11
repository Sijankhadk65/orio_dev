import 'dart:math';

import 'package:flutter/material.dart';

import '../connection/robot_session.dart';
import '../protocol/messages.dart';
import 'time_format.dart';

/// Shown in place of a feature the connected robot does not advertise.
class NotYet extends StatelessWidget {
  final String what;

  const NotYet(this.what, {super.key});

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Padding(
      padding: const EdgeInsets.all(16),
      child: Row(
        children: [
          Icon(Icons.hourglass_empty, color: theme.colorScheme.outline),
          const SizedBox(width: 12),
          Expanded(
            child: Text(
              '$what is not available on this robot yet.',
              style: theme.textTheme.bodyMedium?.copyWith(color: theme.colorScheme.outline),
            ),
          ),
        ],
      ),
    );
  }
}

class StatusPanel extends StatelessWidget {
  final RobotSession session;

  const StatusPanel({super.key, required this.session});

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final status = session.status;
    if (session.hello != null && !session.has(Feature.status)) {
      return const Card(child: NotYet('Status'));
    }
    if (status == null) {
      return const Card(
        child: Padding(
          padding: EdgeInsets.all(24),
          child: Center(child: Text('Waiting for status…')),
        ),
      );
    }
    final sensors = status.sensors.entries.toList()..sort((a, b) => a.key.compareTo(b.key));
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text('Behaviour', style: theme.textTheme.labelMedium),
                      Text(_pretty(status.behaviour), style: theme.textTheme.headlineSmall),
                      if (status.behaviourDetail.isNotEmpty)
                        Text(status.behaviourDetail, style: theme.textTheme.bodyMedium),
                    ],
                  ),
                ),
                if (status.headingDeg != null) Compass(headingDeg: status.headingDeg!),
              ],
            ),
            const SizedBox(height: 12),
            Wrap(
              spacing: 24,
              runSpacing: 8,
              children: [
                _Fact('Voice', status.fsmState),
                _Fact('Wheels', status.wheelOwner ?? 'nobody'),
                if (status.headingDeg != null) _Fact('Heading', '${status.headingDeg!.round() % 360}°'),
                if (status.speedPercent != null)
                  _Fact('Speed', '${status.speedPercent!.toStringAsFixed(0)}%'),
                _Fact('Updated', ago(status.time)),
                if (session.lastLatency != null)
                  _Fact('Cmd RTT', '${session.lastLatency!.inMilliseconds} ms'),
              ],
            ),
            if (sensors.isNotEmpty) ...[
              const SizedBox(height: 12),
              Text('Sensors', style: theme.textTheme.labelMedium),
              const SizedBox(height: 4),
              Wrap(
                spacing: 8,
                runSpacing: 8,
                children: [
                  for (final e in sensors)
                    Tooltip(
                      message: e.value.detail,
                      child: Chip(
                        visualDensity: VisualDensity.compact,
                        avatar: Icon(
                          e.value.ok ? Icons.check_circle : Icons.error,
                          size: 18,
                          color: e.value.ok ? Colors.green : theme.colorScheme.error,
                        ),
                        label: Text(e.key),
                      ),
                    ),
                ],
              ),
            ],
            // Battery is deliberately absent: nothing on the robot measures it yet.
          ],
        ),
      ),
    );
  }

  static String _pretty(String s) {
    final words = s.replaceAll('_', ' ');
    return words.isEmpty ? s : words[0].toUpperCase() + words.substring(1);
  }
}

class _Fact extends StatelessWidget {
  final String label;
  final String value;

  const _Fact(this.label, this.value);

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: theme.textTheme.labelSmall),
        Text(value, style: theme.textTheme.titleSmall),
      ],
    );
  }
}

/// IMU heading as a needle. 0° points up.
class Compass extends StatelessWidget {
  final double headingDeg;

  const Compass({super.key, required this.headingDeg});

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    return SizedBox(width: 56, height: 56, child: CustomPaint(painter: _CompassPainter(headingDeg, scheme)));
  }
}

class _CompassPainter extends CustomPainter {
  final double heading;
  final ColorScheme scheme;

  _CompassPainter(this.heading, this.scheme);

  @override
  void paint(Canvas canvas, Size size) {
    final c = size.center(Offset.zero);
    final r = size.shortestSide / 2 - 2;
    canvas.drawCircle(c, r, Paint()..color = scheme.surfaceContainerHighest);
    canvas.drawCircle(
      c,
      r,
      Paint()
        ..style = PaintingStyle.stroke
        ..color = scheme.outlineVariant,
    );
    final a = heading * pi / 180 - pi / 2;
    final tip = c + Offset(cos(a), sin(a)) * (r - 6);
    canvas.drawLine(
      c,
      tip,
      Paint()
        ..color = scheme.primary
        ..strokeWidth = 3
        ..strokeCap = StrokeCap.round,
    );
    canvas.drawCircle(c, 3, Paint()..color = scheme.primary);
  }

  @override
  bool shouldRepaint(_CompassPainter old) => old.heading != heading || old.scheme != scheme;
}

class EventsCard extends StatelessWidget {
  final RobotSession session;

  const EventsCard({super.key, required this.session});

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final recent = session.events.reversed.take(10).toList();
    return Card(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 8),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
              child: Text('Recent events', style: theme.textTheme.titleSmall),
            ),
            if (recent.isEmpty) const Padding(padding: EdgeInsets.all(16), child: Text('Nothing yet.')),
            for (final e in recent) EventTile(event: e),
          ],
        ),
      ),
    );
  }
}

class EventTile extends StatelessWidget {
  final RobotEvent event;

  const EventTile({super.key, required this.event});

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    final (icon, color) = switch (event.kind) {
      'arrived' => (Icons.flag, Colors.green),
      'halted' || 'blocked' => (Icons.pan_tool, Colors.orange),
      'refused' || 'error' => (Icons.error_outline, scheme.error),
      'result' => (Icons.reply, scheme.primary),
      _ => (Icons.bolt, scheme.tertiary),
    };
    return ListTile(
      dense: true,
      leading: Icon(icon, color: color),
      title: Text(event.text),
      subtitle: Text('${event.kind} · ${clock(event.time)}'),
    );
  }
}
