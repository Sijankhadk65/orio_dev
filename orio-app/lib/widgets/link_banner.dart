import 'package:flutter/material.dart';

import '../connection/robot_session.dart';

(String, IconData, Color) linkLook(LinkState s, ColorScheme c) => switch (s) {
  LinkState.connecting => ('Connecting', Icons.sync, c.tertiary),
  LinkState.online => ('Online', Icons.wifi, Colors.green),
  LinkState.stale => ('No status', Icons.wifi_off, Colors.orange),
  LinkState.lost => ('Lost', Icons.cloud_off, c.error),
  LinkState.incompatible => ('Wrong version', Icons.block, c.error),
  LinkState.closed => ('Disconnected', Icons.link_off, c.outline),
};

class LinkChip extends StatelessWidget {
  final RobotSession session;

  const LinkChip({super.key, required this.session});

  @override
  Widget build(BuildContext context) {
    final (label, icon, color) = linkLook(session.state, Theme.of(context).colorScheme);
    return Chip(
      avatar: Icon(icon, size: 18, color: color),
      label: Text(label),
      visualDensity: VisualDensity.compact,
    );
  }
}

/// A strip under the app bar whenever the link is anything but healthy, so a
/// stale picture never passes for a live one.
class LinkBanner extends StatelessWidget {
  final RobotSession session;

  const LinkBanner({super.key, required this.session});

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    final String? text = switch (session.state) {
      LinkState.online || LinkState.closed => null,
      LinkState.connecting => 'Connecting to ${session.profile.name}…',
      LinkState.stale =>
        'Connected, but no status for over '
            '${RobotSession.staleAfter.inMilliseconds / 1000} s. What you see may be out of date.',
      LinkState.lost =>
        'Connection lost (${session.error ?? 'unknown'}). Retrying… '
            'Orio keeps running on its own.',
      LinkState.incompatible => session.error ?? 'Protocol version mismatch.',
    };
    if (text == null) return const SizedBox.shrink();
    final bad = session.state == LinkState.lost || session.state == LinkState.incompatible;
    return Material(
      color: bad ? scheme.errorContainer : scheme.secondaryContainer,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(
          children: [
            Expanded(
              child: Text(
                text,
                style: TextStyle(color: bad ? scheme.onErrorContainer : scheme.onSecondaryContainer),
              ),
            ),
            if (session.state == LinkState.lost || session.state == LinkState.incompatible)
              TextButton(onPressed: session.connect, child: const Text('Retry now')),
          ],
        ),
      ),
    );
  }
}
