import 'package:flutter/material.dart';

import '../connection/robot_session.dart';
import '../protocol/messages.dart';

/// Stop, stay, follow me, go to. The same intents the voice assistant has —
/// the app only asks; the robot's own guards decide what happens.
class CommandBar extends StatelessWidget {
  final RobotSession session;

  const CommandBar({super.key, required this.session});

  // What the detector can recognise that is worth walking to.
  static const goToTargets = ['person', 'chair', 'couch', 'tv', 'dining table', 'potted plant', 'cup'];

  void _send(BuildContext context, String name, {String? target}) {
    if (!session.command(name, target: target)) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('Not connected — command not sent')));
    }
  }

  Future<void> _goTo(BuildContext context) async {
    final target = await showDialog<String>(
      context: context,
      builder: (context) => _GoToDialog(targets: goToTargets),
    );
    if (target != null && target.isNotEmpty && context.mounted) {
      _send(context, 'go_to', target: target);
    }
  }

  @override
  Widget build(BuildContext context) {
    final enabled = session.connected && session.has(Feature.commands);
    final reason = !session.connected
        ? 'Not connected'
        : session.has(Feature.commands)
        ? null
        : 'Commands are not available on this robot yet';
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        _LastOutcome(session: session),
        Tooltip(
          message: reason ?? '',
          child: Row(
            children: [
              Expanded(
                flex: 2,
                child: FilledButton.icon(
                  style: FilledButton.styleFrom(
                    backgroundColor: const Color(0xFFD32F2F), // the same red in light and dark
                    foregroundColor: Colors.white,
                    minimumSize: const Size.fromHeight(56),
                    textStyle: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  onPressed: enabled ? () => _send(context, 'stop') : null,
                  icon: const Icon(Icons.stop_circle_outlined),
                  label: const Text('STOP'),
                ),
              ),
              const SizedBox(width: 8),
              _Small(
                icon: Icons.front_hand_outlined,
                label: 'Stay',
                onPressed: enabled ? () => _send(context, 'stay') : null,
              ),
              const SizedBox(width: 8),
              _Small(
                icon: Icons.directions_walk,
                label: 'Follow',
                onPressed: enabled ? () => _send(context, 'follow_me') : null,
              ),
              const SizedBox(width: 8),
              _Small(
                icon: Icons.place_outlined,
                label: 'Go to',
                onPressed: enabled ? () => _goTo(context) : null,
              ),
            ],
          ),
        ),
      ],
    );
  }
}

/// The newest command result or robot event, for a few seconds, right above
/// the buttons — on a phone the events list is on another tab.
class _LastOutcome extends StatelessWidget {
  final RobotSession session;

  const _LastOutcome({required this.session});

  static const showFor = Duration(seconds: 8);

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    final last = session.events.isEmpty ? null : session.events.last;
    final fresh = last != null && DateTime.now().difference(last.time) < showFor;
    final bad = last != null && (last.kind == 'refused' || last.kind == 'error' || last.kind == 'halted');
    return SizedBox(
      height: 28, // reserved, so the buttons don't jump when a line appears
      child: AnimatedOpacity(
        opacity: fresh ? 1 : 0,
        duration: const Duration(milliseconds: 200),
        child: last == null
            ? const SizedBox.shrink()
            : Row(
                children: [
                  Icon(
                    bad ? Icons.error_outline : Icons.check_circle_outline,
                    size: 16,
                    color: bad ? scheme.error : Colors.green,
                  ),
                  const SizedBox(width: 6),
                  Expanded(
                    child: Text(
                      last.text,
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: Theme.of(context).textTheme.bodySmall,
                    ),
                  ),
                ],
              ),
      ),
    );
  }
}

class _Small extends StatelessWidget {
  final IconData icon;
  final String label;
  final VoidCallback? onPressed;

  const _Small({required this.icon, required this.label, required this.onPressed});

  @override
  Widget build(BuildContext context) {
    return Expanded(
      child: FilledButton.tonal(
        style: FilledButton.styleFrom(
          minimumSize: const Size.fromHeight(56),
          padding: const EdgeInsets.symmetric(horizontal: 4),
        ),
        onPressed: onPressed,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 20),
            Text(label, style: const TextStyle(fontSize: 12)),
          ],
        ),
      ),
    );
  }
}

class _GoToDialog extends StatefulWidget {
  final List<String> targets;

  const _GoToDialog({required this.targets});

  @override
  State<_GoToDialog> createState() => _GoToDialogState();
}

class _GoToDialogState extends State<_GoToDialog> {
  final _text = TextEditingController();

  @override
  void dispose() {
    _text.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('Go to…'),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Wrap(
            spacing: 8,
            runSpacing: 8,
            children: [
              for (final t in widget.targets)
                ActionChip(label: Text(t), onPressed: () => Navigator.pop(context, t)),
            ],
          ),
          const SizedBox(height: 16),
          TextField(
            controller: _text,
            decoration: const InputDecoration(labelText: 'Or type an object', border: OutlineInputBorder()),
            onSubmitted: (v) => Navigator.pop(context, v.trim()),
          ),
        ],
      ),
      actions: [
        TextButton(onPressed: () => Navigator.pop(context), child: const Text('Cancel')),
        FilledButton(onPressed: () => Navigator.pop(context, _text.text.trim()), child: const Text('Go')),
      ],
    );
  }
}
