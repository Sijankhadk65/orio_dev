import 'dart:convert';

import 'package:flutter/material.dart';

import '../connection/robot_session.dart';
import '../protocol/messages.dart';
import 'status_panel.dart';
import 'time_format.dart';

/// What Orio heard, what it said, and the tools the LLM called in between.
/// With [showEvents], robot events and command results are interleaved by
/// time (the tablet layout has no separate events card).
class ConversationLog extends StatefulWidget {
  final RobotSession session;
  final bool showEvents;

  const ConversationLog({super.key, required this.session, this.showEvents = false});

  @override
  State<ConversationLog> createState() => _ConversationLogState();
}

class _ConversationLogState extends State<ConversationLog> {
  final _scroll = ScrollController();
  bool _follow = true; // stick to the newest line until the user scrolls up

  @override
  void initState() {
    super.initState();
    _scroll.addListener(() {
      if (!_scroll.hasClients) return;
      // The list is reversed, so offset 0 is the newest line.
      final atBottom = _scroll.offset < 40;
      if (atBottom != _follow) setState(() => _follow = atBottom);
    });
  }

  @override
  void dispose() {
    _scroll.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final session = widget.session;
    if (session.hello != null && !session.has(Feature.transcript)) {
      return const NotYet('The conversation log');
    }
    final items = <(DateTime, Widget)>[
      for (final l in session.transcript) (l.time, _TranscriptTile(line: l)),
      if (widget.showEvents)
        for (final e in session.events) (e.time, EventTile(event: e)),
    ]..sort((a, b) => a.$1.compareTo(b.$1));

    if (items.isEmpty) {
      return const Center(
        child: Padding(
          padding: EdgeInsets.all(24),
          child: Text('Nothing said yet. Say "Hey Orio" near the robot.', textAlign: TextAlign.center),
        ),
      );
    }
    return Stack(
      children: [
        ListView.builder(
          controller: _scroll,
          reverse: true,
          padding: const EdgeInsets.all(12),
          itemCount: items.length,
          itemBuilder: (_, i) => items[items.length - 1 - i].$2,
        ),
        if (!_follow)
          Positioned(
            right: 16,
            bottom: 16,
            child: FloatingActionButton.small(
              tooltip: 'Jump to newest',
              onPressed: () =>
                  _scroll.animateTo(0, duration: const Duration(milliseconds: 250), curve: Curves.easeOut),
              child: const Icon(Icons.arrow_downward),
            ),
          ),
      ],
    );
  }
}

class _TranscriptTile extends StatelessWidget {
  final TranscriptLine line;

  const _TranscriptTile({required this.line});

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final scheme = theme.colorScheme;
    final time = Text(clock(line.time), style: theme.textTheme.labelSmall?.copyWith(color: scheme.outline));

    switch (line.kind) {
      case TranscriptKind.wake:
        return Padding(
          padding: const EdgeInsets.symmetric(vertical: 6),
          child: Center(
            child: Chip(
              visualDensity: VisualDensity.compact,
              avatar: const Icon(Icons.hearing, size: 16),
              label: Text('wake: "${line.text}"  ·  ${clock(line.time)}'),
            ),
          ),
        );
      case TranscriptKind.tool:
        final args = line.args.isEmpty ? '' : jsonEncode(line.args);
        return Padding(
          padding: const EdgeInsets.symmetric(vertical: 4, horizontal: 24),
          child: Container(
            padding: const EdgeInsets.all(10),
            decoration: BoxDecoration(
              border: Border.all(color: scheme.outlineVariant),
              borderRadius: BorderRadius.circular(8),
            ),
            child: DefaultTextStyle.merge(
              style: const TextStyle(fontFamily: 'monospace', fontSize: 12),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    children: [
                      Icon(Icons.build_outlined, size: 14, color: scheme.tertiary),
                      const SizedBox(width: 6),
                      Expanded(child: Text('${line.text}($args)')),
                      time,
                    ],
                  ),
                  if (line.result != null) ...[
                    const SizedBox(height: 4),
                    Text('→ ${line.result}', style: TextStyle(color: scheme.onSurfaceVariant)),
                  ],
                ],
              ),
            ),
          ),
        );
      case TranscriptKind.heard:
      case TranscriptKind.said:
        final orio = line.kind == TranscriptKind.said;
        return Align(
          alignment: orio ? Alignment.centerRight : Alignment.centerLeft,
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 420),
            child: Container(
              margin: const EdgeInsets.symmetric(vertical: 4),
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
              decoration: BoxDecoration(
                color: orio ? scheme.primaryContainer : scheme.surfaceContainerHighest,
                borderRadius: BorderRadius.circular(14),
              ),
              child: Column(
                crossAxisAlignment: orio ? CrossAxisAlignment.end : CrossAxisAlignment.start,
                children: [
                  Text(orio ? 'Orio said' : 'Orio heard', style: theme.textTheme.labelSmall),
                  Text(line.text, style: theme.textTheme.bodyLarge),
                  time,
                ],
              ),
            ),
          ),
        );
    }
  }
}
