import 'package:flutter/material.dart';

import '../connection/robot_session.dart';
import '../widgets/command_bar.dart';
import '../widgets/conversation_log.dart';
import '../widgets/link_banner.dart';
import '../widgets/sector_map.dart';
import '../widgets/status_panel.dart';
import '../widgets/video_view.dart';

/// Width at which the screens stop being tabs and sit side by side as panes.
const tabletBreakpoint = 840.0;

class HomeScreen extends StatefulWidget {
  final RobotSession session;

  const HomeScreen({super.key, required this.session});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  int _tab = 0;

  @override
  Widget build(BuildContext context) {
    final session = widget.session;
    return ListenableBuilder(
      listenable: session,
      builder: (context, _) {
        final wide = MediaQuery.sizeOf(context).width >= tabletBreakpoint;
        return Scaffold(
          appBar: AppBar(
            title: Text(session.hello?.robot ?? session.profile.name),
            actions: [
              Padding(
                padding: const EdgeInsets.only(right: 8),
                child: LinkChip(session: session),
              ),
            ],
          ),
          body: Column(
            children: [
              LinkBanner(session: session),
              Expanded(child: wide ? _tablet(session) : _phone(session)),
            ],
          ),
          bottomNavigationBar: wide
              ? null
              : NavigationBar(
                  selectedIndex: _tab,
                  onDestinationSelected: (i) => setState(() => _tab = i),
                  destinations: const [
                    NavigationDestination(icon: Icon(Icons.dashboard_outlined), label: 'Status'),
                    NavigationDestination(icon: Icon(Icons.forum_outlined), label: 'Talk'),
                    NavigationDestination(icon: Icon(Icons.gamepad_outlined), label: 'Drive'),
                  ],
                ),
        );
      },
    );
  }

  Widget _phone(RobotSession session) {
    return IndexedStack(
      index: _tab,
      children: [
        ListView(
          padding: const EdgeInsets.all(12),
          children: [
            StatusPanel(session: session),
            const SizedBox(height: 12),
            SectorMapCard(session: session),
            const SizedBox(height: 12),
            EventsCard(session: session),
          ],
        ),
        ConversationLog(session: session),
        Column(
          children: [
            Expanded(
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: VideoView(session: session),
              ),
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(12, 0, 12, 12),
              child: CommandBar(session: session),
            ),
          ],
        ),
      ],
    );
  }

  Widget _tablet(RobotSession session) {
    return Row(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Expanded(
          flex: 3,
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Expanded(flex: 3, child: VideoView(session: session)),
                const SizedBox(height: 12),
                Expanded(flex: 2, child: SectorMapCard(session: session, expand: true)),
                const SizedBox(height: 12),
                CommandBar(session: session),
              ],
            ),
          ),
        ),
        const VerticalDivider(width: 1),
        Expanded(
          flex: 2,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Padding(
                padding: const EdgeInsets.all(12),
                child: StatusPanel(session: session),
              ),
              const Divider(height: 1),
              Expanded(child: ConversationLog(session: session, showEvents: true)),
            ],
          ),
        ),
      ],
    );
  }
}
