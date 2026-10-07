import 'package:flutter/material.dart';

import '../connection/robot_session.dart';
import '../protocol/messages.dart';
import 'joystick.dart';

/// The camera pane with the joystick over it.
///
/// Video itself is Phase 7 (WebRTC: aiortc on the Jetson, flutter_webrtc
/// here). Until then this is the frame the stream will fill, so the layout and
/// the joystick can be used now.
class VideoView extends StatelessWidget {
  final RobotSession session;

  const VideoView({super.key, required this.session});

  @override
  Widget build(BuildContext context) {
    final canDrive = session.connected && session.has(Feature.drive);
    final driving = session.status?.wheelOwner == 'app';
    final String message = session.hello == null
        ? 'Waiting for the robot…'
        : session.has(Feature.video)
        ? 'Live video arrives with WebRTC (Phase 7)'
        : 'Video is not available on this robot yet';
    return ClipRRect(
      borderRadius: BorderRadius.circular(12),
      child: Container(
        color: const Color(0xFF101418),
        child: Stack(
          children: [
            Center(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  const Icon(Icons.videocam_off_outlined, color: Colors.white38, size: 40),
                  const SizedBox(height: 8),
                  Text(message, style: const TextStyle(color: Colors.white54)),
                ],
              ),
            ),
            Positioned(
              left: 12,
              top: 12,
              child: _Badge(
                text: driving
                    ? 'DRIVING · deadman armed'
                    : (canDrive ? 'Hold the pad to drive' : 'Driving unavailable'),
                color: driving ? Colors.green.shade700 : Colors.black,
              ),
            ),
            Positioned(
              right: 16,
              bottom: 16,
              child: LayoutBuilder(
                builder: (context, _) {
                  final h = MediaQuery.sizeOf(context).shortestSide;
                  return Joystick(size: h < 500 ? 130 : 170, enabled: canDrive, onChanged: session.drive);
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _Badge extends StatelessWidget {
  final String text;
  final Color color;

  const _Badge({required this.text, required this.color});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(color: color.withValues(alpha: 0.85), borderRadius: BorderRadius.circular(6)),
      child: Text(
        text,
        style: const TextStyle(color: Colors.white, fontSize: 12, fontWeight: FontWeight.w600),
      ),
    );
  }
}
