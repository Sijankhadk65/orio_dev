import 'package:flutter/material.dart';

import '../connection/robot_session.dart';
import '../protocol/messages.dart';
import 'joystick.dart';

/// The camera pane with the joystick over it.
///
/// Frames are JPEGs over the robot WebSocket (docs/app-protocol.md, `video`).
/// The pane asks for them only while [active] — the robot encodes on its CPU,
/// so a hidden pane must not cost it anything.
class VideoView extends StatefulWidget {
  final RobotSession session;
  final bool active;

  const VideoView({super.key, required this.session, this.active = true});

  @override
  State<VideoView> createState() => _VideoViewState();
}

class _VideoViewState extends State<VideoView> {
  static const staleAfter = Duration(seconds: 2);
  RobotSession? _watching; // the session this pane holds a video claim on

  void _sync() {
    final want = widget.active ? widget.session : null;
    if (want == _watching) return;
    _watching?.unwatchVideo();
    want?.watchVideo();
    _watching = want;
  }

  @override
  void initState() {
    super.initState();
    _sync();
  }

  @override
  void didUpdateWidget(VideoView old) {
    super.didUpdateWidget(old);
    _sync();
  }

  @override
  void dispose() {
    _watching?.unwatchVideo();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final session = widget.session;
    final canDrive = session.connected && session.has(Feature.drive);
    final driving = session.status?.wheelOwner == 'app';
    final last = session.lastFrameAt;
    final stale = last == null || DateTime.now().difference(last) > staleAfter;
    final String? message = session.hello == null
        ? 'Waiting for the robot…'
        : !session.has(Feature.video)
        ? 'Video is not available on this robot yet'
        : last == null
        ? 'Waiting for the first frame…'
        : null;
    return ClipRRect(
      borderRadius: BorderRadius.circular(12),
      child: Container(
        color: const Color(0xFF101418),
        child: Stack(
          children: [
            Positioned.fill(
              child: ValueListenableBuilder<VideoFrame?>(
                valueListenable: session.video,
                builder: (context, frame, _) => frame == null
                    ? const SizedBox.shrink()
                    : Image.memory(
                        frame.jpeg,
                        fit: BoxFit.contain,
                        gaplessPlayback: true, // keep the old frame up while the next decodes
                        filterQuality: FilterQuality.medium,
                      ),
              ),
            ),
            if (message != null)
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
            if (last != null)
              Positioned(
                right: 12,
                top: 12,
                child: stale
                    ? _Badge(
                        text: 'FROZEN · no frame for ${DateTime.now().difference(last).inSeconds} s',
                        color: Colors.red.shade700,
                      )
                    : ValueListenableBuilder<VideoFrame?>(
                        valueListenable: session.video,
                        builder: (context, frame, _) =>
                            _Badge(text: _rate(session.videoFps, frame), color: Colors.black),
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

  /// "10 fps · 85 ms". The lag compares the robot's clock with this device's,
  /// so it is only shown when it is plausible (both clocks roughly in sync).
  static String _rate(double fps, VideoFrame? frame) {
    final text = '${fps.toStringAsFixed(0)} fps';
    if (frame == null) return text;
    final lag = DateTime.now().difference(frame.captured).inMilliseconds;
    return lag >= 0 && lag < 5000 ? '$text · $lag ms' : text;
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
