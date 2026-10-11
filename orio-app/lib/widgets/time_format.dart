String _two(int n) => n.toString().padLeft(2, '0');

/// 14:03:27
String clock(DateTime t) => '${_two(t.hour)}:${_two(t.minute)}:${_two(t.second)}';

/// "0.2 s ago", "3 min ago"
String ago(DateTime t) {
  final d = DateTime.now().difference(t);
  if (d.inSeconds < 60) return '${(d.inMilliseconds / 1000).clamp(0, 59).toStringAsFixed(1)} s ago';
  if (d.inMinutes < 60) return '${d.inMinutes} min ago';
  return clock(t);
}
