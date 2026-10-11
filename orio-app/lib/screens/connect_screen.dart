import 'package:flutter/material.dart';

import '../connection/profiles.dart';
import '../connection/robot_session.dart';
import 'home_screen.dart';

/// Pick where the robot is (demo, laptop mock, robot on Wi-Fi or Tailscale),
/// edit its address and token, and connect.
class ConnectScreen extends StatefulWidget {
  const ConnectScreen({super.key});

  @override
  State<ConnectScreen> createState() => _ConnectScreenState();
}

class _ConnectScreenState extends State<ConnectScreen> {
  final _store = ProfileStore();
  final _url = TextEditingController();
  final _token = TextEditingController();
  List<ServerProfile> _profiles = List.of(defaultProfiles);
  int _selected = 0;
  String? _urlError;
  bool _hideToken = true;

  @override
  void initState() {
    super.initState();
    _store.load().then((loaded) {
      if (!mounted) return;
      setState(() {
        _profiles = loaded.$1;
        _selected = loaded.$2;
      });
      _fill();
    });
    _fill();
  }

  void _fill() {
    final p = _profiles[_selected];
    _url.text = p.url;
    _token.text = p.token;
    _urlError = null;
  }

  ServerProfile get _edited =>
      _profiles[_selected].copyWith(url: _url.text.trim(), token: _token.text.trim());

  Future<void> _connect() async {
    final profile = _edited;
    final problem = profile.validate();
    if (problem != null) {
      setState(() => _urlError = problem);
      return;
    }
    _profiles[_selected] = profile;
    await _store.save(_profiles, _selected);
    if (!mounted) return;
    final session = RobotSession(profile)..connect();
    await Navigator.of(context).push(MaterialPageRoute(builder: (_) => HomeScreen(session: session)));
    await session.disconnect();
    session.dispose();
  }

  void _resetDefaults() {
    setState(() {
      _profiles = List.of(defaultProfiles);
      _selected = 0;
      _fill();
    });
    _store.save(_profiles, _selected);
  }

  @override
  void dispose() {
    _url.dispose();
    _token.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final demo = _profiles[_selected].demo;
    return Scaffold(
      appBar: AppBar(
        title: const Text('Connect to Orio'),
        actions: [
          IconButton(
            tooltip: 'Reset addresses',
            icon: const Icon(Icons.restart_alt),
            onPressed: _resetDefaults,
          ),
        ],
      ),
      body: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 520),
          child: ListView(
            padding: const EdgeInsets.all(16),
            children: [
              Text('Where is the robot?', style: theme.textTheme.titleMedium),
              const SizedBox(height: 8),
              RadioGroup<int>(
                groupValue: _selected,
                onChanged: (i) => setState(() {
                  _selected = i!;
                  _fill();
                }),
                child: Card(
                  clipBehavior: Clip.antiAlias,
                  child: Column(
                    children: [
                      for (var i = 0; i < _profiles.length; i++)
                        RadioListTile<int>(
                          value: i,
                          title: Text(_profiles[i].name),
                          subtitle: Text(_profiles[i].demo ? 'Fake robot inside the app' : _profiles[i].url),
                        ),
                    ],
                  ),
                ),
              ),
              const SizedBox(height: 16),
              if (!demo) ...[
                TextField(
                  controller: _url,
                  keyboardType: TextInputType.url,
                  autocorrect: false,
                  decoration: InputDecoration(
                    labelText: 'Server address',
                    hintText: 'ws://host:8765/ws',
                    errorText: _urlError,
                    border: const OutlineInputBorder(),
                  ),
                ),
                const SizedBox(height: 12),
                TextField(
                  controller: _token,
                  obscureText: _hideToken,
                  autocorrect: false,
                  decoration: InputDecoration(
                    labelText: 'Token',
                    helperText: 'Sent in hello; the robot refuses a wrong one',
                    border: const OutlineInputBorder(),
                    suffixIcon: IconButton(
                      icon: Icon(_hideToken ? Icons.visibility : Icons.visibility_off),
                      onPressed: () => setState(() => _hideToken = !_hideToken),
                    ),
                  ),
                ),
                const SizedBox(height: 16),
              ],
              FilledButton.icon(
                onPressed: _connect,
                icon: const Icon(Icons.link),
                label: const Text('Connect'),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
