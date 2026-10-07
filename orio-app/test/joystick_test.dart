import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:orio_app/widgets/joystick.dart';

void main() {
  Future<List<Offset>> dragFromCentre(WidgetTester tester, Offset by) async {
    final sent = <Offset>[];
    await tester.pumpWidget(
      MaterialApp(
        home: Center(child: Joystick(size: 200, onChanged: (x, y) => sent.add(Offset(x, y)))),
      ),
    );
    final centre = tester.getCenter(find.byType(Joystick));
    final gesture = await tester.startGesture(centre + const Offset(1, 1));
    await gesture.moveBy(by - const Offset(1, 1));
    await tester.pump(const Duration(milliseconds: 350)); // a few 10 Hz ticks while held
    await gesture.up();
    await tester.pump();
    return sent;
  }

  testWidgets('a touch near the centre sends zero, not a direction', (tester) async {
    final sent = await dragFromCentre(tester, const Offset(5, 5)); // 5 % of the 100 px radius
    expect(sent, isNotEmpty);
    expect(sent.every((o) => o == Offset.zero), isTrue);
  });

  testWidgets('pushing up sends forward while held, then one zero on release', (tester) async {
    final sent = await dragFromCentre(tester, const Offset(0, -80));
    expect(sent.length, greaterThanOrEqualTo(3), reason: 'repeats while held, for the deadman');
    expect(sent[sent.length - 2].dy, greaterThan(0.5)); // y forward is up the screen
    expect(sent.last, Offset.zero);
  });
}
