# orio-dev

Monorepo for **Orio**, a wheel-based mobile robot. The Jetson software and the
STM32 firmware live together in one repository so a single commit/tag captures a
matching, known-good version of both halves — no drift between the high-level
brain and the real-time controller.

## Layout

| Path | What it is | Stack |
|---|---|---|
| `orio-jetson/` | High-level brain: voice → LLM → speech operator layer (perception/planning land here too). | Python · uv |
| `orio-stm32/` | Real-time controller firmware: motor/servo control loops + safety. _(not added yet)_ | C/C++ · bare-metal/RTOS |

See each subproject's own `README.md` for setup and run instructions.

## Architecture (two-tier split)

```
            ┌─────────────── orio-jetson (Linux, not real-time) ───────────────┐
  person ─► mic → ASR → LLM → TTS → speaker        deterministic mediator       │
            │         (semantic intents only)      validate / bound / e-stop ───┼─┐
            └──────────────────────────────────────────────────────────────────┘ │
                                                                                  │ framed
                                                          serial (UART/USB/CAN)   │ binary
            ┌─────────────── orio-stm32 (bare-metal / RTOS) ──────────────────┐   │ protocol
            │  1 kHz real-time control loops + safety  ◄──────────────────────┼───┘
            │  → FSESC drive motors, arm/neck servos, sensors                 │
            └────────────────────────────────────────────────────────────────┘
```

- **Jetson** decides *what* to do and emits semantic intents (`drive_to`,
  `move_arm_to`, `stop`, `get_status`). Linux — **not** real-time.
- **STM32** turns intent into deterministic actuation and enforces safety
  (heartbeat watchdog, bounds). The LLM is never in the control loop.
- The two talk over a hand-rolled framed binary protocol on a serial link —
  **no ROS**, so the whole thing ships as a single deployable device.

## Why a monorepo

The Jetson↔STM32 command protocol is a shared contract. Keeping both sides in
one repo means a protocol change is one atomic commit across firmware and
software, and any checkout/tag gives you a Jetson build and an STM32 build that
were tested together.

## Working in a subproject

```bash
cd orio-jetson && uv run main.py        # the Jetson operator layer
```

Git is rooted at `orio-dev/`; commit from anywhere in the tree. Each subproject
keeps its own `.gitignore` for its build artifacts (e.g. `orio-jetson/.venv/`,
`orio-jetson/voices/`).
