# Wake-word engine — openWakeWord ("Hey Orio")

Orio's voice loop is wake-gated: it sleeps until it hears its name, then takes
commands. There are now **two interchangeable wake engines** behind one seam
(`waker.await_wake()` in `orio/conversation.py`):

| `ORIO_WAKE_ENGINE` | Module | How it works |
|---|---|---|
| `whisper` *(default)* | `orio/wake.py` | Reuses the Whisper ASR — transcribes every dormant phrase and string-matches it. Zero extra model, but **spotty** on the short "Hey Orio" and burns CPU transcribing ambient speech. |
| `oww` | `orio/wake_oww.py` | A dedicated **openWakeWord** hotword model — streams raw mic frames through a small CPU/ONNX spotter. Built for short hotwords, far more reliable, and runs **no Whisper while asleep**. |

The `oww` engine is the fix for the spotty activation. Whisper still does all the
real ASR once Orio is awake; openWakeWord only owns the `ASLEEP → LISTENING` gate.

## Why openWakeWord

- **CPU / onnxruntime, torch-free** — no PyTorch, stays off the GPU reserved for
  the LLM + object detector, matching the rest of the stack.
- Purpose-built for always-on, low-false-negative hotword spotting on short
  phrases — exactly the failure mode Whisper-match had.
- Custom keywords can be **trained from TTS-synthesized samples** (no recorded
  data needed), and it ships pretrained models useful for smoke-testing.

## Enabling it

```bash
# Deps are already in pyproject; install them:
uv sync

# Smoke-test the pipeline with a built-in keyword (says "Hey Jarvis"):
ORIO_WAKE_ENGINE=oww ORIO_WAKE_OWW_MODEL=hey_jarvis uv run main.py

# Production: point at the trained Orio model:
ORIO_WAKE_ENGINE=oww ORIO_WAKE_OWW_MODEL=models/hey_orio.onnx uv run main.py
```

First run downloads openWakeWord's shared feature models (melspectrogram +
embedding, ~few MB) and any built-in keyword to its package cache — needs
network once, offline thereafter.

### Config knobs (all in `orio/config.py`, env-overridable)

| Env var | Default | Meaning |
|---|---|---|
| `ORIO_WAKE_ENGINE` | `whisper` | `oww` to use openWakeWord, `whisper` for the matcher. |
| `ORIO_WAKE_OWW_MODEL` | `hey_jarvis` | Path to a trained `.onnx`/`.tflite`, or a built-in keyword name. |
| `ORIO_WAKE_OWW_THRESHOLD` | `0.5` | Score in [0,1] to fire on. Raise to cut false wakes, lower if it misses. |
| `ORIO_WAKE_OWW_FRAMEWORK` | `onnx` | `onnx` (torch-free) or `tflite`. |

`ORIO_WAKE=0` still disables gating entirely (always-on), independent of engine.

## Training the "Orio" model — off-device

The pretrained models don't include "Orio", so until a custom model is trained,
`oww` can only smoke-test with a built-in keyword (e.g. `hey_jarvis`).

Training is done **on a separate powerful machine, not the Jetson** —
openWakeWord's trainer pulls in **PyTorch**, which we deliberately keep off the
robot (the GPU is reserved for the LLM + object detector). The robot only ever
*runs* the exported model, under the torch-free onnxruntime path. Train heavy
off-device, run lean on-device.

Training lives in its own monorepo component, **`wakeword-training/`** (kept
separate so torch never touches this package) — see its `README.md` for the full
env + dataset setup. On the training box:

```bash
cd ../wakeword-training
# 1) scaffold a config, then fill in dataset paths
python train_orio_wakeword.py --write-config

# 2) generate synthetic clips → augment → train + export
python train_orio_wakeword.py --generate --augment --train
```

Then **copy the exported `hey_orio.onnx` into `orio-jetson/models/`** (it's
committed, so a `git clone` onto the Jetson carries it — no retraining
on-device) and point the engine at it:

```bash
ORIO_WAKE_ENGINE=oww ORIO_WAKE_OWW_MODEL=models/hey_orio.onnx uv run main.py
```

Finally tune `ORIO_WAKE_OWW_THRESHOLD` against the real USB mic — far-field
pickup is the likely weak point; revisit mic hardware if accuracy still lags.
