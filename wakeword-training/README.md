# wakeword-training

Off-device training for Orio's **"Hey Orio"** wake word. Produces a small
`hey_orio.onnx` that the robot runs under its torch-free onnxruntime path.

## Why this is its own component

openWakeWord's trainer pulls in **PyTorch** and a multi-GB pile of datasets.
None of that belongs on the Jetson — its GPU is reserved for the LLM + object
detector, and the on-device install (`orio-jetson/`) is intentionally torch-free.
So training lives here, runs on a **separate powerful machine**, and only the
exported `.onnx` ships to the robot: **train heavy here, run lean on-device.**

The robot side that *consumes* the model is `orio-jetson/orio/wake_oww.py` (the
`oww` wake engine); see `orio-jetson/docs/wakeword_engine.md`.

## Setup (on the training box)

```bash
cd wakeword-training
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt          # install torch to match your CUDA first
git clone https://github.com/rhasspy/piper-sample-generator   # synthetic positives
```

You also need the negative/background data the trainer mixes in (download once):
background audio (e.g. an ACAV100M / FMA subset), room impulse responses (e.g.
the MIT RIR survey), and openWakeWord's precomputed negative features + a
false-positive validation set (published on Hugging Face). The generated config
has placeholder paths for each — see the openWakeWord "automatic model training"
docs for the exact archives.

## Train

```bash
# 1) scaffold a config, then fill in the dataset paths it lists
python train_orio_wakeword.py --write-config

# 2) generate synthetic clips → augment → train + export
python train_orio_wakeword.py --generate --augment --train
```

Each phase is re-runnable and skips work that's already done. Run
`python train_orio_wakeword.py --help` for all options.

## Deploy to the robot

Copy the exported model onto the Jetson at `orio-jetson/models/hey_orio.onnx`
(committed there, so a `git clone` carries it — no retraining on-device), then:

```bash
cd orio-jetson
ORIO_WAKE_ENGINE=oww ORIO_WAKE_OWW_MODEL=models/hey_orio.onnx uv run main.py
```

Tune `ORIO_WAKE_OWW_THRESHOLD` against the real USB mic.
