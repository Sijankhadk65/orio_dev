# Orio eye states — expression data

Six looping expressions, one per FSM state: two solid rounded-rect eyes on
black, no pupils, retro-tuned shapes plus smooth eased motion.

| File | State | Loop | Colour | Motion |
|---|---|---|---|---|
| `asleep.json` | ASLEEP | 4.00 s | `#2f7fb5` | flat bars, slow breathing rise/fall |
| `idle.json` | IDLE | 5.00 s | `#4ddbff` | tall blobs, gentle sway + one blink per loop |
| `listening.json` | LISTENING | 3.00 s | `#5cffc0` | taller, level-style pulse |
| `thinking.json` | THINKING | 4.00 s | `#b57bff` | small, slanted, drifting up-left |
| `speaking.json` | SPEAKING | 2.00 s | `#ffd24d` | syllable bounce (squash/stretch) |
| `error.json` | ERROR | 1.33 s | `#ff4d4d` | angry inward slant + jitter |

Canvas: 1024×600, 30 fps.

## Format

Not Lottie — a small custom schema `orio.eyes._Expression` reads directly:

```json
{
  "fps": 30,
  "eyes": {
    "left":  {"color": [77, 219, 255], "radius": 52, "rotation": 0,
              "keyframes": [{"t": 0, "w": 176, "h": 214, "x": 372, "y": 300}, ...]},
    "right": {"color": [77, 219, 255], "radius": 52, "rotation": 0,
              "keyframes": [...]}
  }
}
```

- `color` is 0-255 RGB. `radius` is the rounded-rect corner radius in px.
  `rotation` is a **static** slant in degrees (not animated — see below).
- `keyframes` is a list of `{t, w, h, x, y}` (frame number, width, height,
  center x, center y), sorted by `t`. Every field is linearly interpolated
  between consecutive keyframes at render time.
- The loop period is the **last** keyframe's `t` (not one past it), so the
  first and last keyframes must carry identical `w/h/x/y` — the last frame
  is never actually drawn, it's where the next loop's frame 0 picks up.

This used to be Lottie, rasterized at runtime by `rlottie`. That was dropped
after finding real `rlottie` bugs: keyframed Scale-type properties (a layer's
scale, a shape group's transform, or a shape's own declared size) render
correctly only on their first frame, and a layer's Position breaks the moment
that same layer also carries an animated Path. Since the art is just two
parametric rects, `orio/eyes.py` now interpolates `(w, h, x, y)` itself and
draws with `pygame.draw.rect(..., border_radius=radius)` each frame — no
Lottie renderer involved, and no per-platform native dependency.

Rotation is static per state (not part of the interpolated motion) because
that's simply what these six clips need — `thinking`/`error`/`asleep`/
`speaking` lean the two eyes in mirrored opposite directions for a
slant/angry/sleepy look. If a future expression needs *animated* rotation,
that's a straightforward addition to `_EyeTrack` (interpolate `rotation` the
same way as `w/h/x/y`), just not needed yet.

## Regenerating / tuning

The shapes and timing come from the parametric definitions in `Ocular FSM.dc.html`
(the `STATES` table: width, height, corner radius, slant, offset per state). Change a
number there, and the same table drives this JSON — ask and it will be re-exported.
For a placeholder set generated straight from Python (no external tool), see
`tools/make_placeholder_eyes.py`.
