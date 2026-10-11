"""Human body keypoints and joint angles, for the yoga assistant.

YOLO pose (via ultralytics, like vision.py) finds each person and 17 COCO
keypoints on them. This module turns those into what a coach talks about: the
angle at each joint, in degrees.

## 2-D and 3-D angles

A joint angle measured in the image is only the true angle when the limb lies
flat to the camera. Seen at a slant, a straight arm still reads straight, but a
bent knee foreshortens and reads straighter or more bent than it is. The Gemini
puts depth on the same pixel grid as colour (gemini.py), so `lift()` gives each
keypoint a position in metres and the angle can be taken in 3-D, where it does
not depend on the viewpoint. Both are kept: the 3-D angle when every point of
the joint has depth, the 2-D one always, so the two can be compared at the bench
(tools/pose_debug.py) before the coaching rules trust either.

Pure geometry plus one model call — no camera, no threads. Callers own the
frames.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

import numpy as np

from . import config

log = logging.getLogger(__name__)

# COCO keypoint order, as YOLO pose emits it.
KEYPOINTS = (
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
)
KP = {name: i for i, name in enumerate(KEYPOINTS)}

# Limbs to draw, as keypoint index pairs.
SKELETON = (
    (KP["left_shoulder"], KP["left_elbow"]), (KP["left_elbow"], KP["left_wrist"]),
    (KP["right_shoulder"], KP["right_elbow"]), (KP["right_elbow"], KP["right_wrist"]),
    (KP["left_shoulder"], KP["right_shoulder"]), (KP["left_hip"], KP["right_hip"]),
    (KP["left_shoulder"], KP["left_hip"]), (KP["right_shoulder"], KP["right_hip"]),
    (KP["left_hip"], KP["left_knee"]), (KP["left_knee"], KP["left_ankle"]),
    (KP["right_hip"], KP["right_knee"]), (KP["right_knee"], KP["right_ankle"]),
    (KP["nose"], KP["left_eye"]), (KP["nose"], KP["right_eye"]),
    (KP["left_eye"], KP["left_ear"]), (KP["right_eye"], KP["right_ear"]),
)

# Joint name -> (a, vertex, c): the angle is measured at the vertex, between the
# rays to a and c. 180 is straight. "Left" is the PERSON's left, as COCO labels
# it — which is the right of the picture when they face the camera.
JOINTS = {
    "left_elbow": ("left_shoulder", "left_elbow", "left_wrist"),
    "right_elbow": ("right_shoulder", "right_elbow", "right_wrist"),
    "left_shoulder": ("left_hip", "left_shoulder", "left_elbow"),
    "right_shoulder": ("right_hip", "right_shoulder", "right_elbow"),
    "left_hip": ("left_shoulder", "left_hip", "left_knee"),
    "right_hip": ("right_shoulder", "right_hip", "right_knee"),
    "left_knee": ("left_hip", "left_knee", "left_ankle"),
    "right_knee": ("right_hip", "right_knee", "right_ankle"),
}

# The torso anchors the depth gate: it is the largest, flattest part of the body,
# so its depth is the most trustworthy estimate of where the person stands.
TORSO = (KP["left_shoulder"], KP["right_shoulder"], KP["left_hip"], KP["right_hip"])

# Half-size (pixels) of the window `lift()` takes a median depth over. One pixel
# is too few: keypoints land on joint centres, but a wrist is a few pixels wide
# at 3 m and a single sample is as likely to be the wall.
_DEPTH_WINDOW = 3


@dataclass
class Person:
    """One detected person.

    `xy` is (17, 2) pixel coordinates and `conf` (17,) per-keypoint confidence,
    both in COCO order. `xyz` is filled by `lift()`: (17, 3) metres in the
    camera frame (x right, y down, z forward), NaN rows where depth is unknown.
    """

    xy: np.ndarray
    conf: np.ndarray
    bbox: tuple[float, float, float, float]  # x1, y1, x2, y2
    score: float
    xyz: np.ndarray | None = None

    @property
    def area(self) -> float:
        x1, y1, x2, y2 = self.bbox
        return (x2 - x1) * (y2 - y1)

    def seen(self, i: int, min_conf: float = config.POSE_KEYPOINT_CONFIDENCE) -> bool:
        return bool(self.conf[i] >= min_conf)

    def distance_m(self) -> float | None:
        """Median depth of the visible torso, or None without depth."""
        if self.xyz is None:
            return None
        z = self.xyz[list(TORSO), 2]
        z = z[np.isfinite(z)]
        return float(np.median(z)) if z.size else None


@dataclass
class JointAngle:
    name: str
    deg_2d: float | None
    deg_3d: float | None = None
    vertex: tuple[float, float] = field(default=(0.0, 0.0))  # pixels, for drawing

    @property
    def deg(self) -> float | None:
        """The best available angle: 3-D when there is depth for it."""
        return self.deg_3d if self.deg_3d is not None else self.deg_2d


def angle_at(a, b, c) -> float | None:
    """Angle at `b` between rays b->a and b->c, in degrees [0, 180].

    Works for 2-D or 3-D points. None if any point is unknown (NaN) or a ray has
    no length — two keypoints on top of each other define no direction.
    """
    a, b, c = (np.asarray(p, dtype=np.float64) for p in (a, b, c))
    if not (np.isfinite(a).all() and np.isfinite(b).all() and np.isfinite(c).all()):
        return None
    u, v = a - b, c - b
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < 1e-6 or nv < 1e-6:
        return None
    cos = float(np.dot(u, v) / (nu * nv))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def joint_angles(
    person: Person, min_conf: float = config.POSE_KEYPOINT_CONFIDENCE
) -> list[JointAngle]:
    """Every joint in JOINTS whose three keypoints were all seen."""
    out = []
    for name, (a, b, c) in JOINTS.items():
        ia, ib, ic = KP[a], KP[b], KP[c]
        if not all(person.seen(i, min_conf) for i in (ia, ib, ic)):
            continue
        deg_3d = None
        if person.xyz is not None:
            deg_3d = angle_at(person.xyz[ia], person.xyz[ib], person.xyz[ic])
        out.append(JointAngle(
            name=name,
            deg_2d=angle_at(person.xy[ia], person.xy[ib], person.xy[ic]),
            deg_3d=deg_3d,
            vertex=(float(person.xy[ib][0]), float(person.xy[ib][1])),
        ))
    return out


def lift(person: Person, depth, intrinsics, gate_m: float = config.POSE_DEPTH_GATE_M) -> None:
    """Fill `person.xyz` from a depth map on the same grid as the keypoints.

    `depth` is float32 metres with NaN for unknown (gemini.Frames.depth);
    `intrinsics` is gemini.Intrinsics. Each keypoint takes the median of the
    valid depth in a small window around it, then is dropped if it sits more
    than `gate_m` from the torso's depth — see POSE_DEPTH_GATE_M.
    """
    h, w = depth.shape[:2]
    z = np.full(len(KEYPOINTS), np.nan)
    for i, (u, v) in enumerate(person.xy):
        if not person.seen(i):
            continue
        u, v = int(round(u)), int(round(v))
        if not (0 <= u < w and 0 <= v < h):
            continue
        patch = depth[max(0, v - _DEPTH_WINDOW):v + _DEPTH_WINDOW + 1,
                      max(0, u - _DEPTH_WINDOW):u + _DEPTH_WINDOW + 1]
        patch = patch[np.isfinite(patch)]
        if patch.size:
            z[i] = float(np.median(patch))

    torso = z[list(TORSO)]
    torso = torso[np.isfinite(torso)]
    if torso.size:
        z[np.abs(z - float(np.median(torso))) > gate_m] = np.nan
    else:
        # No torso depth, no way to tell a limb from the wall behind it.
        z[:] = np.nan

    xyz = np.full((len(KEYPOINTS), 3), np.nan)
    xyz[:, 0] = (person.xy[:, 0] - intrinsics.cx) * z / intrinsics.fx
    xyz[:, 1] = (person.xy[:, 1] - intrinsics.cy) * z / intrinsics.fy
    xyz[:, 2] = z
    person.xyz = xyz


def framing(
    person: Person, width: int, height: int,
    min_conf: float = config.POSE_KEYPOINT_CONFIDENCE,
) -> list[str]:
    """What keeps this person from being fully in shot, in plain words.

    Empty when head to feet are all visible. A body cut off at the frame edge is
    the first thing to fix at the bench: no rule can judge a knee it cannot see.
    """
    problems = []
    if not any(person.seen(KP[k], min_conf) for k in ("nose", "left_eye", "right_eye")):
        problems.append("head not visible")
    if not any(person.seen(KP[k], min_conf) for k in ("left_ankle", "right_ankle")):
        problems.append("feet not visible")
    x1, y1, x2, y2 = person.bbox
    edge = 2  # pixels; a box flush with the border has almost surely been cut
    if y1 <= edge:
        problems.append("cut off at the top")
    if y2 >= height - edge:
        problems.append("cut off at the bottom")
    if x1 <= edge or x2 >= width - edge:
        problems.append("cut off at the side")
    return problems


class PoseEstimator:
    """YOLO pose, loaded lazily on first use. Not thread-safe: one caller."""

    def __init__(
        self,
        model_path=config.POSE_MODEL_PATH,
        confidence: float = config.POSE_CONFIDENCE,
    ) -> None:
        self._model_path = model_path
        self._confidence = confidence
        self._model = None

    def _ensure_model(self) -> None:
        if self._model is None:
            from ultralytics import YOLO

            self._model_path.parent.mkdir(parents=True, exist_ok=True)
            self._model = YOLO(str(self._model_path))

    def estimate(self, frame) -> list[Person]:
        """Every person in a BGR frame, largest (nearest, usually) first."""
        self._ensure_model()
        result = self._model.predict(frame, conf=self._confidence, verbose=False)[0]
        if result.keypoints is None or len(result.boxes) == 0:
            return []
        xy = result.keypoints.xy.cpu().numpy()
        conf = result.keypoints.conf
        # Ultralytics leaves conf None for checkpoints trained without visibility.
        conf = conf.cpu().numpy() if conf is not None else np.ones(xy.shape[:2])
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        people = [
            Person(xy=xy[i], conf=conf[i], bbox=tuple(float(v) for v in boxes[i]),
                   score=float(scores[i]))
            for i in range(len(boxes))
        ]
        people.sort(key=lambda p: p.area, reverse=True)
        return people
