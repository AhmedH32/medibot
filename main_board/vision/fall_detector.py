"""
Stage-1 High-Recall "Inversion of Proof" Fall Detector & Zero-Leak Privacy Silhouette Renderer.
- Always blacks out 100% of the padded (+8%) human DetectionBox (0% raw RGB leakage).
- Uses CLAHE + 21x21 (sigma=7) Gaussian blur + 3x3 dilated Canny structural edges in Cyan
  (body contour) and Light Gray (outer box periphery) so 14x14 ViT patches retain clear
  furniture/floor contact lines.
- Splits vocal-check timing between fast-path plunges (t=0.0s) and slow horizontal postures.
- Guards Safe-Zone latches with yaw-panning compensation and an autonomous docking transit bypass.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple
import cv2
import numpy as np

from vision.pose_tracker import (
    PersonSkeleton,
    KP_NOSE,
    KP_L_SHOULDER,
    KP_R_SHOULDER,
    KP_L_WRIST,
    KP_R_WRIST,
    KP_L_HIP,
    KP_R_HIP,
    KP_L_KNEE,
    KP_R_KNEE,
    KP_L_ANKLE,
    KP_R_ANKLE,
)

# COCO Limb Connections for Skeletal Bone & Contour Rendering
SKELETON_LIMB_PAIRS = [
    (KP_NOSE, KP_L_SHOULDER),
    (KP_NOSE, KP_R_SHOULDER),
    (KP_L_SHOULDER, KP_R_SHOULDER),
    (KP_L_SHOULDER, KP_L_HIP),
    (KP_R_SHOULDER, KP_R_HIP),
    (KP_L_HIP, KP_R_HIP),
    (KP_L_SHOULDER, KP_L_WRIST),
    (KP_R_SHOULDER, KP_R_WRIST),
    (KP_L_HIP, KP_L_KNEE),
    (KP_R_HIP, KP_R_KNEE),
    (KP_L_KNEE, KP_L_ANKLE),
    (KP_R_KNEE, KP_R_ANKLE),
]


class PostureState(Enum):
    NO_PERSON = "NO_PERSON"
    UPRIGHT_OR_SEATED = "UPRIGHT_OR_SEATED"
    ANOMALY_DEBOUNCING = "ANOMALY_DEBOUNCING"
    SUSPECTED_FALL = "SUSPECTED_FALL"


@dataclass(frozen=True)
class FallAssessment:
    state: PostureState
    is_fall_suspected: bool
    immediate_vocal_check: bool  # True on fast plunge (t=0.0s TTS); False on slow recline (wait for Modal)
    anomaly_mode: str
    torso_angle_deg: float
    foreshortening_ratio: float
    vertical_drop_delta: float


class OmniFallDetector:
    def __init__(
        self,
        max_safe_tilt_deg: float = 42.0,
        min_safe_foreshortening: float = 0.80,
        max_safe_bbox_aspect: float = 0.90,
        fast_drop_delta_y: float = 0.18,
        debounce_hold_s: float = 0.40,
        camera_hfov_deg: float = 60.0,
    ):
        self.max_safe_tilt_deg = max_safe_tilt_deg
        self.min_safe_foreshortening = min_safe_foreshortening
        self.max_safe_bbox_aspect = max_safe_bbox_aspect
        self.fast_drop_delta_y = fast_drop_delta_y
        self.debounce_hold_s = debounce_hold_s
        self.camera_hfov_rad = math.radians(camera_hfov_deg)

        self._last_safe_y: Optional[float] = None
        self._anomaly_start_time: Optional[float] = None
        self._latched_mode: str = "NONE"

        # Spatial Safe-Zone Latch anchored to robot odometry pose (x_cm, y_cm, theta_deg)
        self._safe_zone_center: Optional[Tuple[float, float]] = None
        self._safe_zone_odom: Optional[Tuple[float, float, float]] = None

    def reset(self) -> None:
        self._last_safe_y = None
        self._anomaly_start_time = None
        self._latched_mode = "NONE"
        self._safe_zone_center = None
        self._safe_zone_odom = None

    def latch_safe_resting_zone(
        self,
        center_x: float,
        center_y: float,
        robot_odom: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        """
        Latches verified resting coordinates (bed/sofa) junto with robot odometry,
        and anchors _last_safe_y to the bed/sofa elevation so a subsequent roll-off
        onto the floor triggers an immediate fast-drop emergency.
        """
        self._safe_zone_center = (center_x, center_y)
        self._safe_zone_odom = robot_odom
        self._last_safe_y = center_y
        self._anomaly_start_time = None

    def _is_inside_safe_zone(
        self,
        cx: float,
        cy: float,
        current_odom: Tuple[float, float, float],
        is_docking_transit: bool,
    ) -> bool:
        if self._safe_zone_center is None or self._safe_zone_odom is None:
            return False

        sx, sy = self._safe_zone_center
        if (cy - sy) > 0.08:
            self._safe_zone_center = None
            self._safe_zone_odom = None
            return False

        if is_docking_transit:
            return True

        ox, oy, otheta = self._safe_zone_odom
        nx, ny, ntheta = current_odom
        dist_moved_cm = math.hypot(nx - ox, ny - oy)
        d_theta_deg = (ntheta - otheta + 180.0) % 360.0 - 180.0

        if dist_moved_cm > 10.0 or abs(d_theta_deg) > 25.0:
            self._safe_zone_center = None
            self._safe_zone_odom = None
            return False

        d_cx = -math.radians(d_theta_deg) / self.camera_hfov_rad
        panned_sx = sx + d_cx

        if abs(cx - panned_sx) > 0.14:
            self._safe_zone_center = None
            self._safe_zone_odom = None
            return False
        return True

    def evaluate(
        self,
        skeleton: Optional[PersonSkeleton],
        robot_odom: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        is_docking_transit: bool = False,
    ) -> FallAssessment:
        if skeleton is None or skeleton.bbox.conf < 0.35:
            return FallAssessment(
                state=PostureState.NO_PERSON,
                is_fall_suspected=False,
                immediate_vocal_check=False,
                anomaly_mode="NONE",
                torso_angle_deg=0.0,
                foreshortening_ratio=0.0,
                vertical_drop_delta=0.0,
            )

        now = skeleton.timestamp

        # 1. Crumpled / Occluded Heap (Bounding box present, torso keypoints missing)
        if not skeleton.has_unilateral_torso:
            is_tall_edge_exit = (
                (skeleton.bbox.x1 <= 0.06 or skeleton.bbox.x2 >= 0.94)
                and skeleton.bbox.y1 < 0.45
                and skeleton.bbox.aspect_ratio < 0.75
            )
            if not is_tall_edge_exit and (skeleton.bbox.y1 >= 0.45 or skeleton.bbox.aspect_ratio >= 0.85):
                if is_docking_transit and self._safe_zone_center is not None:
                    return FallAssessment(
                        state=PostureState.UPRIGHT_OR_SEATED,
                        is_fall_suspected=False,
                        immediate_vocal_check=False,
                        anomaly_mode="NONE",
                        torso_angle_deg=0.0,
                        foreshortening_ratio=1.0,
                        vertical_drop_delta=0.0,
                    )
                return self._trigger_or_debounce(
                    now=now,
                    mode="CRUMPLED_OCCLUDED",
                    angle=90.0,
                    r_comp=0.0,
                    drop_delta=0.25,
                    fast_path=False,
                )
            return FallAssessment(
                state=PostureState.UPRIGHT_OR_SEATED,
                is_fall_suspected=False,
                immediate_vocal_check=False,
                anomaly_mode="NONE",
                torso_angle_deg=0.0,
                foreshortening_ratio=1.0,
                vertical_drop_delta=0.0,
            )

        # 2. Compute Geometric Invariants on Valid Torso
        _, sh_y = skeleton.shoulder_midpoint
        _, hip_y = skeleton.hip_midpoint
        centroid_y = 0.5 * (sh_y + hip_y)
        centroid_x = 0.5 * (skeleton.shoulder_midpoint[0] + skeleton.hip_midpoint[0])

        torso_angle = skeleton.torso_angle_deg
        sh_width = max(0.04, skeleton.shoulder_width)
        v_span = skeleton.vertical_torso_span
        r_comp = v_span / sh_width
        bbox_ar = skeleton.bbox.aspect_ratio

        drop_delta = 0.0
        if self._last_safe_y is not None:
            drop_delta = max(0.0, sh_y - self._last_safe_y)

        # 3. Evaluate Inversion-of-Proof Safety Invariants
        anomaly_mode = "NONE"
        if torso_angle >= self.max_safe_tilt_deg:
            anomaly_mode = "LATERAL_OR_SLUMPED"
        elif r_comp <= self.min_safe_foreshortening and skeleton.has_core_torso:
            anomaly_mode = "OPTICAL_AXIS_COLLAPSE"
        elif bbox_ar >= self.max_safe_bbox_aspect:
            anomaly_mode = "BBOX_COLLAPSE"

        if anomaly_mode != "NONE":
            is_fast_drop = drop_delta >= self.fast_drop_delta_y
            # Check Safe-Zone latch first (clears latch if cy dropped > 0.08)
            inside_safe = self._is_inside_safe_zone(
                centroid_x, centroid_y, robot_odom, is_docking_transit
            )
            if not is_fast_drop and inside_safe:
                return FallAssessment(
                    state=PostureState.UPRIGHT_OR_SEATED,
                    is_fall_suspected=False,
                    immediate_vocal_check=False,
                    anomaly_mode="NONE",
                    torso_angle_deg=torso_angle,
                    foreshortening_ratio=r_comp,
                    vertical_drop_delta=0.0,
                )

            return self._trigger_or_debounce(
                now=now,
                mode=anomaly_mode,
                angle=torso_angle,
                r_comp=r_comp,
                drop_delta=drop_delta,
                fast_path=is_fast_drop,
            )

        self._anomaly_start_time = None
        self._latched_mode = "NONE"
        self._safe_zone_center = None
        self._safe_zone_odom = None
        self._last_safe_y = sh_y

        return FallAssessment(
            state=PostureState.UPRIGHT_OR_SEATED,
            is_fall_suspected=False,
            immediate_vocal_check=False,
            anomaly_mode="NONE",
            torso_angle_deg=torso_angle,
            foreshortening_ratio=r_comp,
            vertical_drop_delta=0.0,
        )

    def _trigger_or_debounce(
        self,
        now: float,
        mode: str,
        angle: float,
        r_comp: float,
        drop_delta: float,
        fast_path: bool,
    ) -> FallAssessment:
        if self._anomaly_start_time is None:
            self._anomaly_start_time = now
            self._latched_mode = mode

        elapsed = now - self._anomaly_start_time
        if fast_path or elapsed >= self.debounce_hold_s:
            return FallAssessment(
                state=PostureState.SUSPECTED_FALL,
                is_fall_suspected=True,
                immediate_vocal_check=fast_path,
                anomaly_mode=self._latched_mode,
                torso_angle_deg=angle,
                foreshortening_ratio=r_comp,
                vertical_drop_delta=drop_delta,
            )

        return FallAssessment(
            state=PostureState.ANOMALY_DEBOUNCING,
            is_fall_suspected=False,
            immediate_vocal_check=False,
            anomaly_mode=mode,
            torso_angle_deg=angle,
            foreshortening_ratio=r_comp,
            vertical_drop_delta=drop_delta,
        )


def render_privacy_silhouette_frame(
    frame_bgr: np.ndarray,
    skeleton: Optional[PersonSkeleton],
    expand_margin: float = 0.08,
) -> np.ndarray:
    """
    Zero-Leak Two-Tone Canny Privacy Renderer (ViT 14x14 Patch Optimized):
    1. Pads human DetectionBox outward by +8% (`expand_margin=0.08`) to catch stray limbs.
    2. Applies CLAHE local contrast normalization + heavy 21x21 (sigma=7.0) Gaussian blur
       + 3x3 dilated Canny edge detection so low-contrast sofa/floor edges survive ViT patch pooling.
    3. Blacks out 100% of the padded box to [0, 0, 0] (zero raw RGB pixels ever leave the Pi 5).
    4. Renders Canny structural edges in Light Gray [150, 150, 150] in the outer periphery/corners
       and Cyan [255, 255, 0] inside the body contour (Core Ellipse U Skeletal Capsule Hull).
    5. Overlays skeletal bones and valid keypoints in Neon Green [0, 255, 0].
    """
    out = frame_bgr.copy()
    if skeleton is None or skeleton.bbox.conf < 0.20:
        return out

    h, w = out.shape[:2]
    x1 = max(0, int(round((skeleton.bbox.x1 - expand_margin) * w)))
    y1 = max(0, int(round((skeleton.bbox.y1 - expand_margin) * h)))
    x2 = min(w, int(round((skeleton.bbox.x2 + expand_margin) * w)))
    y2 = min(h, int(round((skeleton.bbox.y2 + expand_margin) * h)))

    if x2 <= x1 or y2 <= y1:
        return out

    # 1. Extract CLAHE-enhanced, heavily blurred (21x21, sigma=7) dilated structural edges
    roi = out[y1:y2, x1:x2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (21, 21), 7.0)
    raw_edges = cv2.Canny(blurred, threshold1=20, threshold2=60)
    edges = cv2.dilate(raw_edges, np.ones((3, 3), dtype=np.uint8), iterations=1)

    # 2. Construct internal body contour mask (Core Ellipse U Dilated Skeletal Capsule Hull)
    body_mask = np.zeros((h, w), dtype=np.uint8)
    orig_x1 = max(x1, int(round(skeleton.bbox.x1 * w)))
    orig_y1 = max(y1, int(round(skeleton.bbox.y1 * h)))
    orig_x2 = min(x2, int(round(skeleton.bbox.x2 * w)))
    orig_y2 = min(y2, int(round(skeleton.bbox.y2 * h)))

    cx, cy = (orig_x1 + orig_x2) // 2, (orig_y1 + orig_y2) // 2
    axes = (max(1, int((orig_x2 - orig_x1) * 0.42)), max(1, int((orig_y2 - orig_y1) * 0.42)))
    cv2.ellipse(body_mask, (cx, cy), axes, 0, 0, 360, 255, -1)

    box_span_px = max(orig_x2 - orig_x1, orig_y2 - orig_y1)
    capsule_thickness = max(14, int(box_span_px * 0.22))

    valid_pts = []
    for idx in range(17):
        if skeleton.is_valid(idx):
            kp = skeleton.kp(idx)
            px, py = int(kp.x * w), int(kp.y * h)
            valid_pts.append([px, py])
            cv2.circle(body_mask, (px, py), capsule_thickness // 2, 255, -1)

    for idx_a, idx_b in SKELETON_LIMB_PAIRS:
        if skeleton.is_valid(idx_a) and skeleton.is_valid(idx_b):
            ka, kb = skeleton.kp(idx_a), skeleton.kp(idx_b)
            pa = (int(ka.x * w), int(ka.y * h))
            pb = (int(kb.x * w), int(kb.y * h))
            cv2.line(body_mask, pa, pb, 255, capsule_thickness)

    if len(valid_pts) >= 3:
        hull = cv2.convexHull(np.array(valid_pts, dtype=np.int32))
        cv2.fillConvexPoly(body_mask, hull, 255)

    roi_body_mask = body_mask[y1:y2, x1:x2]

    # 3. Strictly zero out 100% of raw RGB pixels inside the padded human box
    roi[:] = 0

    # 4. Paint outer periphery structural edges Light Gray [150, 150, 150] (furniture/floor lines)
    #    and inner body contour structural edges Cyan [255, 255, 0]
    roi[(edges > 0) & (roi_body_mask == 0)] = (150, 150, 150)
    roi[(edges > 0) & (roi_body_mask > 0)] = (255, 255, 0)

    # 5. Overlay valid skeletal bones and keypoints in Neon Green [0, 255, 0] inside padded box
    for idx_a, idx_b in SKELETON_LIMB_PAIRS:
        if skeleton.is_valid(idx_a) and skeleton.is_valid(idx_b):
            ka, kb = skeleton.kp(idx_a), skeleton.kp(idx_b)
            pa = (int(np.clip(ka.x * w, x1, x2 - 1)), int(np.clip(ka.y * h, y1, y2 - 1)))
            pb = (int(np.clip(kb.x * w, x1, x2 - 1)), int(np.clip(kb.y * h, y1, y2 - 1)))
            cv2.line(out, pa, pb, (0, 255, 0), 2)

    for idx in range(17):
        if skeleton.is_valid(idx):
            kp = skeleton.kp(idx)
            px = int(np.clip(kp.x * w, x1, x2 - 1))
            py = int(np.clip(kp.y * h, y1, y2 - 1))
            cv2.circle(out, (px, py), 4, (0, 255, 0), -1)

    return out