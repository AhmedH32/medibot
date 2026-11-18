"""
17-Keypoint COCO Skeleton, Bounding Box, ArUco/AprilTag 3D Pose Estimator,
and Stateless Non-Reversing Visual Servoing Controller.
"""

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Dict, List, Optional, Tuple
import cv2
import numpy as np


# COCO 17 Keypoint Indices
KP_NOSE = 0
KP_L_SHOULDER = 5
KP_R_SHOULDER = 6
KP_L_WRIST = 9
KP_R_WRIST = 10
KP_L_HIP = 11
KP_R_HIP = 12
KP_L_KNEE = 13
KP_R_KNEE = 14
KP_L_ANKLE = 15
KP_R_ANKLE = 16


class DisappearanceMode(Enum):
    VISIBLE = "VISIBLE"
    PLUNGED_BOTTOM_BLIND_SPOT = "PLUNGED_BOTTOM_BLIND_SPOT"
    OCCLUDED_OR_DOORWAY = "OCCLUDED_OR_DOORWAY"


@dataclass(frozen=True)
class Keypoint:
    x: float     # Normalized [0.0, 1.0] across frame width
    y: float     # Normalized [0.0, 1.0] across frame height
    conf: float  # Detection confidence [0.0, 1.0]


@dataclass(frozen=True)
class DetectionBox:
    """Normalized 2D human bounding box [0.0, 1.0] from YOLO."""
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float = 0.90

    @property
    def width(self) -> float:
        return max(1e-4, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(1e-4, self.y2 - self.y1)

    @property
    def aspect_ratio(self) -> float:
        return self.width / self.height

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)


class PersonSkeleton:
    """
    Wraps a 17x3 normalized keypoint array and optional 2D DetectionBox.
    Supports both bilateral (full) and unilateral (side-profile/doorway) torso geometry.
    """
    def __init__(
        self,
        keypoints_norm: np.ndarray,
        timestamp: float,
        bbox: Optional[DetectionBox] = None,
        min_conf: float = 0.35,
    ):
        if keypoints_norm.shape != (17, 3):
            raise ValueError(f"Expected (17, 3) keypoint array, got {keypoints_norm.shape}")
        self.raw = keypoints_norm
        self.timestamp = timestamp
        self.min_conf = min_conf
        self.bbox = bbox or self._infer_bbox_from_keypoints()

    def _infer_bbox_from_keypoints(self) -> DetectionBox:
        valid_mask = self.raw[:, 2] >= self.min_conf
        if not np.any(valid_mask):
            return DetectionBox(0.0, 0.0, 0.0, 0.0, conf=0.0)
        pts = self.raw[valid_mask]
        x1 = float(np.clip(np.min(pts[:, 0]) - 0.04, 0.0, 1.0))
        y1 = float(np.clip(np.min(pts[:, 1]) - 0.04, 0.0, 1.0))
        x2 = float(np.clip(np.max(pts[:, 0]) + 0.04, 0.0, 1.0))
        y2 = float(np.clip(np.max(pts[:, 1]) + 0.04, 0.0, 1.0))
        mean_conf = float(np.mean(pts[:, 2]))
        return DetectionBox(x1, y1, x2, y2, conf=mean_conf)

    def kp(self, idx: int) -> Keypoint:
        row = self.raw[idx]
        return Keypoint(x=float(row[0]), y=float(row[1]), conf=float(row[2]))

    def is_valid(self, idx: int) -> bool:
        return float(self.raw[idx, 2]) >= self.min_conf

    @property
    def valid_keypoint_count(self) -> int:
        return int(np.sum(self.raw[:, 2] >= self.min_conf))

    @property
    def has_core_torso(self) -> bool:
        return (
            self.is_valid(KP_L_SHOULDER)
            and self.is_valid(KP_R_SHOULDER)
            and self.is_valid(KP_L_HIP)
            and self.is_valid(KP_R_HIP)
        )

    @property
    def has_unilateral_torso(self) -> bool:
        left_ok = self.is_valid(KP_L_SHOULDER) and self.is_valid(KP_L_HIP)
        right_ok = self.is_valid(KP_R_SHOULDER) and self.is_valid(KP_R_HIP)
        return left_ok or right_ok

    @property
    def shoulder_midpoint(self) -> Tuple[float, float]:
        if self.is_valid(KP_L_SHOULDER) and self.is_valid(KP_R_SHOULDER):
            l_sh, r_sh = self.kp(KP_L_SHOULDER), self.kp(KP_R_SHOULDER)
            return ((l_sh.x + r_sh.x) * 0.5, (l_sh.y + r_sh.y) * 0.5)
        if self.is_valid(KP_L_SHOULDER):
            l_sh = self.kp(KP_L_SHOULDER)
            return (l_sh.x, l_sh.y)
        if self.is_valid(KP_R_SHOULDER):
            r_sh = self.kp(KP_R_SHOULDER)
            return (r_sh.x, r_sh.y)
        return self.bbox.center

    @property
    def hip_midpoint(self) -> Tuple[float, float]:
        if self.is_valid(KP_L_HIP) and self.is_valid(KP_R_HIP):
            l_hp, r_hp = self.kp(KP_L_HIP), self.kp(KP_R_HIP)
            return ((l_hp.x + r_hp.x) * 0.5, (l_hp.y + r_hp.y) * 0.5)
        if self.is_valid(KP_L_HIP):
            l_hp = self.kp(KP_L_HIP)
            return (l_hp.x, l_hp.y)
        if self.is_valid(KP_R_HIP):
            r_hp = self.kp(KP_R_HIP)
            return (r_hp.x, r_hp.y)
        return (self.bbox.center[0], self.bbox.y2)

    @property
    def shoulder_width(self) -> float:
        if self.is_valid(KP_L_SHOULDER) and self.is_valid(KP_R_SHOULDER):
            l_sh, r_sh = self.kp(KP_L_SHOULDER), self.kp(KP_R_SHOULDER)
            return math.hypot(l_sh.x - r_sh.x, l_sh.y - r_sh.y)
        return self.bbox.width * 0.80

    @property
    def torso_length(self) -> float:
        sx, sy = self.shoulder_midpoint
        hx, hy = self.hip_midpoint
        return math.hypot(hx - sx, hy - sy)

    @property
    def vertical_torso_span(self) -> float:
        _, sy = self.shoulder_midpoint
        _, hy = self.hip_midpoint
        return abs(hy - sy)

    @property
    def torso_angle_deg(self) -> float:
        sx, sy = self.shoulder_midpoint
        hx, hy = self.hip_midpoint
        dx = abs(sx - hx)
        dy = max(1e-5, abs(hy - sy))
        return math.degrees(math.atan2(dx, dy))


@dataclass(frozen=True)
class TargetObservation:
    vx_cmd: int
    wz_cmd: int
    target_visible: bool
    estimated_distance_cm: float
    last_seen_x: float
    disappearance_mode: DisappearanceMode


class SafeFollowerController:
    def __init__(
        self,
        target_torso_span: float = 0.28,
        close_freeze_span: float = 0.40,
        dist_deadband: float = 0.025,
        yaw_deadband: float = 0.04,
        kp_linear: float = 220.0,
        kp_angular: float = 110.0,
        max_forward_cm_s: int = 60,
        max_yaw_deg_s: int = 60,
    ):
        self.target_torso_span = target_torso_span
        self.close_freeze_span = close_freeze_span
        self.dist_deadband = dist_deadband
        self.yaw_deadband = yaw_deadband
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular
        self.max_forward_cm_s = max_forward_cm_s
        self.max_yaw_deg_s = max_yaw_deg_s

        self.last_distance_cm: float = 150.0
        self.last_seen_x: float = 0.50
        self.last_bbox_y2: float = 0.65

    def compute_cmd_vel(
        self,
        skeleton: Optional[PersonSkeleton],
        now: Optional[float] = None,
        buffered_replay_skeletons: Optional[List[PersonSkeleton]] = None,
    ) -> Tuple[int, int, bool]:
        obs = self.observe_and_control(skeleton, buffered_replay_skeletons)
        return obs.vx_cmd, obs.wz_cmd, obs.target_visible

    def observe_and_control(
        self,
        skeleton: Optional[PersonSkeleton],
        buffered_replay_skeletons: Optional[List[PersonSkeleton]] = None,
    ) -> TargetObservation:
        if skeleton is None or not skeleton.has_unilateral_torso:
            if buffered_replay_skeletons:
                for replay_skel in buffered_replay_skeletons:
                    if replay_skel.bbox.y2 >= 0.88 or replay_skel.hip_midpoint[1] >= 0.82:
                        self.last_bbox_y2 = max(self.last_bbox_y2, replay_skel.bbox.y2)

            mode = (
                DisappearanceMode.PLUNGED_BOTTOM_BLIND_SPOT
                if self.last_bbox_y2 >= 0.88
                else DisappearanceMode.OCCLUDED_OR_DOORWAY
            )
            return TargetObservation(
                vx_cmd=0,
                wz_cmd=0,
                target_visible=False,
                estimated_distance_cm=self.last_distance_cm,
                last_seen_x=self.last_seen_x,
                disappearance_mode=mode,
            )

        measured_span = max(0.05, skeleton.torso_length)
        self.last_distance_cm = 150.0 * (self.target_torso_span / measured_span)

        hx, _ = skeleton.hip_midpoint
        sx, _ = skeleton.shoulder_midpoint
        person_center_x = 0.5 * (hx + sx)
        self.last_seen_x = person_center_x
        self.last_bbox_y2 = skeleton.bbox.y2

        if measured_span >= self.close_freeze_span:
            return TargetObservation(
                vx_cmd=0,
                wz_cmd=0,
                target_visible=True,
                estimated_distance_cm=self.last_distance_cm,
                last_seen_x=person_center_x,
                disappearance_mode=DisappearanceMode.VISIBLE,
            )

        yaw_error = 0.50 - person_center_x
        if abs(yaw_error) <= self.yaw_deadband:
            wz = 0
        else:
            raw_wz = self.kp_angular * yaw_error
            wz = int(max(-self.max_yaw_deg_s, min(self.max_yaw_deg_s, round(raw_wz))))

        dist_error = self.target_torso_span - measured_span
        if dist_error <= self.dist_deadband:
            vx = 0
        else:
            raw_vx = self.kp_linear * dist_error
            vx = int(max(0, min(self.max_forward_cm_s, round(raw_vx))))

        return TargetObservation(
            vx_cmd=vx,
            wz_cmd=wz,
            target_visible=True,
            estimated_distance_cm=self.last_distance_cm,
            last_seen_x=person_center_x,
            disappearance_mode=DisappearanceMode.VISIBLE,
        )


@dataclass(frozen=True)
class TagPose3D:
    tag_id: int
    distance_cm: float
    lateral_offset_cm: float
    apparent_width_px: float  # 2D horizontal pixel span (immune to solvePnP rvec normal flipping)


class PortalTagDetector:
    """
    Detects ArUco/AprilTag markers (4x4_50 dictionary) for doorway navigation,
    charging dock alignment, and closed-door verification using 2D projective width
    invariants + Z-depth equality (immune to planar homography rvec flipping).
    """
    def __init__(self, tag_size_cm: float = 10.0):
        self.tag_size_cm = tag_size_cm
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

    def detect_tags(self, frame_bgr: np.ndarray) -> Dict[int, TagPose3D]:
        h, w = frame_bgr.shape[:2]
        corners_list, ids, _ = self._detector.detectMarkers(frame_bgr)
        results: Dict[int, TagPose3D] = {}
        if ids is None:
            return results

        focal_px = float(w) * 0.85
        half_s = self.tag_size_cm * 0.5
        obj_pts = np.array([
            [-half_s,  half_s, 0.0],
            [ half_s,  half_s, 0.0],
            [ half_s, -half_s, 0.0],
            [-half_s, -half_s, 0.0],
        ], dtype=np.float32)
        cam_matrix = np.array([
            [focal_px, 0.0, w * 0.5],
            [0.0, focal_px, h * 0.5],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        for corners, tag_id_arr in zip(corners_list, ids):
            tag_id = int(tag_id_arr[0])
            img_pts = corners[0].astype(np.float32)
            # Horizontal pixel span across top and bottom edges of the marker
            top_w = float(np.linalg.norm(img_pts[1] - img_pts[0]))
            bot_w = float(np.linalg.norm(img_pts[2] - img_pts[3]))
            apparent_w_px = 0.5 * (top_w + bot_w)

            ok, _, tvec = cv2.solvePnP(obj_pts, img_pts, cam_matrix, dist_coeffs)
            if not ok:
                continue
            tx, _, tz = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
            results[tag_id] = TagPose3D(
                tag_id=tag_id,
                distance_cm=max(1.0, tz),
                lateral_offset_cm=tx,
                apparent_width_px=max(1.0, apparent_w_px),
            )
        return results

    @staticmethod
    def is_door_closed(detected_tags: Dict[int, TagPose3D], frame_tag_id: int, leaf_tag_id: int) -> bool:
        """
        Uses 2D projective foreshortening (W_leaf / W_frame ~= cos(theta_door)) and Z-depth equality.
        When a door swings open, the leaf tag either foreshortens horizontally or shifts in depth.
        """
        if frame_tag_id not in detected_tags or leaf_tag_id not in detected_tags:
            return False
        t_frame = detected_tags[frame_tag_id]
        t_leaf = detected_tags[leaf_tag_id]

        width_ratio = t_leaf.apparent_width_px / max(1.0, t_frame.apparent_width_px)
        depth_diff_cm = abs(t_frame.distance_cm - t_leaf.distance_cm)
        return (0.75 <= width_ratio <= 1.25) and (depth_diff_cm <= 25.0)