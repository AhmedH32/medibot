"""
Skeletal Hand-to-Mouth Medication Intake Verifier with Temporal Occlusion Resilience.
Bridges brief keypoint dropouts (up to 300ms) caused by cup/hand self-occlusion
without granting phantom time credit or merging separate unrelated gestures.
"""

from dataclasses import dataclass
import math
from typing import Optional
from vision.pose_tracker import (
    PersonSkeleton,
    KP_NOSE,
    KP_L_WRIST,
    KP_R_WRIST,
)


@dataclass(frozen=True)
class MedIntakeStatus:
    intake_confirmed: bool
    hand_near_mouth: bool
    normalized_dist: float
    hold_duration_s: float


class MedicationVerifier:
    def __init__(
        self,
        enter_ratio_thresh: float = 0.45,      # Wrist distance <= 45% of shoulder width to enter
        exit_ratio_thresh: float = 0.55,       # Hysteresis exit boundary (55% of shoulder width)
        far_reset_ratio_thresh: float = 0.80,  # Immediate reset if hand visibly moves far away
        grace_dropout_s: float = 0.30,         # Max bridgeable occlusion gap (300 ms)
        required_hold_s: float = 1.20,         # Cumulative active hold required
    ):
        self.enter_ratio_thresh = enter_ratio_thresh
        self.exit_ratio_thresh = exit_ratio_thresh
        self.far_reset_ratio_thresh = far_reset_ratio_thresh
        self.grace_dropout_s = grace_dropout_s
        self.required_hold_s = required_hold_s

        self._accumulated_hold_s: float = 0.0
        self._dropout_elapsed_s: float = 0.0
        self._prev_timestamp: Optional[float] = None
        self._in_active_zone: bool = False
        self._confirmed_latch: bool = False

    def reset(self) -> None:
        self._accumulated_hold_s = 0.0
        self._dropout_elapsed_s = 0.0
        self._prev_timestamp = None
        self._in_active_zone = False
        self._confirmed_latch = False

    def update(self, skeleton: Optional[PersonSkeleton]) -> MedIntakeStatus:
        if skeleton is None:
            return MedIntakeStatus(
                intake_confirmed=self._confirmed_latch,
                hand_near_mouth=False,
                normalized_dist=99.0,
                hold_duration_s=self._accumulated_hold_s,
            )

        now = skeleton.timestamp
        dt = 0.0
        if self._prev_timestamp is not None:
            dt = max(0.0, min(0.5, now - self._prev_timestamp))
        self._prev_timestamp = now

        if not skeleton.has_unilateral_torso or not skeleton.is_valid(KP_NOSE):
            return self._handle_dropout(dt, norm_dist=99.0)

        nose = skeleton.kp(KP_NOSE)
        sh_width = max(0.05, skeleton.shoulder_width)

        dists = []
        if skeleton.is_valid(KP_L_WRIST):
            lw = skeleton.kp(KP_L_WRIST)
            dists.append(math.hypot(lw.x - nose.x, lw.y - nose.y) / sh_width)
        if skeleton.is_valid(KP_R_WRIST):
            rw = skeleton.kp(KP_R_WRIST)
            dists.append(math.hypot(rw.x - nose.x, rw.y - nose.y) / sh_width)

        if not dists:
            # Both wrists occluded by cup/pill bottle -> pause timer inside grace window
            return self._handle_dropout(dt, norm_dist=99.0)

        min_norm_dist = min(dists)

        # If hand is visibly tracked far from the face, reset immediately (prevents merging gestures)
        if min_norm_dist >= self.far_reset_ratio_thresh:
            self._accumulated_hold_s = 0.0
            self._dropout_elapsed_s = 0.0
            self._in_active_zone = False
            return MedIntakeStatus(
                intake_confirmed=self._confirmed_latch,
                hand_near_mouth=False,
                normalized_dist=min_norm_dist,
                hold_duration_s=0.0,
            )

        # Spatial hysteresis evaluation
        thresh = self.exit_ratio_thresh if self._in_active_zone else self.enter_ratio_thresh
        near_mouth = min_norm_dist <= thresh

        if near_mouth:
            self._in_active_zone = True
            self._dropout_elapsed_s = 0.0
            self._accumulated_hold_s += dt
            if self._accumulated_hold_s >= self.required_hold_s:
                self._confirmed_latch = True
            return MedIntakeStatus(
                intake_confirmed=self._confirmed_latch,
                hand_near_mouth=True,
                normalized_dist=min_norm_dist,
                hold_duration_s=self._accumulated_hold_s,
            )

        return self._handle_dropout(dt, norm_dist=min_norm_dist)

    def _handle_dropout(self, dt: float, norm_dist: float) -> MedIntakeStatus:
        """Pauses accumulation during brief occlusions; resets if dropout exceeds grace_dropout_s."""
        if self._accumulated_hold_s > 0.0:
            self._dropout_elapsed_s += dt
            if self._dropout_elapsed_s > self.grace_dropout_s:
                self._accumulated_hold_s = 0.0
                self._dropout_elapsed_s = 0.0
                self._in_active_zone = False

        return MedIntakeStatus(
            intake_confirmed=self._confirmed_latch,
            hand_near_mouth=False,
            normalized_dist=norm_dist,
            hold_duration_s=self._accumulated_hold_s,
        )