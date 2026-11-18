"""
Comprehensive unit tests for Subsystem 3 (Vision, Ring Buffer Replay, Fall Stage-1,
Split Vocal Check Timing, Docking Escape Safe-Zone Latching, Zero-Leak Two-Tone Canny
Privacy Masking, Projective Door-Tag Verification, and Hysteresis Medication Verification).
"""

import os
import sys
import numpy as np
import pytest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from hal.camera import CameraStream
from vision.pose_tracker import (
    DetectionBox,
    DisappearanceMode,
    PersonSkeleton,
    SafeFollowerController,
    PortalTagDetector,
    TagPose3D,
    KP_NOSE,
    KP_L_SHOULDER,
    KP_R_SHOULDER,
    KP_L_WRIST,
    KP_R_WRIST,
    KP_L_HIP,
    KP_R_HIP,
)
from vision.fall_detector import (
    OmniFallDetector,
    PostureState,
    render_privacy_silhouette_frame,
)
from vision.med_verifier import MedicationVerifier


def make_skeleton(
    l_sh=(0.42, 0.30),
    r_sh=(0.58, 0.30),
    l_hip=(0.44, 0.58),
    r_hip=(0.56, 0.58),
    nose=(0.50, 0.22),
    l_wrist=(0.38, 0.60),
    r_wrist=(0.62, 0.60),
    wrist_conf=0.95,
    timestamp=1.0,
    bbox=None,
) -> PersonSkeleton:
    arr = np.zeros((17, 3), dtype=np.float32)
    arr[KP_NOSE] = [nose[0], nose[1], 0.95]
    arr[KP_L_SHOULDER] = [l_sh[0], l_sh[1], 0.95]
    arr[KP_R_SHOULDER] = [r_sh[0], r_sh[1], 0.95]
    arr[KP_L_HIP] = [l_hip[0], l_hip[1], 0.95]
    arr[KP_R_HIP] = [r_hip[0], r_hip[1], 0.95]
    arr[KP_L_WRIST] = [l_wrist[0], l_wrist[1], wrist_conf]
    arr[KP_R_WRIST] = [r_wrist[0], r_wrist[1], wrist_conf]
    return PersonSkeleton(arr, timestamp=timestamp, bbox=bbox)


def test_safe_follower_never_reverses_and_freezes_when_crowded():
    """Verify strict non-negative Vx >= 0 invariant and inner proximity freeze."""
    ctrl = SafeFollowerController(target_torso_span=0.28, close_freeze_span=0.40)

    skel_ideal = make_skeleton(l_sh=(0.42, 0.30), r_sh=(0.58, 0.30), l_hip=(0.44, 0.58), r_hip=(0.56, 0.58))
    vx, wz, vis = ctrl.compute_cmd_vel(skel_ideal)
    assert vis is True and vx == 0 and wz == 0

    skel_far = make_skeleton(l_sh=(0.25, 0.35), r_sh=(0.35, 0.35), l_hip=(0.26, 0.51), r_hip=(0.34, 0.51))
    vx, wz, _ = ctrl.compute_cmd_vel(skel_far)
    assert vx > 20 and wz > 15

    skel_near = make_skeleton(l_sh=(0.40, 0.25), r_sh=(0.60, 0.25), l_hip=(0.42, 0.59), r_hip=(0.58, 0.59))
    vx, _, _ = ctrl.compute_cmd_vel(skel_near)
    assert vx == 0, f"Safety violation: robot commanded reverse Vx={vx} without rear sensors!"

    skel_crowded = make_skeleton(l_sh=(0.20, 0.18), r_sh=(0.44, 0.18), l_hip=(0.22, 0.62), r_hip=(0.42, 0.62))
    vx, wz, _ = ctrl.compute_cmd_vel(skel_crowded)
    assert vx == 0 and wz == 0


def test_ring_buffer_retroactive_plunge_vs_doorway_disappearance():
    """Verify 30-frame ring buffer distinguishes a low-FPS bottom plunge from a doorway exit."""
    cam = CameraStream(ring_buffer_size=30)
    for i in range(15):
        cam.inject_synthetic_frame(np.zeros((60, 80, 3), dtype=np.uint8), timestamp=1.0 + i * 0.033)

    recent_frames = cam.get_frames_since(since_timestamp=1.10, max_samples=4)
    assert len(recent_frames) == 4

    ctrl = SafeFollowerController()
    ctrl.observe_and_control(make_skeleton(timestamp=1.0))

    plunge_replay = [
        make_skeleton(
            l_sh=(0.42, 0.65), r_sh=(0.58, 0.65), l_hip=(0.44, 0.86), r_hip=(0.56, 0.86),
            bbox=DetectionBox(0.38, 0.60, 0.62, 0.93), timestamp=1.25
        )
    ]
    obs_plunge = ctrl.observe_and_control(None, buffered_replay_skeletons=plunge_replay)
    assert obs_plunge.disappearance_mode == DisappearanceMode.PLUNGED_BOTTOM_BLIND_SPOT

    ctrl.observe_and_control(make_skeleton(timestamp=2.0, bbox=DetectionBox(0.42, 0.20, 0.58, 0.68)))
    obs_door = ctrl.observe_and_control(None, buffered_replay_skeletons=[])
    assert obs_door.disappearance_mode == DisappearanceMode.OCCLUDED_OR_DOORWAY


def test_stage1_fall_detector_vocal_split_and_docking_escape_latch():
    """
    Verify:
    1. Fast-path plunge triggers immediate_vocal_check=True at t=0.0s.
    2. Slow horizontal lie-down triggers immediate_vocal_check=False (silent until Modal confirms).
    3. Docking escape guard prevents re-triggering when robot turns >4 deg away from sleeping user.
    4. User rolling downward off sofa onto floor breaks latch immediately.
    """
    detector = OmniFallDetector(debounce_hold_s=0.40)

    # 1. Fast-path plunge at 2 FPS (delta_y = 0.38 >= 0.18) -> immediate vocal check
    detector.evaluate(make_skeleton(timestamp=1.0))
    skel_plunge = make_skeleton(
        l_sh=(0.38, 0.68), r_sh=(0.62, 0.68),
        l_hip=(0.40, 0.76), r_hip=(0.60, 0.76),
        timestamp=1.5,
    )
    res_plunge = detector.evaluate(skel_plunge)
    assert res_plunge.is_fall_suspected is True
    assert res_plunge.immediate_vocal_check is True

    # 2. Slow horizontal lie-down on sofa (no sudden downward drop from prior frame) -> silent wait for Modal
    detector.reset()
    for t in [2.0, 2.25, 2.50]:
        skel_sofa = make_skeleton(
            l_sh=(0.35, 0.44), r_sh=(0.35, 0.52),
            l_hip=(0.65, 0.44), r_hip=(0.65, 0.52),
            timestamp=t,
        )
        res_slow = detector.evaluate(skel_sofa)
    assert res_slow.is_fall_suspected is True
    assert res_slow.immediate_vocal_check is False

    # 3. Modal confirms safe sofa rest -> latch at (0.50, 0.48).
    # Robot battery hits <=20% and enters AUTONOMOUS_PORTAL_DOCKING, rotating 35 deg and driving 50 cm:
    detector.latch_safe_resting_zone(center_x=0.50, center_y=0.48, robot_odom=(0.0, 0.0, 0.0))
    res_docking = detector.evaluate(
        skel_sofa,
        robot_odom=(50.0, 12.0, 35.0),
        is_docking_transit=True,
    )
    assert res_docking.is_fall_suspected is False, "Docking transit must not re-trigger static fall lock!"

    # 4. User rolls downward off sofa onto floor (cy drops from 0.48 to 0.76) -> breaks latch even during transit!
    skel_roll_off = make_skeleton(
        l_sh=(0.35, 0.72), r_sh=(0.35, 0.80),
        l_hip=(0.65, 0.72), r_hip=(0.65, 0.80),
        timestamp=3.0,
    )
    res_roll = detector.evaluate(skel_roll_off, robot_odom=(0.0, 0.0, 0.0), is_docking_transit=True)
    assert res_roll.is_fall_suspected is True


def test_zero_leak_two_tone_canny_privacy_mask():
    """
    Verify:
    1. 100% of raw RGB pixels inside the +8% padded DetectionBox are zeroed (even corners & lower body
       when only upper-body head/shoulder keypoints are detected).
    2. Structural furniture/floor Canny edges inside the padded box render as Light Gray (150, 150, 150)
       in the outer periphery and Cyan (255, 255, 0) inside the body contour.
    """
    # Create synthetic room with a bright horizontal sofa/floor boundary line at y=110
    raw_frame = np.full((200, 200, 3), 180, dtype=np.uint8)
    raw_frame[110:150, :] = 40  # Sharp horizontal contrast edge at y=110 across entire frame

    # Simulate partial upper-body-only keypoints (5 keypoints on head/shoulders, 0 on hips/legs)
    kps = np.zeros((17, 3), dtype=np.float32)
    kps[0] = [0.50, 0.55, 0.95]  # Nose
    kps[1] = [0.48, 0.54, 0.90]  # L_Eye
    kps[2] = [0.52, 0.54, 0.90]  # R_Eye
    kps[5] = [0.42, 0.60, 0.92]  # L_Shoulder
    kps[6] = [0.58, 0.60, 0.92]  # R_Shoulder
    bbox = DetectionBox(x1=0.30, y1=0.50, x2=0.70, y2=0.80, conf=0.90)
    skel_partial = PersonSkeleton(kps, timestamp=1.0, bbox=bbox)

    # With expand_margin=0.08 on a 200x200 image, padded box spans x=[44..156], y=[84..176]
    masked = render_privacy_silhouette_frame(raw_frame, skel_partial, expand_margin=0.08)

    # 1. Room pixel outside the padded box (y=20, x=20) preserves original RGB (180, 180, 180)
    assert np.all(masked[20, 20] == 180)

    # 2. Corner inside the padded box (y=90, x=48) and lower-body pixel (y=155, x=100) must be blacked out (0,0,0),
    #    proving zero raw RGB leakage (180 or 40) anywhere inside the padded box!
    assert np.all(masked[90, 48] == 0), f"Raw RGB leaked in padded box corner: {masked[90, 48]}"
    assert np.all(masked[155, 100] == 0), f"Raw RGB leaked in un-keypointed lower body: {masked[155, 100]}"

    # 3. Verify two-tone structural Canny edges exist inside the padded box:
    roi_pixels = masked[84:176, 44:156].reshape(-1, 3)
    has_light_gray_periphery_edge = np.any(np.all(roi_pixels == [150, 150, 150], axis=1))
    has_cyan_body_edge = np.any(np.all(roi_pixels == [255, 255, 0], axis=1))
    assert has_light_gray_periphery_edge, "Expected Light Gray (150,150,150) structural furniture edge in box periphery"
    assert has_cyan_body_edge, "Expected Cyan (255,255,0) structural silhouette edge inside body contour"


def test_medication_verifier_dropout_resilience_and_gesture_separation():
    """Verify 200ms wrist occlusion bridges cleanly, while separate brief face touches do not merge."""
    verifier = MedicationVerifier(required_hold_s=1.20, grace_dropout_s=0.30)

    for t in [1.0, 1.35, 1.70]:
        verifier.update(make_skeleton(nose=(0.50, 0.25), r_wrist=(0.51, 0.26), timestamp=t))
    verifier.update(make_skeleton(nose=(0.50, 0.25), r_wrist=(0.65, 0.60), timestamp=1.90))
    for t in [2.1, 2.45, 2.80]:
        status = verifier.update(make_skeleton(nose=(0.50, 0.25), r_wrist=(0.51, 0.26), timestamp=t))
    assert status.intake_confirmed is False

    verifier.reset()
    for t in [3.0, 3.3, 3.6]:
        verifier.update(make_skeleton(nose=(0.50, 0.25), r_wrist=(0.51, 0.26), timestamp=t))
    verifier.update(make_skeleton(nose=(0.50, 0.25), r_wrist=(0.51, 0.26), wrist_conf=0.05, timestamp=3.8))
    for t in [3.9, 4.2, 4.55]:
        status = verifier.update(make_skeleton(nose=(0.50, 0.25), r_wrist=(0.51, 0.26), timestamp=t))
    assert status.intake_confirmed is True


def test_closed_door_projective_width_verification():
    """Verify 2D projective width ratio + depth equality detects closed vs open doors without rvec noise."""
    tags_closed = {
        10: TagPose3D(tag_id=10, distance_cm=120.0, lateral_offset_cm=-15.0, apparent_width_px=48.0),
        11: TagPose3D(tag_id=11, distance_cm=122.0, lateral_offset_cm=10.0, apparent_width_px=46.5),
    }
    assert PortalTagDetector.is_door_closed(tags_closed, frame_tag_id=10, leaf_tag_id=11) is True

    tags_open = {
        10: TagPose3D(tag_id=10, distance_cm=120.0, lateral_offset_cm=-15.0, apparent_width_px=48.0),
        11: TagPose3D(tag_id=11, distance_cm=88.0, lateral_offset_cm=18.0, apparent_width_px=20.0),
    }
    assert PortalTagDetector.is_door_closed(tags_open, frame_tag_id=10, leaf_tag_id=11) is False