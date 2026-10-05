#!/usr/bin/env python3
"""Detect RoboCup balls, rings, and selected A/B/C/D letter blocks."""

from __future__ import annotations

import argparse
import concurrent.futures
from collections import deque
try:
    import fcntl
except ImportError:  # Keeps protocol-only tests runnable on Windows.
    fcntl = None
import getpass
import http.server
import json
import math
import multiprocessing
import os
import re
import select
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path

try:
    import termios
except ImportError:  # Keeps detector and protocol tests runnable on Windows.
    termios = None

import cv2
import numpy as np
from abcd_detector.detector import ABCDDetector, LETTERS
from .arm_kinematics import (
    BASE_YAW_CENTER_TICK,
    BASE_YAW_HOME_TICK,
    GRIPPER_CLOSED_TICK,
    GRIPPER_OPEN_TICK,
    HOME_ID1_TICK,
    HOME_ID2_TICK,
    ID1_SAFE_LIMITS,
    ID2_SAFE_LIMITS,
    JOINT_NAMES,
    MIN_ANGLE_GAP_DEG,
    READY_ID1_TICK,
    READY_ID2_TICK,
    angle_gap_degrees,
    enforce_angle_gap,
    gripper_position_mm,
    id1_degrees,
    id2_degrees,
    id2_tick_from_degrees,
    joint_positions,
)
from .grasp_calibration import (
    GRASP_MAX_DISTANCE_CM,
    GRASP_MIN_DISTANCE_CM,
    calibrated_grasp_ticks,
)
from .chassis_link import ChassisArmLink
from .platform_task import (
    CENTER_DEADBAND_PX as PLATFORM_CENTER_DEADBAND_PX,
    CENTER_ID2_RANGE as PLATFORM_CENTER_ID2_RANGE,
    CENTER_ID2_STEP_TICKS as PLATFORM_CENTER_ID2_STEP_TICKS,
    CENTER_ID6_RANGE as PLATFORM_CENTER_ID6_RANGE,
    CENTER_ID6_STEP_TICKS as PLATFORM_CENTER_ID6_STEP_TICKS,
    PLATFORM_CENTER_TIME_MS,
    HIGH as PLATFORM_HIGH_POSE,
    PLATFORM_ARM_TIME_MS,
    PLATFORM_RETREAT_TIME_MS,
    PLATFORM_RETURN_HIGH_TIME_MS,
    PLATFORM_GRIPPER_CLOSED,
    PLATFORM_GRIPPER_OPEN,
    PLATFORM_GRIPPER_CLOSE_TIME_MS,
    PLATFORM_GRIPPER_TIME_MS,
    PLATFORM_LETTER_PLACE_TIME_MS,
    PLATFORM_PLACE_GRIPPER_TIME_MS,
    PLATFORM_LETTER_SUCCESS_QUOTA,
    PLATFORM_RING_AXIS_TIME_MS,
    PLATFORM_RING_PLACE_TIME_MS,
    HTD85_AUX_HIGH,
    PLATFORM_AUX_TIME_MS,
    PlatformTask,
)
from .platform_vision import detect_rings as detect_platform_rings
from abcd_detector.secondary_detector import SecondaryLetterDetector
from balls_detector import (
    BALL_COLORS,
    BALL_DETECTORS,
    DRAW_COLORS,
    MASK_BUILDERS,
    SHAPE_COLORS,
    blue,
    red,
    yellow,
)
from balls_detector.common import circularity as contour_circularity
from balls_detector.common import external_contours
from .field_mode import FieldMode, FieldTargetPolicy, parse_field, policy_for
from .white_line_alignment import WhiteLineAlignmentDetector


WINDOW_NAME = "Ros2_test1 Target Vision"

# H7 white-line frames carry an explicit image size and rejects measurements
# from another geometry. Keep both cameras on the calibrated route format.
ROUTE_CAMERA_WIDTH = 800
ROUTE_CAMERA_HEIGHT = 600

# Keep only the detector family needed by the current route stage.  The
# white-line detector is handled in the main loop and never shares a target
# result with the ball/letter pipelines.
DETECTION_MODE_IDLE = "idle"
DETECTION_MODE_DISC_BALLS = "disc_balls"
DETECTION_MODE_PLATFORM_TARGETS = "platform_targets"
DETECTION_MODE_COLUMN_LETTERS = "column_letters"
DETECTION_MODE_COLUMN_BLOCKS = "column_blocks"
DETECTION_MODE_COLUMN_ROTATED_LETTERS = "column_rotated_letters"
DETECTION_MODE_WHITE_LINE = "white_line"

# Task-three orbit acquisition is geometric first; rotation-aware letter
# classification is enabled only after H7 pauses on the locked block.
COLUMN_BLOCK_MIN_AREA = 700.0
COLUMN_BLOCK_MAX_AREA_RATIO = 0.12
COLUMN_BLOCK_MIN_SIDE = 28.0
COLUMN_BLOCK_MAX_SIDE_RATIO = 0.45
COLUMN_BLOCK_MIN_RECTANGULARITY = 0.55
COLUMN_BLOCK_MIN_SOLIDITY = 0.72
COLUMN_BLOCK_MIN_GREEN_SUPPORT = 0.42
COLUMN_BLOCK_MIN_GREEN_RING_SUPPORT = 0.55
# The white candidate must look like a quadrilateral, not a circular mark.
COLUMN_BLOCK_APPROX_EPSILON = 0.04
COLUMN_BLOCK_MIN_APPROX_VERTICES = 4
COLUMN_BLOCK_MAX_APPROX_VERTICES = 8
# A letter block retains a meaningful white interior around its dark glyph.
COLUMN_BLOCK_MIN_INNER_WHITE_RATIO = 0.12
COLUMN_BLOCK_GREEN_HUE_RANGE = (35, 105)
COLUMN_BLOCK_GREEN_MIN_SATURATION = 40
COLUMN_BLOCK_GREEN_MIN_VALUE = 25
# Keep the block away from the image boundary so clipped chassis hardware
# cannot pause the formal orbit.
COLUMN_BLOCK_EDGE_MARGIN_RATIO = 0.05
# A usable letter block has a non-white glyph/interior; reject solid white
# panels that only satisfy the outer rectangle geometry.
COLUMN_BLOCK_MAX_INNER_WHITE_RATIO = 0.97
# A real square has four continuous white sides and white corner joins. A
# circular hole/reflection has white pixels only on scattered arcs.
COLUMN_BLOCK_MIN_EDGE_WHITE_RATIO = 0.30
COLUMN_BLOCK_MIN_CORNER_WHITE_RATIO = 0.18
COLUMN_BLOCK_MIN_INNER_NONWHITE_RATIO = 0.15
COLUMN_BLOCK_MAX_CIRCULARITY = 0.88
# Keep H7 stopped long enough to classify the fresh letter inside the locked
# white-block ROI. A selected letter then gets a separate bounded tracking
# grace period while the arm centers; a brief detector gap must not skip it.
COLUMN_BLOCK_CLASSIFY_TIMEOUT_S = 3.0
COLUMN_BLOCK_TRACK_LOST_TIMEOUT_S = 3.0
COLUMN_BLOCK_CONFIRM_FRAMES = 1
COLUMN_CLASSIFY_VOTE_WINDOW = 6
COLUMN_CLASSIFY_MIN_VOTES = 4
COLUMN_CLASSIFY_MAX_ANGLE_JITTER_DEG = 22.0
COLUMN_BLOCK_MATCH_CENTER_PX = 55.0
COLUMN_BLOCK_MATCH_IOU = 0.35
# The block moves in the image while the arm recenters. Keep the original
# block identity with bounded motion and scale instead of fixed-frame IoU.
COLUMN_LOCKED_BLOCK_MAX_JUMP_PX = 220.0
COLUMN_LOCKED_BLOCK_MAX_SIZE_RATIO = 2.5
COLUMN_HANDLED_BLOCK_REARM_FRAMES = 15
COLUMN_LOCKED_LETTER_MAX_JUMP_PX = 260.0
COLUMN_LOCKED_LETTER_MAX_SIZE_RATIO = 2.8
COLUMN_H7_PAUSE_TIMEOUT_S = 3.0
COLUMN_H7_RESUME_TIMEOUT_S = 3.0

BALL_DISTANCE_OFFSET_CM = -1.6072186919749336
BALL_DISTANCE_SCALE_CM = 31.628878020276648
GOLF_BALL_DIAMETER_MM = 42.67
LETTER_CUBE_SIDE_MM = 30.0
RED_RING_OUTER_DIAMETER_MM = 55.0
RING_DISTANCE_EXTRA_CM = 0.5
POST_CENTER_REFERENCE_DISTANCE_MM = 235.0
POST_CENTER_MIN_DOWN_MM = 105.0
POST_CENTER_MAX_DOWN_MM = 210.0
POST_OPEN_RETREAT_MM = 50.0
POST_OPEN_ID2_RETREAT_TICKS = 100
SINGLE_ID2_DEADBAND_PX = 45.0
SINGLE_ID2_MIN_STEP_TICKS = 8
SINGLE_ID2_MAX_STEP_TICKS = 40
SINGLE_ID6_MIN_STEP_TICKS = 1
ID6_SAFE_LIMITS = (300, 700)
TASK23_ID6_CENTER_LIMITS = (0, 700)
TASK23_ID6_CENTER_STEP_TICKS = 5
CENTERING_SLOW_COMMAND_INTERVAL_S = 0.48
CENTERING_SLOW_ID2_GAIN = 0.06
CENTERING_SLOW_ID6_GAIN = 0.045
CENTERING_SLOW_ID2_MAX_STEP_TICKS = 12
CENTERING_SLOW_ID6_MAX_STEP_TICKS = 6
CENTERING_VECTOR_COMPONENT_DEADBAND_PX = 5.0
HTD85_SPLITTER_ID = 14
HTD85_CATCHER_ID = 15
HTD85_GRIPPER_ID = 17
# Task-one auxiliary calibration. Keep these separate from task-two/three
# high-pose values because the same physical bus IDs have different jobs.
TASK1_ID3_RETRACT_TICK = 300
TASK1_ID3_RETRACT_TIME_MS = 500
TASK1_ID14_RETRACT_TICK = 300
TASK1_ID14_FIELD_TICK = 700
TASK1_ID14_YELLOW_TICK = 300
TASK1_ID14_TIME_MS = 40
TASK1_ID15_RETRACT_TICK = 600
TASK1_ID15_OPEN_TICK = 730
TASK1_AUX_TIME_MS = 100
SPLITTER_RETRACT_TICK = 300
CATCHER_HOME_TICK = 600
CATCHER_RELEASE_READY_TICK = TASK1_ID15_OPEN_TICK
POST_GRAB_ID2_RETREAT_TICK = 100
DISC_CATCH_PREP_ID1_TICK = 650
DISC_CATCH_PREP_ID2_TICK = 600
DISC_CATCH_READY_ID1_TICK = 560
DISC_CATCH_READY_ID2_TICK = 550
DISC_CATCH_ID6_TICK = 413
DISC_CATCH_CATCHER_READY_TICK = TASK1_ID15_OPEN_TICK
DISC_CATCH_PREP_SPLITTER_TICK = TASK1_ID14_RETRACT_TICK
DISC_CATCH_SPLITTER_READY_TICK = TASK1_ID14_RETRACT_TICK
DISC_CATCH_TARGET_TIMEOUT_S = 4.0
DISC_CATCH_SPLITTER_FIELD_TICK = TASK1_ID14_FIELD_TICK
DISC_CATCH_SPLITTER_YELLOW_TICK = TASK1_ID14_YELLOW_TICK
DISC_CATCH_SPLITTER_RESET_TICK = TASK1_ID14_RETRACT_TICK
DISC_CATCH_CATCHER_FIELD_TICK = TASK1_ID15_OPEN_TICK
DISC_CATCH_CATCHER_YELLOW_TICK = TASK1_ID15_OPEN_TICK
# All formal RK flows use the same physical ID17 calibration.
DISC_CATCH_GRIPPER_OPEN_TICK = 420
DISC_CATCH_YELLOW_COOLDOWN_S = 0.5
DISC_CATCH_OPEN_HOLD_MARGIN_S = 0.1
DISC_CATCH_CLOSE_CONFIRM_DELAY_S = 0.05
DISC_CATCH_GRIPPER_TIME_MS = 80
DISC_CATCH_NORMAL_GRIPPER_TIME_MS = 150
# Task-one disc trigger window in the original 800x600 main-camera frame.
# Only a ball whose detected center is inside this window may trigger the
# splitter/catcher/gripper action.  This is intentionally local to DISC_CATCH;
# task-two and task-three target detectors do not use it.
DISC_CATCH_WINDOW_X_MIN = 280
DISC_CATCH_WINDOW_X_MAX = 720
DISC_CATCH_WINDOW_Y_MIN = 100
DISC_CATCH_WINDOW_Y_MAX = 500
# Blue-field ID14 must remain on the detected ball's channel for a full
# half-second before preparing the other channel for the next target.
DISC_CATCH_BLUE_CHANNEL_HOLD_S = 0.5
DISC_CATCH_BLUE_CLEAR_FRAMES = 3
ARM_JOINT_SEQUENCE_DELAY_S = 0.15
ARM_TUNE_COMMAND_PATH = Path("/home/cat/ros2_ws/arm_tune_command.txt")
ARM_TUNE_RESULT_PATH = Path("/home/cat/ros2_ws/arm_tune_result.txt")
ARM_TUNE_POLL_INTERVAL_S = 0.10
ARM_TUNE_85KG_LIMITS = {
    1: ID1_SAFE_LIMITS,
    2: ID2_SAFE_LIMITS,
    6: ID6_SAFE_LIMITS,
}
ARM_TUNE_HTD85_AUX_LIMITS = {
    14: (0, 1000),
    15: (0, 1000),
    17: (0, 1000),
}
COLUMN_CATCH_READY_ID1_TICK = 650
COLUMN_CATCH_READY_ID2_TICK = 570
# Formal task three keeps its own high-pose calibration. Task-two alignment
# must not change the column-catch pose implicitly.
COLUMN_CATCH_READY_ID6_TICK = 413
COLUMN_CATCH_AUX14_TICK, COLUMN_CATCH_AUX15_TICK, COLUMN_CATCH_GRIPPER_CLOSED_TICK = HTD85_AUX_HIGH
COLUMN_CATCH_SPLITTER_TICK = COLUMN_CATCH_AUX14_TICK
COLUMN_CATCH_CATCHER_HOME_TICK = COLUMN_CATCH_AUX15_TICK
COLUMN_CATCH_HOLD_HIGH = (650, 600, 413)
COLUMN_CATCH_GRIPPER_OPEN_TICK = PLATFORM_GRIPPER_OPEN
COLUMN_CATCH_GRIPPER_TIME_MS = PLATFORM_GRIPPER_TIME_MS
COLUMN_CATCH_CENTER_DEADBAND_PX = PLATFORM_CENTER_DEADBAND_PX
COLUMN_CATCH_ID2_CENTER_RANGE = PLATFORM_CENTER_ID2_RANGE
COLUMN_CATCH_ID6_CENTER_RANGE = PLATFORM_CENTER_ID6_RANGE
COLUMN_CATCH_ID2_CENTER_STEP_TICKS = PLATFORM_CENTER_ID2_STEP_TICKS
COLUMN_CATCH_ID6_CENTER_STEP_TICKS = PLATFORM_CENTER_ID6_STEP_TICKS
TASK3_RING_PLACE_HIGH = (650, 550, 413)
TASK3_RING_PLACE_RETURN_HIGH = (600, 480, 413)
TASK3_RING_PLACE_POSE = (470, 350, 171)
TASK3_RING_PLACE_AXIS_TIME_MS = 500
TASK3_RING_PLACE_INITIAL_HIGH_TIME_MS = 900
TASK3_RING_PLACE_ID1_TIME_MS = 700
TASK3_RING_PLACE_HIGH_TIME_MS = 1000
TASK3_RING_PLACE_GRIPPER_OPEN_TICK = PLATFORM_GRIPPER_OPEN
TASK3_RING_PLACE_GRIPPER_CLOSED_TICK = PLATFORM_GRIPPER_CLOSED
TASK3_RING_PLACE_GRIPPER_TIME_MS = 200
TASK3_RING_PLACE_SLOW_CLOSE_TIME_MS = 2000
TASK3_RING_PLACE_RELEASE_GRIPPER_TIME_MS = 1000
TASK3_RING_PLACE_RELEASE_HIGH_TIME_MS = 500
TASK3_RING_PLACE_RELEASE_HOLD_MS = 1000
TASK3_RING_PLACE_RELEASE_ID1_TICK = 580
TASK3_RING_PLACE_RELEASE_ID2_TICK = 467
TASK3_RING_PLACE_RELEASE_ID1_TIME_MS = 1000
TASK3_RING_PLACE_RELEASE_ID1_HOLD_MS = 1000
TASK3_RING_PLACE_CONTRACT_AXIS_TIME_MS = 500
COLUMN_CATCH_LETTER_PLACE = (530, 350, 670)
COLUMN_CATCH_LETTER_PLACE_TIME_MS = 500
COLUMN_CATCH_CENTER_TIME_MS = 100
# Formal BLUE task-three only: one fixed recovery grab before releasing the
# post-orbit H7 hold when the selected letter pair still has a quota gap.
TASK3_SUPPLEMENT_OPEN_HOLD_MS = 500
TASK3_SUPPLEMENT_DESCEND_ID1_TICK = 580
TASK3_SUPPLEMENT_DESCEND_TIME_MS = 800
TASK3_SUPPLEMENT_GRIPPER_TIME_MS = 200
TASK3_SUPPLEMENT_AXIS_TIME_MS = 500
TASK3_SUPPLEMENT_RETURN_HIGH_TIME_MS = 200
TASK3_SUPPLEMENT_PLACE_TIME_MS = 300
TASK3_SUPPLEMENT_HIGH = (
    COLUMN_CATCH_READY_ID1_TICK,
    COLUMN_CATCH_READY_ID2_TICK,
    COLUMN_CATCH_READY_ID6_TICK,
)
TASK3_SUPPLEMENT_PLACE = (545, 365, 600)
RING_DISTANCE_OFFSET_CM = BALL_DISTANCE_OFFSET_CM + RING_DISTANCE_EXTRA_CM
RING_DISTANCE_SCALE_CM = (
    BALL_DISTANCE_SCALE_CM
    * RED_RING_OUTER_DIAMETER_MM
    / GOLF_BALL_DIAMETER_MM
)

POSITION_REPORT_RE = re.compile(
    r"ID1=(-?\d+)\s+ID2=(-?\d+)(?:\s+ID6=(-?\d+|ERR))?"
)
ARM_READY_REPORT_RE = re.compile(r"OK\s+ARMREADY\s+ID1=(-?\d+)\s+ID2=(-?\d+)")
HTD85_MOVE_COMMAND = 0x01
HTD85_POSITION_READ_COMMAND = 0x1C
HTD85_FRAME_HEADER = b"\x55\x55"
HTD85_MIN_POSITION = 0
HTD85_MAX_POSITION = 1000


def htd85_checksum(body):
    return (~sum(body)) & 0xFF


def htd85_move_packet(servo_id, position, time_ms):
    servo_id = int(servo_id)
    position = max(HTD85_MIN_POSITION, min(HTD85_MAX_POSITION, int(position)))
    time_ms = max(0, min(30000, int(time_ms)))
    body = bytes(
        (
            servo_id,
            7,
            HTD85_MOVE_COMMAND,
            position & 0xFF,
            (position >> 8) & 0xFF,
            time_ms & 0xFF,
            (time_ms >> 8) & 0xFF,
        )
    )
    return HTD85_FRAME_HEADER + body + bytes((htd85_checksum(body),))


def htd85_position_read_packet(servo_id):
    body = bytes((int(servo_id), 3, HTD85_POSITION_READ_COMMAND))
    return HTD85_FRAME_HEADER + body + bytes((htd85_checksum(body),))


def htd85_read_position(fd, servo_id, timeout_s=0.25):
    termios_timeout = max(0.0, float(timeout_s))
    try:
        os.write(fd, htd85_position_read_packet(servo_id))
    except OSError:
        return None
    deadline = time.monotonic() + termios_timeout
    received = bytearray()

    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break
        try:
            chunk = os.read(fd, 64)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        received.extend(chunk)

        while True:
            header = received.find(HTD85_FRAME_HEADER)
            if header < 0:
                received.clear()
                break
            if header:
                del received[:header]
            if len(received) < 4:
                break
            frame_size = int(received[3]) + 3
            if frame_size < 6 or frame_size > 64:
                del received[0]
                continue
            if len(received) < frame_size:
                break
            frame = bytes(received[:frame_size])
            del received[:frame_size]
            body = frame[2:-1]
            if (
                frame[2] == int(servo_id)
                and frame[4] == HTD85_POSITION_READ_COMMAND
                and frame[-1] == htd85_checksum(body)
            ):
                if len(frame) >= 7:
                    return frame[5] | (frame[6] << 8)

    return None


def load_last_gripper_target(default):
    try:
        state = json.loads((Path.home() / ".servo_htd85_state.json").read_text())
        return max(0, min(1000, int(state.get("gripper_target", default))))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return int(default)


def scale_detections(detections, scale_x, scale_y):
    radius_scale = (scale_x + scale_y) * 0.5
    for detection in detections:
        if "center" in detection:
            center_x, center_y = detection["center"]
            detection["center"] = (
                int(round(center_x * scale_x)),
                int(round(center_y * scale_y)),
            )
        if "bbox" in detection:
            x, y, width, height = detection["bbox"]
            detection["bbox"] = (
                int(round(x * scale_x)),
                int(round(y * scale_y)),
                int(round(width * scale_x)),
                int(round(height * scale_y)),
            )
        for key in ("radius", "outer_radius", "inner_radius"):
            if key in detection:
                detection[key] = int(round(detection[key] * radius_scale))
        for key in ("box", "points"):
            if key in detection:
                points = np.asarray(detection[key], dtype=np.float32)
                # OpenCV contours are commonly shaped (N, 1, 2), while
                # detector boxes may already be (N, 2).  Normalize only the
                # point axis so a resized column-block contour cannot abort
                # the worker with "index 1 ... size 1".
                if points.size < 2 or points.size % 2:
                    continue
                point_shape = points.shape
                scaled = points.reshape(-1, 2).copy()
                scaled[:, 0] *= scale_x
                scaled[:, 1] *= scale_y
                detection[key] = scaled.reshape(point_shape).astype(np.int32)
        if "projected_area" in detection:
            detection["projected_area"] *= scale_x * scale_y
    return detections


class DetectionSmoother:
    def __init__(self, alpha=0.32, max_match_px=90.0, keep_missing_frames=6):
        self.alpha = max(0.05, min(1.0, float(alpha)))
        self.max_match_px = max(8.0, float(max_match_px))
        self.keep_missing_frames = max(1, int(keep_missing_frames))
        self.tracks = []

    @staticmethod
    def _center(det):
        if "center" in det:
            return np.asarray(det["center"], dtype=np.float32)
        if "bbox" in det:
            x, y, w, h = det["bbox"]
            return np.asarray((x + w * 0.5, y + h * 0.5), dtype=np.float32)
        return None

    @staticmethod
    def _copy_detection(det):
        copied = {}
        for key, value in det.items():
            if isinstance(value, np.ndarray):
                copied[key] = value.copy()
            else:
                copied[key] = value
        return copied

    def _new_track(self, det):
        track = {
            "color": det.get("color"),
            "kind": det.get("kind"),
            "missing": 0,
            "det": self._copy_detection(det),
        }
        self._init_float_state(track, det)
        return track

    @staticmethod
    def _init_float_state(track, det):
        for key in ("center", "bbox"):
            if key in det:
                track[key] = np.asarray(det[key], dtype=np.float32)
        for key in (
            "radius",
            "outer_radius",
            "inner_radius",
            "projected_area",
            "area_ratio",
            "area_percent",
            "diameter_ratio",
            "distance_cm",
        ):
            if key in det and det[key] is not None:
                track[key] = float(det[key])

    def _match_track(self, det, used):
        center = self._center(det)
        if center is None:
            return None
        best_index = None
        best_distance = self.max_match_px
        for index, track in enumerate(self.tracks):
            if index in used:
                continue
            if track.get("color") != det.get("color") or track.get("kind") != det.get("kind"):
                continue
            track_center = track.get("center")
            if track_center is None:
                track_center = self._center(track["det"])
            if track_center is None:
                continue
            distance = float(np.linalg.norm(center - track_center))
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def _smooth_track(self, track, det):
        alpha = self.alpha
        track["missing"] = 0
        track["det"] = self._copy_detection(det)
        for key in ("center", "bbox"):
            if key not in det:
                continue
            value = np.asarray(det[key], dtype=np.float32)
            if key in track and np.asarray(track[key]).shape == value.shape:
                track[key] = (1.0 - alpha) * track[key] + alpha * value
            else:
                track[key] = value
        for key in (
            "radius",
            "outer_radius",
            "inner_radius",
            "projected_area",
            "area_ratio",
            "area_percent",
            "diameter_ratio",
            "distance_cm",
        ):
            if key not in det or det[key] is None:
                continue
            value = float(det[key])
            track[key] = (1.0 - alpha) * track[key] + alpha * value if key in track else value

    def _track_to_detection(self, track):
        det = self._copy_detection(track["det"])
        # A retained track is useful for display continuity, but must never be
        # treated as a new visual measurement by the grasp state machine.
        det["observed"] = track.get("missing", 0) == 0
        if "center" in track:
            det["center"] = tuple(int(round(v)) for v in track["center"])
        if "bbox" in track:
            det["bbox"] = tuple(int(round(v)) for v in track["bbox"])
        for key in ("radius", "outer_radius", "inner_radius"):
            if key in track:
                det[key] = int(round(track[key]))
        for key in (
            "projected_area",
            "area_ratio",
            "area_percent",
            "diameter_ratio",
            "distance_cm",
        ):
            if key in track:
                det[key] = float(track[key])
        return det

    def update(self, detections):
        used = set()
        smoothed = []
        for det in detections:
            index = self._match_track(det, used)
            if index is None:
                track = self._new_track(det)
                self.tracks.append(track)
                index = len(self.tracks) - 1
            else:
                track = self.tracks[index]
                self._smooth_track(track, det)
            used.add(index)
            smoothed.append(self._track_to_detection(track))

        kept_tracks = []
        for index, track in enumerate(self.tracks):
            if index in used:
                kept_tracks.append(track)
                continue
            track["missing"] = track.get("missing", 0) + 1
            if track["missing"] <= self.keep_missing_frames:
                kept_tracks.append(track)
        self.tracks = kept_tracks
        return smoothed


_PROCESS_DETECTOR = None


def detection_process_worker(
    source_frame,
    detection_scale,
    detection_mode=DETECTION_MODE_IDLE,
    field_name="red",
    target_letters=(),
    *,
    platform=False,
    task3_roi=None,
):
    if platform:
        detection_mode = DETECTION_MODE_PLATFORM_TARGETS
    global _PROCESS_DETECTOR
    if _PROCESS_DETECTOR is None:
        _PROCESS_DETECTOR = TargetDetector()
    detect_started = time.perf_counter()
    # Formal task-one blue balls are small at the station. Keep the full
    # 800x600 frame for that path; task-two/task-three retain their launch
    # scale and timing.
    effective_detection_scale = (
        1.0
        if (
            detection_mode == DETECTION_MODE_DISC_BALLS
            and str(field_name or "").strip().lower() == "blue"
        )
        else detection_scale
    )
    if effective_detection_scale < 0.999:
        source_height, source_width = source_frame.shape[:2]
        detect_width = max(1, int(round(source_width * effective_detection_scale)))
        detect_height = max(1, int(round(source_height * effective_detection_scale)))
        detect_frame = cv2.resize(
            source_frame,
            (detect_width, detect_height),
            interpolation=cv2.INTER_AREA,
        )
        detect_roi = None
        if task3_roi is not None and len(task3_roi) >= 4:
            x, y, roi_width, roi_height = (float(value) for value in task3_roi[:4])
            detect_roi = (
                int(round(x * effective_detection_scale)),
                int(round(y * effective_detection_scale)),
                int(round(roi_width * effective_detection_scale)),
                int(round(roi_height * effective_detection_scale)),
            )
        result = _PROCESS_DETECTOR.detect(
            detect_frame,
            mode=detection_mode,
            field_name=field_name,
            target_letters=target_letters,
            task3_roi=detect_roi,
        )
        result = scale_detections(
            result,
            source_width / float(detect_width),
            source_height / float(detect_height),
        )
    else:
        result = _PROCESS_DETECTOR.detect(
            source_frame,
            mode=detection_mode,
            field_name=field_name,
            target_letters=target_letters,
            task3_roi=task3_roi,
        )
    return result, time.perf_counter() - detect_started

_PROCESS_SECONDARY_DETECTOR = None


def secondary_detection_process_worker(frame):
    global _PROCESS_SECONDARY_DETECTOR
    if _PROCESS_SECONDARY_DETECTOR is None:
        _PROCESS_SECONDARY_DETECTOR = SecondaryLetterDetector()
    started = time.perf_counter()
    return _PROCESS_SECONDARY_DETECTOR.detect(frame), time.perf_counter() - started


def secondary_detector_warmup_worker():
    """Load the secondary detector before task-two preselection starts."""
    global _PROCESS_SECONDARY_DETECTOR
    if _PROCESS_SECONDARY_DETECTOR is None:
        _PROCESS_SECONDARY_DETECTOR = SecondaryLetterDetector()
    return True


class FrameState:
    def __init__(self):
        self.frame = None
        self.info = "starting"
        self.running = True
        self.lock = threading.Lock()

    def update(self, frame, info):
        with self.lock:
            self.frame = frame.copy()
            self.info = info

    def snapshot(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            return frame, self.info


class WindowCloseWatcher:
    def __init__(self, title):
        self.title = title
        self.closed = threading.Event()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stopped.set()
        self.thread.join(timeout=0.5)

    def _run(self):
        window_id = None
        while not self.stopped.wait(0.2):
            result = subprocess.run(
                ["xwininfo", "-name", self.title],
                capture_output=True,
                text=True,
                check=False,
            )
            match = re.search(r"Window id:\s+(0x[0-9a-fA-F]+)", result.stdout)
            if match is not None:
                window_id = match.group(1)
                break
        if window_id is None:
            return
        while not self.stopped.wait(0.2):
            result = subprocess.run(
                ["xprop", "-id", window_id],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if result.returncode != 0:
                self.closed.set()
                return


class StreamHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        state = self.server.state
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(
                b"<html><head><meta charset='utf-8'><title>Target Vision</title></head>"
                b"<body style='margin:0;background:#111;color:#eee;text-align:center'>"
                b"<img src='/video' style='max-width:100%;height:auto'>"
                b"<pre id='info' style='font-size:18px'></pre>"
                b"<script>setInterval(()=>fetch('/status').then(r=>r.text()).then(t=>info.textContent=t),300)</script>"
                b"</body></html>"
            )
            return

        if self.path == "/status":
            _, info = state.snapshot()
            self.send_response(200)
            self.send_header("Content-type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(info.encode("utf-8", "replace"))
            return

        if self.path == "/video":
            self.send_response(200)
            self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while state.running:
                    frame, _ = state.snapshot()
                    if frame is not None:
                        ok, jpg = cv2.imencode(
                            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                        )
                        if ok:
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                + jpg.tobytes()
                                + b"\r\n"
                            )
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_error(404)

    def log_message(self, *args):
        pass


class ThreadedHTTPServer(http.server.ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, addr, handler, state):
        self.state = state
        super().__init__(addr, handler)


class ArmTuneFileBridge:
    """Polls a local file so the owner process can safely tune bus servos."""

    def __init__(self, command_path, result_path, poll_interval_s=0.10):
        self.command_path = Path(command_path)
        self.result_path = Path(result_path)
        self.poll_interval_s = max(0.02, float(poll_interval_s))
        self.last_poll = 0.0
        self.last_mtime_ns = None
        self.last_command_text = ""

    def _write_result(self, message):
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        text = f"{stamp} {message}\n"
        try:
            self.result_path.write_text(text)
        except OSError as exc:
            print(f"ARM_TUNE RESULT_WRITE_FAILED {exc}", flush=True)
        print(f"ARM_TUNE {message}", flush=True)
        return message

    def poll(self, controller, chassis_link, frame_shape):
        now = time.monotonic()
        if now - self.last_poll < self.poll_interval_s:
            return None
        self.last_poll = now
        try:
            stat = self.command_path.stat()
        except OSError:
            return None
        if stat.st_size <= 0:
            return None
        if stat.st_mtime_ns == self.last_mtime_ns:
            return None
        self.last_mtime_ns = stat.st_mtime_ns
        try:
            command_text = self.command_path.read_text().strip()
        except OSError as exc:
            return self._write_result(f"ERR READ_FAILED {exc}")
        if not command_text or command_text == self.last_command_text:
            return None
        self.last_command_text = command_text
        return self._execute(command_text, controller, chassis_link, frame_shape)

    def _execute(self, command_text, controller, chassis_link, frame_shape):
        parts = command_text.replace(",", " ").split()
        if not parts:
            return None
        verb = parts[0].upper()
        if verb == "STATUS":
            return self._write_result(
                "OK STATUS "
                f"active_task={chassis_link.active_task} "
                f"controller_station={controller.active_chassis_station} "
                f"ID1={controller.id1} ID2={controller.id2} "
                f"ID14={controller.splitter_id4} ID15={controller.id5} "
                f"ID6={controller.id6} ID17={controller.id7}"
            )
        if verb not in {"SET", "MOVE"}:
            return self._write_result("ERR UNKNOWN_COMMAND use: SET ID6 580 [ID1 500 ...] or STATUS")
        if controller.active_chassis_station is not None or chassis_link.active_task is not None:
            return self._write_result(
                "ERR BUSY active_task="
                f"{chassis_link.active_task} controller_station={controller.active_chassis_station}"
            )
        servo_ready = (
            controller.servo_bridge.enabled
            and controller.servo_bridge.write_enabled
            and getattr(controller.servo_bridge, "arm_fd", None) is not None
            and getattr(controller.servo_bridge, "arm_fd", None) is not None
        )
        if not servo_ready:
            return self._write_result(f"ERR SERVO_NOT_READY {controller.servo_bridge.status}")
        if (len(parts) - 1) % 2 != 0:
            return self._write_result("ERR BAD_ARGS use pairs like: SET ID6 580 ID17 405")

        targets = {}
        for index in range(1, len(parts), 2):
            servo_token = parts[index].upper()
            if not servo_token.startswith("ID"):
                return self._write_result(f"ERR BAD_SERVO {parts[index]}")
            try:
                servo_id = int(servo_token[2:])
                value = int(parts[index + 1])
            except ValueError:
                return self._write_result(f"ERR BAD_VALUE {parts[index]} {parts[index + 1]}")
            limits = ARM_TUNE_85KG_LIMITS.get(servo_id) or ARM_TUNE_HTD85_AUX_LIMITS.get(servo_id)
            if limits is None:
                return self._write_result("ERR UNSUPPORTED_ID allowed=ID1,ID2,ID6,ID14,ID15,ID17")
            lower, upper = limits
            if value < lower or value > upper:
                return self._write_result(f"ERR RANGE ID{servo_id} {value} allowed={lower}-{upper}")
            targets[servo_id] = value

        if not targets:
            return self._write_result("ERR NO_TARGETS")

        send_kwargs = {}
        if 1 in targets:
            send_kwargs["id1"] = targets[1]
        if 2 in targets:
            send_kwargs["id2"] = targets[2]
        if 6 in targets:
            send_kwargs["id6"] = targets[6]
        if 14 in targets:
            send_kwargs["splitter_id4"] = targets[14]
        if 15 in targets:
            send_kwargs["id5"] = targets[15]
        if 17 in targets:
            send_kwargs["id4"] = targets[17]

        status = controller.servo_bridge.send_targets(**send_kwargs)
        if not controller.servo_bridge.last_command_ok:
            return self._write_result(f"ERR WRITE_FAILED {status}")

        if 1 in targets:
            controller.id1 = targets[1]
        if 2 in targets:
            controller.id2 = targets[2]
        if 6 in targets:
            controller.id6 = targets[6]
        if 14 in targets:
            controller.splitter_id4 = targets[14]
        if 15 in targets:
            controller.id5 = targets[15]
        if 17 in targets:
            controller.id7 = targets[17]
        controller.last_command_time = time.monotonic()
        controller.arm_preview.set_targets(controller.id1, controller.id2, controller.id7, controller.id6)
        controller.arm_preview.publish(
            "arm tune " + " ".join(f"ID{sid}={value}" for sid, value in sorted(targets.items())),
            None,
            controller.state,
        )
        return self._write_result(
            "OK SET "
            + " ".join(f"ID{sid}={value}" for sid, value in sorted(targets.items()))
            + f" | {status}"
        )


class HiwonderSingleBusServoBridge:
    """Single HTD-85 bus for arm IDs 1/2/3/6 and replacement IDs 14/15/17.

    Logical callers retain the historical task names: ``splitter_id4`` maps to
    physical ID14, ``id5`` maps to physical ID15, and ``id4`` maps to physical
    ID17.  Physical ID6 remains physical ID6 and is never remapped.
    """

    LOGICAL_TO_PHYSICAL = {1: 1, 2: 2, 3: 3, 6: 6, 4: 14, 5: 15, 7: 17}

    def __init__(
        self,
        device,
        baudrate,
        enabled=True,
        write_enabled=True,
        arm_device=None,
        **kwargs,
    ):
        self.device = device
        self.arm_device = arm_device or device
        self.baudrate = int(baudrate)
        self.enabled = bool(enabled)
        self.write_enabled = bool(enabled and write_enabled)
        self.arm_time_ms = int(kwargs.get("arm_time_ms", PLATFORM_ARM_TIME_MS))
        self.gripper_time_ms = int(kwargs.get("gripper_time_ms", 200))
        self.splitter_time_ms = int(kwargs.get("splitter_time_ms", 200))
        self.aux_time_ms = int(kwargs.get("aux_time_ms", 200))
        self.repeat = max(1, min(8, int(kwargs.get("repeat", 1))))
        self.arm_fd = None
        self.last_feedback = None
        self.last_command_ok = False
        self.last_ack = {}
        self.assumed_feedback = True
        self._arm_io_lock = threading.RLock()
        self._next_open_retry = 0.0
        self.status = "single HTD85 bus disabled"
        if self.enabled:
            self._open()

    def _open_device(self, device):
        subprocess.run(
            [
                "stty", "-F", device, str(self.baudrate), "cs8", "-cstopb",
                "-parenb", "-ixon", "-ixoff", "-crtscts", "raw", "-echo",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            if fcntl is not None and hasattr(termios, "TIOCEXCL"):
                fcntl.ioctl(fd, termios.TIOCEXCL)
        except OSError:
            os.close(fd)
            raise
        return fd

    def _open(self):
        if not self.enabled or self.arm_fd is not None:
            return self.arm_fd is not None
        now = time.monotonic()
        if now < self._next_open_retry:
            return False
        self._next_open_retry = now + 0.5
        try:
            self.arm_fd = self._open_device(self.arm_device)
            self._next_open_retry = 0.0
            self.status = (
                f"single HTD85 bus={self.arm_device} baud={self.baudrate} "
                "physical_ids=1,2,3,6,14,15,17 exclusive=yes"
            )
            print(self.status, flush=True)
            return True
        except (OSError, subprocess.CalledProcessError) as exc:
            self.close()
            self.status = f"single HTD85 bus open failed: {exc}"
            print(self.status, flush=True)
            return False

    def _ensure_open(self):
        return self.arm_fd is not None or self._open()

    def _write_payload(self, payload, repeat=None):
        count = self.repeat if repeat is None else max(1, min(8, int(repeat)))
        if self.arm_fd is None:
            raise OSError("HTD85 bus is closed")
        for attempt in range(count):
            offset = 0
            deadline = time.monotonic() + 0.2
            while offset < len(payload):
                try:
                    written = os.write(self.arm_fd, payload[offset:])
                    if written <= 0:
                        raise OSError("serial write returned zero bytes")
                    offset += written
                except BlockingIOError:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("HTD85 serial write timed out")
                    _, writable, _ = select.select([], [self.arm_fd], [], min(0.02, remaining))
                    if not writable:
                        continue
            if attempt < count - 1:
                time.sleep(0.08)

    def _send_targets(self, targets, repeat=None):
        if not targets:
            return True, ""
        sent = []
        try:
            with self._arm_io_lock:
                for physical_id, value, motion_ms in targets:
                    value = max(HTD85_MIN_POSITION, min(HTD85_MAX_POSITION, int(value)))
                    payload = htd85_move_packet(physical_id, value, motion_ms)
                    self._write_payload(payload, repeat=repeat)
                    sent.append(f"ID{physical_id}={value}@{motion_ms}ms")
                    print(
                        f"DIRECT SERVO TX htd85-binary={self.arm_device} "
                        f"{self._format_payload(payload)} physical_id={physical_id} "
                        f"target={value} time={motion_ms}ms",
                        flush=True,
                    )
                    time.sleep(0.003)
        except (OSError, TimeoutError) as exc:
            # EIO/ENODEV leaves the old descriptor unusable after a USB
            # adapter reset.  Drop it so the next readiness poll can reopen
            # the stable by-id path and let H7 retry RESET/PREP_HIGH.
            self.close()
            warning = f"HTD85 UART write failed: {exc}"
            print(warning, flush=True)
            return False, warning
        return True, "HTD85 TX " + ",".join(sent)

    @staticmethod
    def _format_payload(payload):
        return " ".join(f"{byte:02X}" for byte in payload)

    def send_targets(self, id1=None, id2=None, id3=None, id4=None, id6=None,
                     id5=None, splitter_id4=None, repeat=None,
                     aux_time_ms=None, splitter_time_ms=None):
        if not self.enabled or not self._ensure_open():
            self.last_command_ok = False
            return self.status
        if not self.write_enabled:
            self.last_command_ok = False
            return "single HTD85 bus read-only; targets not sent"
        targets = []
        logical = []
        aux_motion_ms = (
            self.aux_time_ms if aux_time_ms is None else int(aux_time_ms)
        )
        splitter_motion_ms = (
            self.splitter_time_ms
            if splitter_time_ms is None
            else int(splitter_time_ms)
        )
        for servo_id, value in ((1, id1), (2, id2), (3, id3), (6, id6)):
            if value is not None:
                targets.append((servo_id, int(value), self.arm_time_ms))
                logical.append((servo_id, int(value)))
        if splitter_id4 is not None:
            targets.append((
                HTD85_SPLITTER_ID,
                int(splitter_id4),
                splitter_motion_ms,
            ))
            logical.append((HTD85_SPLITTER_ID, int(splitter_id4)))
        if id5 is not None:
            targets.append((HTD85_CATCHER_ID, int(id5), aux_motion_ms))
            logical.append((HTD85_CATCHER_ID, int(id5)))
        if id4 is not None:
            targets.append((HTD85_GRIPPER_ID, int(id4), self.gripper_time_ms))
            logical.append((HTD85_GRIPPER_ID, int(id4)))
        if not targets:
            return self.status
        self.last_command_ok = False
        ok, status = self._send_targets(targets, repeat=repeat)
        if not ok:
            self.status = status
            return self.status
        previous = self.last_feedback or (READY_ID1_TICK, READY_ID2_TICK, None)
        self.last_feedback = (
            next((value for sid, value in logical if sid == 1), previous[0]),
            next((value for sid, value in logical if sid == 2), previous[1]),
            next((value for sid, value in logical if sid == 6), previous[2]),
        )
        self.last_command_ok = True
        self.status = status + " feedback=OPTIONAL"
        return self.status

    def send_aux_request(self, servo_id=None, pulse=None, time_ms=None, **kwargs):
        """Send the dedicated HTD85 ID3 request on the same exclusive bus."""
        if (
            not self.enabled
            or not self._ensure_open()
            or not self.write_enabled
            or int(servo_id or 0) != 3
            or pulse is None
            or not 0 <= int(pulse) <= HTD85_MAX_POSITION
            or time_ms is None
            or not 0 <= int(time_ms) <= 30000
        ):
            self.last_command_ok = False
            self.status = "HTD85 auxiliary request rejected"
            return self.status
        self.last_command_ok = False
        ok, status = self._send_targets(
            [(3, int(pulse), int(time_ms))]
        )
        self.last_command_ok = bool(ok)
        self.status = status if ok else status
        return self.status

    def _read_htd85_position(self, servo_id, timeout_s=0.25):
        if not self._ensure_open():
            return None
        with self._arm_io_lock:
            try:
                if termios is not None:
                    termios.tcflush(self.arm_fd, termios.TCIFLUSH)
                os.write(self.arm_fd, htd85_position_read_packet(servo_id))
                return htd85_read_position(self.arm_fd, servo_id, timeout_s)
            except OSError:
                return None

    def query_positions(self, timeout_s=0.90):
        if not self.enabled or not self._ensure_open():
            return None
        positions = {}
        per_servo_timeout = max(0.08, min(0.30, float(timeout_s) / 3.0))
        for servo_id in (1, 2, 6):
            position = self._read_htd85_position(servo_id, per_servo_timeout)
            if position is not None:
                positions[servo_id] = int(position)
        if 1 in positions and 2 in positions:
            self.last_feedback = (positions[1], positions[2], positions.get(6))
            self.last_command_ok = True
            return self.last_feedback
        return None

    def ready_for_commands(self):
        return self.enabled and self.write_enabled and self._ensure_open()

    def wait_ready(self, timeout_s=0.2):
        return self.enabled and self._ensure_open()

    def command_arm_ready(self, timeout_s=4.0):
        if not self._ensure_open():
            return None
        feedback = self.query_positions(timeout_s=min(0.8, max(0.2, float(timeout_s))))
        if feedback is not None:
            self.last_command_ok = True
        return feedback

    def close(self):
        if self.arm_fd is not None:
            try:
                os.close(self.arm_fd)
            except OSError:
                pass
        self.arm_fd = None
        self.last_command_ok = False

class TargetGraspController:
    def __init__(
        self,
        enabled,
        servo_bridge,
        arm_preview,
        id1_ready,
        id2_ready,
        id7_closed,
        id7_open,
        center_deadband_px,
        stable_frames,
        command_interval_s,
        retrigger_cooldown_s,
        id2_pixel_gain,
        id6_pixel_gain,
        id6_max_step_ticks,
        id1_pixel_gain_y,
        id2_distance_gain,
        camera_gripper_offset_mm,
        target_gripper_distance_mm,
        distance_deadband_mm,
        max_step_ticks,
        id1_limits,
        id2_limits,
        angle_gap_degrees,
        startup_sequence,
        one_shot,
        camera_gripper_vertical_offset_mm,
        max_lateral_offset_mm,
        max_one_shot_ik_error_mm,
        post_center_retreat_mm,
        post_center_down_mm,
        post_center_ik_error_mm,
        min_target_area_percent=0.4,
        min_target_distance_cm=GRASP_MIN_DISTANCE_CM,
        max_target_distance_cm=GRASP_MAX_DISTANCE_CM,
        post_center_direct_descend=False,
        simple_vertical_grasp=False,
        vertical_grasp_id1=HOME_ID1_TICK,
        vertical_grasp_id2=READY_ID2_TICK,
        initial_id4=None,
        id2_center_min=None,
        id2_center_max=None,
        field_mode=FieldMode.RED,
        target_letters=LETTERS,
    ):
        self.enabled = enabled
        self.servo_bridge = servo_bridge
        self.arm_preview = arm_preview
        self.id1 = int(id1_ready)
        self.id2 = int(id2_ready)
        self.id7 = int(id7_closed if initial_id4 is None else initial_id4)
        self.id6 = int(arm_preview.id6)
        self.splitter_id4 = SPLITTER_RETRACT_TICK
        self.id5 = CATCHER_HOME_TICK
        self.field_mode = parse_field(field_mode)
        self.target_policy = policy_for(self.field_mode)
        self.target_letters = frozenset(target_letters) or frozenset(LETTERS)
        self.id7_closed = int(id7_closed)
        self.id7_open = int(id7_open)
        self.id4_closed = self.id7_closed  # legacy compatibility for older callers
        self.id4_open = self.id7_open  # legacy compatibility for older callers
        self.center_deadband_px = center_deadband_px
        self.stable_frames_required = max(1, stable_frames)
        self.command_interval_s = command_interval_s
        self.retrigger_cooldown_s = max(0.0, float(retrigger_cooldown_s))
        self.ignore_new_targets_until = 0.0
        self.id2_pixel_gain = id2_pixel_gain
        self.id6_pixel_gain = max(0.0, float(id6_pixel_gain))
        self.id6_max_step_ticks = max(1, int(id6_max_step_ticks))
        self.id1_pixel_gain_y = id1_pixel_gain_y
        self.id2_distance_gain = id2_distance_gain
        self.camera_gripper_offset_mm = camera_gripper_offset_mm
        self.camera_gripper_vertical_offset_mm = camera_gripper_vertical_offset_mm
        self.target_gripper_distance_mm = target_gripper_distance_mm
        self.max_lateral_offset_mm = max_lateral_offset_mm
        self.max_one_shot_ik_error_mm = max_one_shot_ik_error_mm
        self.post_center_retreat_mm = float(post_center_retreat_mm)
        self.post_center_down_mm = float(post_center_down_mm)
        self.post_center_ik_error_mm = float(post_center_ik_error_mm)
        self.min_target_area_percent = max(0.0, float(min_target_area_percent))
        self.min_target_distance_cm = float(min_target_distance_cm)
        self.max_target_distance_cm = float(max_target_distance_cm)
        self.post_center_direct_descend = bool(post_center_direct_descend)
        self.simple_vertical_grasp = bool(simple_vertical_grasp)
        self.vertical_grasp_id1 = int(vertical_grasp_id1)
        self.vertical_grasp_id2 = int(vertical_grasp_id2)
        self.id2_center_min = None if id2_center_min is None else int(id2_center_min)
        self.id2_center_max = None if id2_center_max is None else int(id2_center_max)
        self.distance_deadband_mm = distance_deadband_mm
        self.max_step_ticks = max(1, int(max_step_ticks))
        self.id1_limits = id1_limits
        self.id2_limits = id2_limits
        self.preview_id1_limits = (
            max(id1_limits[0], self.id1 - 90),
            min(id1_limits[1], self.id1 + 90),
        )
        self.preview_id2_limits = (
            max(id2_limits[0], self.id2 - 90),
            min(id2_limits[1], self.id2 + 90),
        )
        self.angle_gap_degrees = max(0.0, float(angle_gap_degrees))
        self.centered_frames = 0
        self.center_distance_samples = []
        self.horizontal_correction_done = False
        self.centering_correction_count = 0
        self.last_command_time = 0.0
        self.last_aux_command_time = 0.0
        self.aux_command_interval_s = 0.25
        self.last_preview_step_time = 0.0
        self.preview_step_interval_s = 0.12
        self.last_feedback_attempt = 0.0
        self.feedback_due = 0.0
        self.feedback_pending = False
        self.feedback_failures = 0
        self.startup_sequence = bool(startup_sequence)
        self.startup_stage = "complete"
        self.startup_deadline = 0.0
        self.startup_verify_deadline = 0.0
        self.startup_next_feedback = 0.0
        self.startup_feedback_attempts = 0
        self.startup_ready_resends = 0
        self.startup_ready_timeout = time.monotonic() + 120.0
        self.startup_position_tolerance = 10
        self.one_shot = one_shot
        self.one_shot_target = None
        self.one_shot_approach_sent = False
        self.one_shot_complete = False
        self.post_center_move_complete = False
        self.locked_target = None
        self.locked_plan = None
        self.last_visual_target = None
        self.visual_confirm_frames = 0
        self.visual_lost_frames = 0
        self.visual_descend_start = None
        self.visual_descend_step_index = 0
        self.visual_descend_steps = 4
        self.target_jump_reset_px = max(180.0, float(center_deadband_px) * 5.0)
        self.centering_jump_reset_px = max(120.0, float(center_deadband_px) * 4.0)
        self.abort_after_return = False
        self.approach_attempts = 0
        self.return_attempts = 0
        self.active_chassis_station = None
        self.chassis_station_done_reason = None
        self.chassis_station_error_reason = None
        self.chassis_station_stage = None
        self.chassis_station_deadline = 0.0
        self.chassis_station_no_target_deadline = 0.0
        self.task3_ring_place_actions = deque()
        self.task3_supplement_actions = deque()
        self.task3_supplement_label = None
        self.task3_supplement_active = False
        self.task3_supplement_done = False
        self.task3_supplement_hold_completed = False
        self.disc_pulse_done = False
        self.disc_last_pulsed_color = None
        self.disc_last_pulsed_center = None
        self.disc_blue_clear_frames = 0
        self.disc_fast_blue_cycle = False
        self.disc_blue_channel_hold_deadline = 0.0
        self.disc_prep_high_active = False
        self.column_target_armed = True
        self.column_target_absent_frames = 0
        self.column_pending_target = None
        self.column_pending_frames = 0
        self.column_locked_block = None
        self.column_locked_distance_cm = None
        # A white block alone must never authorize arm descent.
        self.column_capture_authorized = False
        self.column_handled_blocks = []
        self.column_classify_deadline = 0.0
        self.column_classify_votes = []
        self.column_target_lost_since = None
        self.column_pause_deadline = 0.0
        self.column_resume_deadline = 0.0
        self.column_timeout_recovering = False
        self.letter_success_counts = {}
        self.platform_selected_letters = frozenset()
        self.platform_preselect_letters = set()
        self.platform_preselect_count = 2
        self.platform_task = PlatformTask(
            self._platform_pose,
            self._platform_gripper,
            self._platform_center,
            retreat_pose=self._platform_retreat_pose,
            gripper_open=self._platform_gripper_open,
            gripper_close=self._platform_gripper_close,
            place_open=self._platform_place_open,
            place_close=self._platform_place_close,
            ring_place_id6=self._platform_ring_place_id6,
            ring_place_id2=self._platform_ring_place_id2,
            ring_place_id1=self._platform_ring_place_id1,
            ring_place_id12=self._platform_ring_place_id12,
            ring_return_high_id1=self._platform_ring_return_high_id1,
            ring_return_high_id2=self._platform_ring_return_high_id2,
            ring_return_high_id6=self._platform_ring_return_high_id6,
            letter_place_id6=self._platform_letter_place_id6,
            letter_place_id2=self._platform_letter_place_id2,
            letter_place_id1=self._platform_letter_place_id1,
            letter_place_id12=self._platform_letter_place_id12,
            letter_return_high_id1=self._platform_letter_return_high_id1,
            letter_return_high_id2=self._platform_letter_return_high_id2,
            letter_return_high_id6=self._platform_letter_return_high_id6,
            letter_success_counts=self.letter_success_counts,
        )
        # Keep the arm in the task-two observation pose between slot
        # transactions.  The chassis link clears active_task after every
        # slot, so this state must live in the arm controller itself.
        self.platform_high_hold = False
        self.platform_high_pose_sent = False
        self.approach_feedback_tolerance = 10
        self.stage_deadline = 0.0
        self.next_stage_after_open = None
        self.next_stage_after_retreat = None
        self.motion_stage_after_wait = None
        self.algorithm_stage = "centering"
        self.read_only_sync = enabled and servo_bridge.enabled and not servo_bridge.write_enabled
        self.read_only_feedback_lock = threading.Lock()
        self.read_only_feedback_thread = None
        self.read_only_feedback_ready = False
        self.read_only_feedback = None
        self.synchronized = (
            not (enabled and servo_bridge.enabled)
            or getattr(servo_bridge, "assumed_feedback", False)
        )
        self.state = "disabled" if not enabled else "searching"
        self.status = "target grasp disabled" if not enabled else "letter target grasp ready"

        self._enforce_angle_gap()
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        if self.enabled and self.servo_bridge.write_enabled:
            if self.startup_sequence:
                if getattr(self.servo_bridge, "assumed_feedback", False):
                    self.startup_stage = "send_home"
                    self.state = "startup_home"
                    self.status = "RK direct startup: sending home before ready"
                else:
                    self.startup_stage = "wait_ready"
                    self.state = "startup_wait_ready"
                    self.status = "waiting for Hiwonder HTD85 board ready"
            else:
                self.state = "waiting servo feedback"
                self.status = "waiting for Hiwonder HTD85 position synchronization"
        elif self.read_only_sync:
            self.state = "read_only_sync"
            self.status = "waiting for read-only HTD85 position synchronization"

    def set_field_mode(self, field_mode):
        """Apply a new field only while no chassis station is active."""

        requested = parse_field(field_mode)
        if (
            self.active_chassis_station is not None
            and requested != self.field_mode
        ):
            return False
        self.field_mode = requested
        self.target_policy = policy_for(requested)
        return True

    @staticmethod
    def _clamp(value, limits):
        lower, upper = limits
        return max(lower, min(upper, int(round(value))))

    def _limited_delta(self, delta):
        return max(-self.max_step_ticks, min(self.max_step_ticks, int(round(delta))))

    def _arm_settle_s(self):
        return max(0.15, getattr(self.servo_bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS) / 1000.0 + 0.12)

    def _aux_settle_s(self):
        return max(0.12, getattr(self.servo_bridge, "aux_time_ms", 200) / 1000.0 + 0.08)

    def _gripper_settle_s(self):
        return max(
            0.12,
            getattr(self.servo_bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS) / 1000.0 + 0.05,
        )

    def _gripper_motion_s(self):
        return max(
            0.10,
            getattr(self.servo_bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS) / 1000.0,
        )

    def _disc_low_pose_settle_s(self):
        return max(
            0.15,
            max(
                getattr(self.servo_bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS),
                getattr(self.servo_bridge, "aux_time_ms", 200),
                getattr(self.servo_bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS),
            )
            / 1000.0
            + 0.15,
        )

    def _disc_open_hold_s(self):
        gripper_time_ms = (
            DISC_CATCH_GRIPPER_TIME_MS
            if self.disc_fast_blue_cycle
            else DISC_CATCH_NORMAL_GRIPPER_TIME_MS
        )
        return max(
            0.12,
            gripper_time_ms / 1000.0 + DISC_CATCH_OPEN_HOLD_MARGIN_S,
        )

    def _disc_gripper_motion_s(self):
        gripper_time_ms = (
            DISC_CATCH_GRIPPER_TIME_MS
            if self.disc_fast_blue_cycle
            else DISC_CATCH_NORMAL_GRIPPER_TIME_MS
        )
        return max(0.10, gripper_time_ms / 1000.0)

    def _splitter_settle_s(self):
        return max(
            0.12,
            getattr(self.servo_bridge, "splitter_time_ms", PLATFORM_AUX_TIME_MS) / 1000.0 + 0.08,
        )

    def _clamp_center_id2(self, value):
        value = self._clamp(value, self.id2_limits)
        if self.id2_center_min is not None:
            value = max(value, self.id2_center_min)
        if self.id2_center_max is not None:
            value = min(value, self.id2_center_max)
        return value

    def _id2_center_limit_blocks(self, delta_id2, target_id2):
        if delta_id2 == 0 or target_id2 != self.id2:
            return False
        if delta_id2 > 0 and self.id2_center_max is not None:
            return self.id2 >= self.id2_center_max
        if delta_id2 < 0 and self.id2_center_min is not None:
            return self.id2 <= self.id2_center_min
        return False

    @staticmethod
    def _target_center(target):
        if target is None or "center" not in target:
            return None
        return target["center"]

    @staticmethod
    def _target_error(target, frame_shape):
        height, width = frame_shape[:2]
        cx, cy = target["center"]
        return cx - width / 2.0, cy - height / 2.0

    def _record_center_distance_sample(self, distance_cm):
        if distance_cm is None:
            return None
        try:
            sample = float(distance_cm)
        except (TypeError, ValueError):
            return None
        self.center_distance_samples.append(sample)
        self.center_distance_samples = self.center_distance_samples[-8:]
        sorted_samples = sorted(self.center_distance_samples)
        mid = len(sorted_samples) // 2
        if len(sorted_samples) % 2:
            return sorted_samples[mid]
        return (sorted_samples[mid - 1] + sorted_samples[mid]) / 2.0

    def _accept_live_target(self, live_target):
        center = self._target_center(live_target)
        if center is None:
            self.visual_lost_frames += 1
            return None, "target lost"
        if not live_target.get("fully_visible", True):
            self.visual_lost_frames += 1
            return None, "target at edge"
        reference = self.last_visual_target or self.locked_target
        reference_center = self._target_center(reference)
        if reference_center is not None:
            jump_px = math.hypot(
                float(center[0]) - float(reference_center[0]),
                float(center[1]) - float(reference_center[1]),
            )
            if jump_px > self.target_jump_reset_px:
                self.visual_lost_frames += 1
                return None, f"target jump {jump_px:.0f}px"
        self.visual_lost_frames = 0
        self.last_visual_target = self._copy_target(live_target)
        return live_target, "target live"

    def _centering_target_hold_reason(self, target, frame_shape):
        center = self._target_center(target)
        if center is None:
            self.visual_lost_frames += 1
            if self.visual_lost_frames > 6:
                self.last_visual_target = None
            return "target lost before centering"
        error_x, error_y = self._target_error(target, frame_shape)
        if not target.get("fully_visible", True):
            self.visual_lost_frames += 1
            if self.visual_lost_frames > 6:
                self.last_visual_target = None
            return (
                f"target at edge; hold centering dx={error_x:.0f} dy={error_y:.0f}"
            )
        reference_center = self._target_center(self.last_visual_target)
        if reference_center is not None:
            jump_px = math.hypot(
                float(center[0]) - float(reference_center[0]),
                float(center[1]) - float(reference_center[1]),
            )
            if jump_px > self.centering_jump_reset_px:
                self.centered_frames = 0
                self.visual_lost_frames += 1
                if self.visual_lost_frames >= 3:
                    self.last_visual_target = self._copy_target(target)
                    self.visual_lost_frames = 0
                    return f"target jump {jump_px:.0f}px; rebase and hold"
                return f"target jump {jump_px:.0f}px; hold centering"
        self.visual_lost_frames = 0
        self.last_visual_target = self._copy_target(target)
        return None

    def _abort_to_standby(self, reason):
        self.locked_target = None
        self.locked_plan = None
        self.last_visual_target = None
        self.visual_confirm_frames = 0
        self.visual_lost_frames = 0
        self.visual_descend_start = None
        self.visual_descend_step_index = 0
        self.post_center_move_complete = False
        self.center_distance_samples = []
        self.approach_attempts = 0
        self.return_attempts = 0
        self.next_stage_after_open = None
        self.next_stage_after_retreat = None
        self.motion_stage_after_wait = None
        self.abort_after_return = True
        self.algorithm_stage = "return"
        self.state = "abort to standby"
        self.status = f"{reason}; aborting to standby"
        self.arm_preview.publish(self.status)
        return self.status

    def _visual_center_step(self, target, frame_shape, now, can_preview_step, label):
        error_x, error_y = self._target_error(target, frame_shape)
        if self.active_chassis_station == "COLUMN_CATCH":
            if (
                abs(error_x) <= COLUMN_CATCH_CENTER_DEADBAND_PX
                and abs(error_y) <= COLUMN_CATCH_CENTER_DEADBAND_PX
            ):
                return True, (
                    f"{label} centered dx={error_x:.0f} dy={error_y:.0f} "
                    f"ID2={self.id2} ID6={self.id6}"
                )

            target_id2 = self.id2
            target_id6 = self.id6
            if abs(error_x) > COLUMN_CATCH_CENTER_DEADBAND_PX:
                target_id6 += (
                    -COLUMN_CATCH_ID6_CENTER_STEP_TICKS
                    if error_x > 0.0
                    else COLUMN_CATCH_ID6_CENTER_STEP_TICKS
                )
            if abs(error_y) > COLUMN_CATCH_CENTER_DEADBAND_PX:
                target_id2 += (
                    -COLUMN_CATCH_ID2_CENTER_STEP_TICKS
                    if error_y > 0.0
                    else COLUMN_CATCH_ID2_CENTER_STEP_TICKS
                )
            target_id2 = max(
                COLUMN_CATCH_ID2_CENTER_RANGE[0],
                min(COLUMN_CATCH_ID2_CENTER_RANGE[1], target_id2),
            )
            target_id6 = max(
                COLUMN_CATCH_ID6_CENTER_RANGE[0],
                min(COLUMN_CATCH_ID6_CENTER_RANGE[1], target_id6),
            )
            if target_id2 == self.id2 and target_id6 == self.id6:
                return False, (
                    f"{label} blocked by limits dx={error_x:.0f} dy={error_y:.0f} "
                    f"ID2={self.id2} ID6={self.id6}"
                )

            target_id1, target_id2 = enforce_angle_gap(
                self.id1,
                target_id2,
                self.id2_limits,
                self.angle_gap_degrees,
            )
            if not self.servo_bridge.write_enabled:
                if can_preview_step:
                    self.id2 = target_id2
                    self.id6 = target_id6
                    self.last_preview_step_time = now
                self.arm_preview.set_targets(
                    target_id1, target_id2, self.id7, target_id6
                )
                return False, (
                    f"preview {label} dx={error_x:.0f} dy={error_y:.0f} "
                    f"ID2={target_id2} ID6={target_id6} step=7/5"
                )

            can_command = now - self.last_command_time >= (
                PLATFORM_CENTER_TIME_MS / 1000.0
            )
            if not can_command:
                self.arm_preview.set_targets(
                    self.id1, self.id2, self.id7, self.id6
                )
                return False, f"waiting {label} dx={error_x:.0f} dy={error_y:.0f}"

            previous_id2 = self.id2
            previous_id6 = self.id6
            self.id2 = target_id2
            self.id6 = target_id6
            return False, self._column_center_correction(
                f"{label} dx={error_x:.0f} dy={error_y:.0f} "
                f"dID2={self.id2 - previous_id2} "
                f"dID6={self.id6 - previous_id6} step=7/5",
                send_id2=self.id2 != previous_id2,
                send_id6=self.id6 != previous_id6,
            )

        centering_profile = self._centering_profile()
        delta_id2, delta_id6 = self._vector_centering_raw_deltas(
            error_x,
            error_y,
            centering_profile["id2_gain"],
            centering_profile["id6_gain"],
            centering_profile["id2_max_step_ticks"],
            centering_profile["id6_max_step_ticks"],
        )
        target_id2 = self._clamp_center_id2(self.id2 + delta_id2)
        id2_center_limited = self._id2_center_limit_blocks(delta_id2, target_id2)
        if id2_center_limited:
            delta_id2 = 0
            if delta_id6 == 0:
                return True, (
                    f"{label} centered at ID2 limit dx={error_x:.0f} dy={error_y:.0f} "
                    f"ID2={self.id2} ID6={self.id6}"
                )
        delta_id2, delta_id6, axis_note = self._vector_centering_deltas(
            delta_id2,
            delta_id6,
            id2_center_limited,
            error_x,
            error_y,
            centering_profile["id2_max_step_ticks"],
            centering_profile["id6_max_step_ticks"],
        )
        target_id2 = self._clamp_center_id2(self.id2 + delta_id2)
        if delta_id2 == 0 and delta_id6 == 0:
            return True, f"{label} centered dx={error_x:.0f} dy={error_y:.0f}"

        id6_limits = (
            TASK23_ID6_CENTER_LIMITS
            if self.active_chassis_station in {"PLATFORM_PICK", "COLUMN_CATCH"}
            else ID6_SAFE_LIMITS
        )
        if delta_id6:
            delta_id6 = (
                TASK23_ID6_CENTER_STEP_TICKS
                if delta_id6 > 0 else -TASK23_ID6_CENTER_STEP_TICKS
            )
        target_id6 = self._clamp(self.id6 + delta_id6, id6_limits)
        target_id1 = self.id1
        target_id1, target_id2 = enforce_angle_gap(
            target_id1,
            target_id2,
            self.id2_limits,
            self.angle_gap_degrees,
        )
        if target_id2 == self.id2 and target_id6 == self.id6:
            return False, (
                f"{label} blocked by limits dx={error_x:.0f} dy={error_y:.0f} "
                f"ID2={self.id2} ID6={self.id6}"
            )

        can_command = now - self.last_command_time >= centering_profile["command_interval_s"]
        if not self.servo_bridge.write_enabled:
            if can_preview_step:
                self.id2 = target_id2
                self.id6 = target_id6
                self.last_preview_step_time = now
            self.arm_preview.set_targets(target_id1, target_id2, self.id7, target_id6)
            return False, (
                f"preview {label} dx={error_x:.0f} dy={error_y:.0f} "
                f"ID2={target_id2} ID6={target_id6}"
            )
        if not can_command:
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            return False, f"waiting {label} dx={error_x:.0f} dy={error_y:.0f}"

        previous_id2 = self.id2
        previous_id6 = self.id6
        self.id2 = target_id2
        self.id6 = target_id6
        send_id2 = self.id2 != previous_id2
        send_id6 = self.id6 != previous_id6
        return False, self._send_center_correction(
            f"{label} dx={error_x:.0f} dy={error_y:.0f} "
            f"axis={axis_note} dID2={self.id2 - previous_id2} "
            f"dID6={self.id6 - previous_id6} {centering_profile['note']}",
            send_id2=send_id2,
            send_id6=send_id6,
        )

    def _centering_profile(self):
        correction_index = max(0, int(self.centering_correction_count))
        decay = 0.5 ** (correction_index / 4.0)
        return {
            "index": correction_index + 1,
            "decay": decay,
            "command_interval_s": min(
                CENTERING_SLOW_COMMAND_INTERVAL_S,
                self.command_interval_s / max(decay, 0.001),
            ),
            "id2_gain": max(CENTERING_SLOW_ID2_GAIN, self.id2_pixel_gain * decay),
            "id6_gain": max(CENTERING_SLOW_ID6_GAIN, self.id6_pixel_gain * decay),
            "id2_max_step_ticks": max(
                CENTERING_SLOW_ID2_MAX_STEP_TICKS,
                int(round(self.max_step_ticks * decay)),
            ),
            "id6_max_step_ticks": max(
                CENTERING_SLOW_ID6_MAX_STEP_TICKS,
                int(round(self.id6_max_step_ticks * decay)),
            ),
            "note": (
                f"centerStep={correction_index + 1} decay={decay:.2f}"
            ),
        }

    def _centering_delta(self, error_px, gain, max_step_ticks=None):
        if abs(error_px) <= self.center_deadband_px:
            return 0
        if max_step_ticks is None:
            max_step_ticks = self.max_step_ticks
        delta = int(round(error_px * gain))
        delta = max(-max_step_ticks, min(max_step_ticks, delta))
        minimum_step = min(4, max_step_ticks)
        if abs(delta) < minimum_step:
            return minimum_step if error_px > 0 else -minimum_step
        return delta

    def _id6_centering_delta(self, error_x_px, gain=None, max_step_ticks=None):
        if abs(error_x_px) <= self.center_deadband_px:
            return 0
        if gain is None:
            gain = self.id6_pixel_gain
        if max_step_ticks is None:
            max_step_ticks = self.id6_max_step_ticks
        delta = int(round(-error_x_px * gain))
        delta = max(-max_step_ticks, min(max_step_ticks, delta))
        minimum_step = min(SINGLE_ID6_MIN_STEP_TICKS, max_step_ticks)
        if abs(delta) < minimum_step:
            return -minimum_step if error_x_px > 0 else minimum_step
        return delta

    def _component_centering_delta(self, error_px, gain, max_step_ticks, reverse=False):
        if abs(error_px) <= CENTERING_VECTOR_COMPONENT_DEADBAND_PX:
            return 0
        signed_error = -error_px if reverse else error_px
        delta = int(round(signed_error * gain))
        delta = max(-max_step_ticks, min(max_step_ticks, delta))
        minimum_step = min(3, max_step_ticks)
        if abs(delta) < minimum_step:
            return minimum_step if signed_error > 0 else -minimum_step
        return delta

    def _vector_centering_raw_deltas(
        self,
        error_x_px,
        error_y_px,
        id2_gain,
        id6_gain,
        id2_max_step_ticks,
        id6_max_step_ticks,
    ):
        distance_px = math.hypot(float(error_x_px), float(error_y_px))
        if distance_px <= self.center_deadband_px:
            return 0, 0
        delta_id2 = self._component_centering_delta(
            -error_y_px,
            id2_gain,
            id2_max_step_ticks,
        )
        delta_id6 = self._component_centering_delta(
            error_x_px,
            id6_gain,
            id6_max_step_ticks,
            reverse=True,
        )
        return delta_id2, delta_id6

    def _vector_centering_deltas(
        self,
        delta_id2,
        delta_id6,
        id2_center_limited,
        error_x_px=None,
        error_y_px=None,
        id2_max_step_ticks=None,
        id6_max_step_ticks=None,
    ):
        if id2_max_step_ticks is None:
            id2_max_step_ticks = self.max_step_ticks
        if id6_max_step_ticks is None:
            id6_max_step_ticks = self.id6_max_step_ticks
        if id2_center_limited:
            delta_id2 = 0
        if delta_id2 == 0 and delta_id6 == 0:
            return 0, 0, "none"

        distance_px = None
        if error_x_px is not None and error_y_px is not None:
            distance_px = math.hypot(float(error_x_px), float(error_y_px))

        if distance_px and delta_id2 != 0 and delta_id6 != 0:
            biggest_component = max(abs(float(error_x_px)), abs(float(error_y_px)), 1.0)
            diagonal_boost = min(1.35, max(1.0, distance_px / biggest_component))
            delta_id2 = int(round(delta_id2 * diagonal_boost))
            delta_id6 = int(round(delta_id6 * diagonal_boost))
            delta_id2 = max(-id2_max_step_ticks, min(id2_max_step_ticks, delta_id2))
            delta_id6 = max(
                -id6_max_step_ticks,
                min(id6_max_step_ticks, delta_id6),
            )
            return delta_id2, delta_id6, f"ID2+ID6 vector r={distance_px:.0f}px"

        if delta_id2 != 0:
            return delta_id2, 0, "ID2"
        axis = "ID6 after ID2 limit" if id2_center_limited else "ID6"
        return 0, delta_id6, axis

    def _single_id2_correction(self, error_px):
        if abs(error_px) <= SINGLE_ID2_DEADBAND_PX:
            return 0
        delta = int(round(error_px * self.id2_pixel_gain))
        delta = max(-SINGLE_ID2_MAX_STEP_TICKS, min(SINGLE_ID2_MAX_STEP_TICKS, delta))
        if abs(delta) < SINGLE_ID2_MIN_STEP_TICKS:
            return SINGLE_ID2_MIN_STEP_TICKS if error_px > 0 else -SINGLE_ID2_MIN_STEP_TICKS
        return delta

    @staticmethod
    def _id1_degrees(id1_tick):
        return id1_degrees(id1_tick)

    @staticmethod
    def _id2_degrees(id2_tick):
        return id2_degrees(id2_tick)

    @staticmethod
    def _id2_tick_from_degrees(degrees):
        return id2_tick_from_degrees(degrees)

    def _enforce_angle_gap(self):
        self.id1, self.id2 = enforce_angle_gap(
            self.id1,
            self.id2,
            self.id2_limits,
            self.angle_gap_degrees,
        )
        return angle_gap_degrees(self.id1, self.id2)

    def _apply_feedback(self, feedback):
        if not (self.id1_limits[0] <= feedback[0] <= self.id1_limits[1]):
            self.state = "fault"
            self.status = f"ID1 feedback outside safe range: {feedback[0]}"
            return False
        if not (self.id2_limits[0] <= feedback[1] <= self.id2_limits[1]):
            self.state = "fault"
            self.status = f"ID2 feedback outside safe range: {feedback[1]}"
            return False
        self.id1 = int(feedback[0])
        self.id2 = int(feedback[1])
        id6 = feedback[2] if len(feedback) > 2 else None
        if id6 is not None:
            self.id6 = int(id6)
        gap = angle_gap_degrees(self.id1, self.id2)
        if self.angle_gap_degrees > 0.0 and gap <= self.angle_gap_degrees:
            self.state = "fault"
            self.status = f"unsafe measured angle gap {gap:.1f}deg"
            return False
        self.synchronized = True
        self.feedback_pending = False
        self.feedback_failures = 0
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish("measured servo feedback")
        return True

    def _read_only_feedback_worker(self):
        feedback = self.servo_bridge.query_positions(timeout_s=0.45)
        with self.read_only_feedback_lock:
            self.read_only_feedback = feedback
            self.read_only_feedback_ready = True

    def _start_read_only_feedback(self):
        with self.read_only_feedback_lock:
            if (
                self.read_only_feedback_thread is not None
                and self.read_only_feedback_thread.is_alive()
            ):
                return
            self.read_only_feedback_ready = False
            self.read_only_feedback_thread = threading.Thread(
                target=self._read_only_feedback_worker,
                daemon=True,
            )
            self.read_only_feedback_thread.start()

    def _send(self, reason, require_feedback=True):
        gap = self._enforce_angle_gap()
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(reason)
        print(
            f"GRASP COMMAND reason={reason} ID1={self.id1} "
            f"ID2={self.id2} ID6={self.id6} ID7={self.id7} gap={gap:.1f}deg",
            flush=True,
        )
        self.status = self.servo_bridge.send_targets(
            id1=self.id1,
            id2=self.id2,
            id4=self.id7,
            id6=self.id6,
        )
        self.last_command_time = time.monotonic()
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.feedback_pending = False
            self.status = f"automatic motion stopped: {self.servo_bridge.status}"
            self.arm_preview.publish(self.status)
            return self.status
        if self.servo_bridge.write_enabled and require_feedback:
            if getattr(self.servo_bridge, "assumed_feedback", False):
                self.feedback_pending = False
            else:
                self.feedback_pending = True
                self.feedback_due = self.last_command_time + max(1.35, self.command_interval_s)
        gap_text = "" if gap is None else f" gap={gap:.1f}deg"
        return f"{reason}: id1={self.id1} id2={self.id2} id6={self.id6} id7={self.id7}{gap_text} | {self.status}"

    def _send_fixed_arm_pose_staged(
        self,
        target_id1,
        target_id2,
        target_id6,
        reason,
        *,
        raising,
        id7=None,
        id5=None,
        splitter_id4=None,
        aux_time_ms=None,
        splitter_time_ms=None,
        joint_sequence_delay_s=None,
        first_arm_time_ms=None,
        second_arm_time_ms=None,
    ):
        """Move fixed poses without letting ID1 and ID2 load each other."""

        target_id1 = self._clamp(target_id1, self.id1_limits)
        target_id2 = self._clamp(target_id2, self.id2_limits)
        target_id1, target_id2 = enforce_angle_gap(
            target_id1,
            target_id2,
            self.id2_limits,
            self.angle_gap_degrees,
        )
        target_id6 = max(HTD85_MIN_POSITION, min(HTD85_MAX_POSITION, int(target_id6)))
        target_id7 = self.id7 if id7 is None else int(id7)
        target_id5 = self.id5 if id5 is None else int(id5)
        target_splitter = (
            self.splitter_id4 if splitter_id4 is None else int(splitter_id4)
        )
        sequence_delay_s = (
            ARM_JOINT_SEQUENCE_DELAY_S
            if joint_sequence_delay_s is None
            else max(0.0, float(joint_sequence_delay_s))
        )

        if not self.servo_bridge.write_enabled:
            self.id1 = target_id1
            self.id2 = target_id2
            self.id6 = target_id6
            self.id7 = target_id7
            self.id5 = target_id5
            self.splitter_id4 = target_splitter
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            return (
                f"preview {reason} staged={'ID1->ID2' if raising else 'ID2->ID1'} "
                f"ID1={self.id1} ID2={self.id2} ID6={self.id6}"
            )

        if raising:
            self.id1 = target_id1
            self.id6 = target_id6
            first_targets = {
                "id1": self.id1,
                "id6": self.id6,
                "id4": target_id7,
                "id5": target_id5,
                "splitter_id4": target_splitter,
            }
            second_targets = {"id2": target_id2}
            first_label, second_label = "ID1", "ID2"
        else:
            self.id2 = target_id2
            first_targets = {
                "id2": self.id2,
                "id4": target_id7,
                "id5": target_id5,
                "splitter_id4": target_splitter,
            }
            second_targets = {"id1": target_id1, "id6": target_id6}
            first_label, second_label = "ID2", "ID1"

        if aux_time_ms is not None:
            first_targets["aux_time_ms"] = int(aux_time_ms)
        if splitter_time_ms is not None:
            first_targets["splitter_time_ms"] = int(splitter_time_ms)

        self.id7 = target_id7
        self.id5 = target_id5
        self.splitter_id4 = target_splitter
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(f"{reason} phase 1 {first_label}")
        print(
            f"ARM STAGED reason={reason} phase=1 joint={first_label} "
            f"time_ms={int(first_arm_time_ms or self.servo_bridge.arm_time_ms)} "
            f"delay_ms={int(sequence_delay_s * 1000)}",
            flush=True,
        )
        previous_arm_time = getattr(
            self.servo_bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS
        )
        try:
            if first_arm_time_ms is not None:
                self.servo_bridge.arm_time_ms = int(first_arm_time_ms)
            first_status = self.servo_bridge.send_targets(**first_targets)
            self.last_command_time = time.monotonic()
            if not self.servo_bridge.last_command_ok:
                self.status = f"{reason} phase 1 {first_label} failed: {first_status}"
                return self.status

            time.sleep(sequence_delay_s)
            self.id1 = target_id1
            self.id2 = target_id2
            self.id6 = target_id6
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            self.arm_preview.publish(f"{reason} phase 2 {second_label}")
            if second_arm_time_ms is not None:
                self.servo_bridge.arm_time_ms = int(second_arm_time_ms)
            print(
                f"ARM STAGED reason={reason} phase=2 joint={second_label} "
                f"time_ms={int(second_arm_time_ms or previous_arm_time)} "
                f"after_ms={int(sequence_delay_s * 1000)}",
                flush=True,
            )
            second_status = self.servo_bridge.send_targets(**second_targets)
            self.last_command_time = time.monotonic()
            if not self.servo_bridge.last_command_ok:
                self.status = f"{reason} phase 2 {second_label} failed: {second_status}"
                return self.status
        finally:
            self.servo_bridge.arm_time_ms = previous_arm_time

        self.status = (
            f"{reason} staged={first_label}->{second_label} "
            f"delay_ms={int(ARM_JOINT_SEQUENCE_DELAY_S * 1000)} "
            f"ID1={self.id1} ID2={self.id2} ID6={self.id6}; "
            f"phase1={first_status}; phase2={second_status}"
        )
        return self.status

    def _send_retreat_with_catcher_open(self, reason, require_feedback=True):
        gap = self._enforce_angle_gap()
        self.id5 = CATCHER_RELEASE_READY_TICK
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(reason)
        print(
            f"GRASP COMMAND reason={reason} ID1={self.id1} "
            f"ID2={self.id2} ID6={self.id6} ID5={self.id5} "
            f"ID7={self.id7} gap={gap:.1f}deg",
            flush=True,
        )
        self.status = self.servo_bridge.send_targets(
            id1=self.id1,
            id2=self.id2,
            id6=self.id6,
            id5=self.id5,
        )
        self.last_command_time = time.monotonic()
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.feedback_pending = False
            self.status = f"automatic motion stopped: {self.servo_bridge.status}"
            self.arm_preview.publish(self.status)
            return self.status
        if self.servo_bridge.write_enabled and require_feedback:
            if getattr(self.servo_bridge, "assumed_feedback", False):
                self.feedback_pending = False
            else:
                self.feedback_pending = True
                self.feedback_due = self.last_command_time + max(1.35, self.command_interval_s)
        gap_text = "" if gap is None else f" gap={gap:.1f}deg"
        return (
            f"{reason}: id1={self.id1} id2={self.id2} id5={self.id5} "
            f"id6={self.id6} id7={self.id7}{gap_text} | {self.status}"
        )

    def _send_center_correction(self, reason, send_id2, send_id6, require_feedback=False):
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(reason)
        print(
            f"CENTER COMMAND reason={reason} ID2={self.id2} ID6={self.id6}",
            flush=True,
        )
        self.status = self.servo_bridge.send_targets(
            id2=self.id2 if send_id2 else None,
            id6=self.id6 if send_id6 else None,
        )
        self.last_command_time = time.monotonic()
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.feedback_pending = False
            self.status = f"automatic motion stopped: {self.servo_bridge.status}"
            self.arm_preview.publish(self.status)
            return self.status
        if send_id2 or send_id6:
            self.centering_correction_count += 1
        if self.servo_bridge.write_enabled and require_feedback:
            if getattr(self.servo_bridge, "assumed_feedback", False):
                self.feedback_pending = False
            else:
                self.feedback_pending = True
                self.feedback_due = self.last_command_time + max(1.35, self.command_interval_s)
        return f"{reason}: id2={self.id2} id6={self.id6} | {self.status}"

    # The logical gripper state is sent to physical HTD85 ID17.
    def _send_gripper_id7(self, target, reason, motion_ms=None):
        previous_id7 = self.id7
        target = int(target)
        self.arm_preview.set_targets(self.id1, self.id2, target, self.id6)
        self.arm_preview.publish(reason)
        print(
            f"GRASP COMMAND reason={reason} ID1={self.id1} "
            f"ID2={self.id2} ID6={self.id6} ID7={previous_id7}->{target}",
            flush=True,
        )
        previous_gripper_time = getattr(
            self.servo_bridge, "gripper_time_ms", None
        )
        try:
            if motion_ms is not None:
                self.servo_bridge.gripper_time_ms = int(motion_ms)
            self.status = self.servo_bridge.send_targets(id4=target)
        finally:
            if motion_ms is not None and previous_gripper_time is not None:
                self.servo_bridge.gripper_time_ms = previous_gripper_time
        self.last_command_time = time.monotonic()
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            self.id7 = previous_id7
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.feedback_pending = False
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            self.status = f"automatic motion stopped: {self.servo_bridge.status}"
            self.arm_preview.publish(self.status)
            return self.status
        self.id7 = target
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        return f"{reason}: id7={self.id7} | {self.status}"

    def _send_id5(self, target, reason, critical=True):
        previous_id5 = self.id5
        target = int(target)
        print(
            f"AUX COMMAND reason={reason} ID5={previous_id5}->{target}",
            flush=True,
        )
        self.status = self.servo_bridge.send_targets(id5=target)
        self.last_command_time = time.monotonic()
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            self.id5 = previous_id5
            if critical:
                self.state = "fault"
                self.algorithm_stage = "fault"
                self.feedback_pending = False
                self.status = f"automatic motion stopped: {self.servo_bridge.status}"
                self.arm_preview.publish(self.status)
            return self.status
        self.id5 = target
        return f"{reason}: id5={self.id5} | {self.status}"

    def _send_splitter_id4(self, target, reason, splitter_time_ms=None):
        previous = self.splitter_id4
        target = int(target)
        print(
            f"AUX COMMAND reason={reason} ID4={previous}->{target}",
            flush=True,
        )
        send_kwargs = {"splitter_id4": target}
        if splitter_time_ms is not None:
            send_kwargs["splitter_time_ms"] = int(splitter_time_ms)
        self.status = self.servo_bridge.send_targets(**send_kwargs)
        self.last_aux_command_time = time.monotonic()
        if self.servo_bridge.write_enabled:
            if self.servo_bridge.last_command_ok:
                self.splitter_id4 = target
            else:
                self.state = "fault"
                self.algorithm_stage = "fault"
                self.feedback_pending = False
                self.status = f"automatic motion stopped: {self.servo_bridge.status}"
                self.arm_preview.publish(self.status)
        return f"{reason}: id4={self.splitter_id4} | {self.status}"

    def _send_disc_open_phase(self, splitter_target, catcher_target, reason):
        previous_splitter = self.splitter_id4
        previous_id5 = self.id5
        previous_id7 = self.id7
        splitter_target = int(splitter_target)
        catcher_target = int(catcher_target)
        self.arm_preview.set_targets(
            self.id1, self.id2, DISC_CATCH_GRIPPER_OPEN_TICK, self.id6
        )
        self.arm_preview.publish(reason)
        print(
            f"DISC OPEN reason={reason} ID1={self.id1} ID2={self.id2} ID6={self.id6} "
            f"ID14={previous_splitter}->{splitter_target} "
            f"ID15={previous_id5}->{catcher_target} "
            f"ID17={previous_id7}->{DISC_CATCH_GRIPPER_OPEN_TICK}",
            flush=True,
        )
        previous_gripper_time = getattr(
            self.servo_bridge, "gripper_time_ms", None
        )
        try:
            self.servo_bridge.gripper_time_ms = (
                DISC_CATCH_GRIPPER_TIME_MS
                if self.disc_fast_blue_cycle
                else DISC_CATCH_NORMAL_GRIPPER_TIME_MS
            )
            self.status = self.servo_bridge.send_targets(
                splitter_id4=splitter_target,
                id5=catcher_target,
                id4=DISC_CATCH_GRIPPER_OPEN_TICK,
                aux_time_ms=TASK1_AUX_TIME_MS,
                splitter_time_ms=TASK1_ID14_TIME_MS,
            )
        finally:
            if previous_gripper_time is not None:
                self.servo_bridge.gripper_time_ms = previous_gripper_time
        self.last_command_time = time.monotonic()
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            self.splitter_id4 = previous_splitter
            self.id5 = previous_id5
            self.id7 = previous_id7
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.feedback_pending = False
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            self.status = f"automatic motion stopped: {self.servo_bridge.status}"
            self.arm_preview.publish(self.status)
            return self.status
        self.splitter_id4 = splitter_target
        self.id5 = catcher_target
        self.id7 = DISC_CATCH_GRIPPER_OPEN_TICK
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        return (
            f"{reason}: ID14={self.splitter_id4} ID15={self.id5} "
            f"ID17={self.id7} | {self.status}"
        )

    def update_auxiliary(self, detections):
        if not self.enabled or not self.servo_bridge.write_enabled:
            return ""
        now = time.monotonic()
        if now - self.last_aux_command_time < self.aux_command_interval_s:
            return ""
        balls = [det for det in detections if det.get("kind") == "ball"]
        if any(det.get("color") == "yellow" for det in balls):
            target = SPLITTER_RETRACT_TICK
            reason = "yellow ball detected; ID14 neutral"
        elif balls:
            target = SPLITTER_RETRACT_TICK
            reason = "ball detected; ID14 remains retracted outside DISC_CATCH"
        else:
            return ""
        if target == self.splitter_id4:
            return ""
        return self._send_splitter_id4(target, reason)

    def _reset_cycle_for_search(self, status, start_retrigger_cooldown=False):
        if (
            start_retrigger_cooldown
            and self.active_chassis_station is not None
            and self.chassis_station_done_reason is None
        ):
            self.chassis_station_done_reason = "GRASP_DONE"
            self.active_chassis_station = None
        self.locked_target = None
        self.locked_plan = None
        self.last_visual_target = None
        self.visual_confirm_frames = 0
        self.visual_lost_frames = 0
        self.visual_descend_start = None
        self.visual_descend_step_index = 0
        self.centered_frames = 0
        self.center_distance_samples = []
        self.horizontal_correction_done = False
        self.centering_correction_count = 0
        self.post_center_move_complete = False
        self.approach_attempts = 0
        self.return_attempts = 0
        self.next_stage_after_open = None
        self.next_stage_after_retreat = None
        self.motion_stage_after_wait = None
        self.abort_after_return = False
        self.algorithm_stage = "centering"
        if start_retrigger_cooldown and self.retrigger_cooldown_s > 0.0:
            self.ignore_new_targets_until = (
                time.monotonic() + self.retrigger_cooldown_s
            )
        self.state = "searching"
        self.status = status
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(status)
        return status

    def startup_complete(self):
        return self.startup_stage == "complete"

    def ready_for_chassis_link(self, execute_auto_grasp):
        """READY means both the controller and the writable servo link are up."""

        command_link_ready = getattr(
            self.servo_bridge,
            "ready_for_commands",
            lambda: (
                self.servo_bridge.enabled
                and self.servo_bridge.write_enabled
                and self.servo_bridge.last_command_ok
            ),
        )
        return (
            self.startup_complete()
            and (
                not execute_auto_grasp
                or command_link_ready()
            )
        )

    def busy_for_chassis(self):
        return (
            self.startup_stage != "complete"
            or self.feedback_pending
            or self.chassis_station_stage is not None
            or self.locked_target is not None
            or self.algorithm_stage != "centering"
        )

    def searching_for_chassis_target(self):
        return (
            self.startup_stage == "complete"
            and self.active_chassis_station == "PLATFORM_PICK"
            and self.chassis_station_stage is None
            and self.locked_target is None
            and self.algorithm_stage == "centering"
            # A first sighting starts the three-frame vote, but the station
            # remains in the bounded search path until target_key is locked.
            and self.platform_task.target_key is None
        )

    def begin_chassis_station(self, station):
        if station == "PLATFORM_PICK":
            self._reset_cycle_for_search("platform slot start")
            self.active_chassis_station = station
            self.chassis_station_done_reason = self.chassis_station_error_reason = None
            self.platform_task.begin_slot(self.field_mode.value)
            return self._sync_platform_task()
        self.platform_task.reset()
        self._reset_cycle_for_search(f"chassis station {station} start")
        if self.startup_stage != "complete":
            return f"chassis station {station} queued until startup completes"
        self.platform_high_hold = False
        self.platform_high_pose_sent = False
        self.active_chassis_station = station
        self.chassis_station_done_reason = None
        self.chassis_station_error_reason = None
        if station == "DISC_CATCH":
            return self._begin_disc_catch_station()
        if station == "COLUMN_CATCH":
            return self._begin_column_catch_station()
        if station == "TASK3_RING_PLACE":
            return self._begin_task3_ring_place_station()
        if station == "PLATFORM_PICK":
            # A slot START is the explicit boundary after H7 white-line
            # alignment; keep the arm high until that boundary arrives.
            self.chassis_station_stage = None
            self.id1, self.id2, self.id6 = PLATFORM_HIGH_POSE
            self.id5 = CATCHER_HOME_TICK
            self.splitter_id4 = SPLITTER_RETRACT_TICK
        else:
            self.id1 = READY_ID1_TICK
            self.id2 = READY_ID2_TICK
            self.id6 = BASE_YAW_CENTER_TICK
            self.id5 = CATCHER_HOME_TICK
            self.splitter_id4 = SPLITTER_RETRACT_TICK
        self.id7 = self.id7_closed
        self._enforce_angle_gap()
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(f"chassis station {station} ready")
        if not self.servo_bridge.write_enabled:
            return (
                f"preview chassis station {station} ready "
                f"ID1={self.id1} ID2={self.id2} ID6={self.id6} ID7={self.id7}"
            )
        status = self._send_fixed_arm_pose_staged(
            self.id1,
            self.id2,
            self.id6,
            f"chassis station {station} ready",
            raising=True,
            id7=self.id7,
            id5=self.id5,
            splitter_id4=self.splitter_id4,
        )
        if not self.servo_bridge.last_command_ok:
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.status = f"chassis station {station} ready failed: {status}"
            self.arm_preview.publish(self.status)
            return self.status
        self.status = (
            f"chassis station {station} ready ID1={self.id1} ID2={self.id2} "
            f"ID4={self.splitter_id4} ID5={self.id5} "
            f"ID6={self.id6} ID7={self.id7} | {status}"
        )
        print(f"CHASSIS STATION {station} READY {self.status}", flush=True)
        return self.status

    def _platform_pose(self, pose, raising):
        bridge = self.servo_bridge
        previous_times = (
            getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS),
            getattr(bridge, "aux_time_ms", PLATFORM_AUX_TIME_MS),
            getattr(bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS),
            getattr(bridge, "splitter_time_ms", PLATFORM_AUX_TIME_MS),
        )
        try:
            bridge.arm_time_ms = PLATFORM_ARM_TIME_MS
            bridge.aux_time_ms = PLATFORM_AUX_TIME_MS
            bridge.gripper_time_ms = PLATFORM_GRIPPER_TIME_MS
            bridge.splitter_time_ms = PLATFORM_AUX_TIME_MS
            self._send_fixed_arm_pose_staged(
                *pose, "PLATFORM_PICK pose", raising=raising,
                id7=self.id7, id5=HTD85_AUX_HIGH[1],
                splitter_id4=HTD85_AUX_HIGH[0],
                joint_sequence_delay_s=0.10,
            )
        finally:
            (
                bridge.arm_time_ms,
                bridge.aux_time_ms,
                bridge.gripper_time_ms,
                bridge.splitter_time_ms,
            ) = previous_times
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            raise RuntimeError(self.servo_bridge.status)
        if (self.id1, self.id2, self.id6) != tuple(pose):
            raise RuntimeError("PLATFORM_POSE_CLAMPED")
        return self._arm_settle_s()

    def _platform_retreat_pose(self, pose, raising):
        """Use the restored standard arm duration for the existing ID2 retreat action."""
        bridge = self.servo_bridge
        previous_times = (
            getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS),
            getattr(bridge, "aux_time_ms", PLATFORM_AUX_TIME_MS),
            getattr(bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS),
            getattr(bridge, "splitter_time_ms", PLATFORM_AUX_TIME_MS),
        )
        try:
            bridge.arm_time_ms = PLATFORM_RETREAT_TIME_MS
            bridge.aux_time_ms = PLATFORM_AUX_TIME_MS
            bridge.gripper_time_ms = PLATFORM_GRIPPER_TIME_MS
            bridge.splitter_time_ms = PLATFORM_AUX_TIME_MS
            self._send_fixed_arm_pose_staged(
                *pose, "PLATFORM_PICK ID2 retreat", raising=raising,
                id7=self.id7, id5=HTD85_AUX_HIGH[1],
                splitter_id4=HTD85_AUX_HIGH[0],
            )
        finally:
            (
                bridge.arm_time_ms,
                bridge.aux_time_ms,
                bridge.gripper_time_ms,
                bridge.splitter_time_ms,
            ) = previous_times
        return PLATFORM_RETREAT_TIME_MS / 1000.0 + 0.12

    def _platform_gripper(self, pulse):
        # Formal task two uses the standalone task-two gripper calibration.
        return self._platform_gripper_timed(pulse, PLATFORM_GRIPPER_TIME_MS)

    def _platform_gripper_timed(self, pulse, motion_ms):
        bridge = self.servo_bridge
        previous = getattr(bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS)
        try:
            bridge.gripper_time_ms = int(motion_ms)
            self.id7 = pulse
            if bridge.write_enabled:
                bridge.send_targets(id4=pulse)  # Legacy id4 keyword = physical ID7.
                if not bridge.last_command_ok:
                    raise RuntimeError(bridge.status)
        finally:
            bridge.gripper_time_ms = previous
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        return int(motion_ms) / 1000.0 + 0.05

    def _platform_gripper_open(self, pulse):
        return self._platform_gripper_timed(pulse, PLATFORM_GRIPPER_TIME_MS)

    def _platform_gripper_close(self, pulse):
        return self._platform_gripper_timed(pulse, PLATFORM_GRIPPER_CLOSE_TIME_MS)

    def _platform_place_open(self, pulse):
        return self._platform_gripper_timed(pulse, PLATFORM_PLACE_GRIPPER_TIME_MS)

    def _platform_place_close(self, pulse):
        return self._platform_gripper_timed(pulse, PLATFORM_PLACE_GRIPPER_TIME_MS)

    def _platform_center(self, id2, id6):
        bridge = self.servo_bridge
        previous = getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS)
        try:
            bridge.arm_time_ms = PLATFORM_CENTER_TIME_MS
            if bridge.write_enabled:
                bridge.send_targets(id2=id2, id6=id6)
                if not bridge.last_command_ok:
                    raise RuntimeError(bridge.status)
            self.id2, self.id6 = id2, id6
        finally:
            bridge.arm_time_ms = previous
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        return PLATFORM_CENTER_TIME_MS / 1000.0

    def _platform_ring_pair(self, servo_name_a, position_a, servo_name_b,
                            position_b, motion_ms, label):
        """Move the two placement joints in one HTD85 bus transaction."""

        bridge = self.servo_bridge
        previous = getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS)
        try:
            bridge.arm_time_ms = int(motion_ms)
            if bridge.write_enabled:
                bridge.send_targets(**{
                    servo_name_a: int(position_a),
                    servo_name_b: int(position_b),
                })
                if not bridge.last_command_ok:
                    raise RuntimeError(bridge.status)
            setattr(self, servo_name_a, int(position_a))
            setattr(self, servo_name_b, int(position_b))
        finally:
            bridge.arm_time_ms = previous
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        print(
            f"PLATFORM_PICK {label} {servo_name_a.upper()}={int(position_a)} "
            f"{servo_name_b.upper()}={int(position_b)} time={int(motion_ms)}ms",
            flush=True,
        )
        return max(0.10, float(motion_ms) / 1000.0)

    def _platform_ring_single(self, servo_name, position, motion_ms, label):
        """Move one ring-placement joint with an explicit bus duration."""

        bridge = self.servo_bridge
        previous = getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS)
        try:
            bridge.arm_time_ms = motion_ms
            if bridge.write_enabled:
                bridge.send_targets(**{servo_name: int(position)})
                if not bridge.last_command_ok:
                    raise RuntimeError(bridge.status)
            setattr(self, servo_name, int(position))
        finally:
            bridge.arm_time_ms = previous
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        print(
            f"PLATFORM_PICK {label} {servo_name.upper()}={int(position)} "
            f"time={motion_ms}ms",
            flush=True,
        )
        return motion_ms / 1000.0

    def _task3_ring_single(self, servo_name, position, motion_ms, label):
        """Move one joint for the deterministic formal blue-ring tail."""

        bridge = self.servo_bridge
        previous = getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS)
        try:
            bridge.arm_time_ms = int(motion_ms)
            if bridge.write_enabled:
                bridge.send_targets(**{servo_name: int(position)})
                if not bridge.last_command_ok:
                    raise RuntimeError(bridge.status)
            setattr(self, servo_name, int(position))
        finally:
            bridge.arm_time_ms = previous
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        print(
            f"TASK3_RING_PLACE {label} {servo_name.upper()}={int(position)} "
            f"time={int(motion_ms)}ms",
            flush=True,
        )
        return max(0.10, float(motion_ms) / 1000.0)

    def _task3_ring_release_pose(self, id1, id2, motion_ms, label):
        """Move the paired release-confirmation pose in one servo command."""

        bridge = self.servo_bridge
        previous = getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS)
        try:
            bridge.arm_time_ms = int(motion_ms)
            if bridge.write_enabled:
                bridge.send_targets(
                    id1=int(id1),
                    id2=int(id2),
                    id6=int(self.id6),
                    id4=int(self.id7),
                )
                if not bridge.last_command_ok:
                    raise RuntimeError(bridge.status)
            self.id1 = int(id1)
            self.id2 = int(id2)
        finally:
            bridge.arm_time_ms = previous
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        print(
            f"TASK3_RING_PLACE {label} ID1={int(id1)} ID2={int(id2)} "
            f"time={int(motion_ms)}ms",
            flush=True,
        )
        return max(0.10, float(motion_ms) / 1000.0)

    def _task3_ring_release_high(self, motion_ms, label):
        """Return all task-three end-ring joints to the station high pose."""

        bridge = self.servo_bridge
        previous = getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS)
        id1, id2, id6 = TASK3_RING_PLACE_HIGH
        try:
            bridge.arm_time_ms = int(motion_ms)
            if bridge.write_enabled:
                bridge.send_targets(
                    id1=int(id1),
                    id2=int(id2),
                    id6=int(id6),
                    id4=int(self.id7),
                )
                if not bridge.last_command_ok:
                    raise RuntimeError(bridge.status)
            self.id1 = int(id1)
            self.id2 = int(id2)
            self.id6 = int(id6)
        finally:
            bridge.arm_time_ms = previous
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        print(
            f"TASK3_RING_PLACE {label} ID1={self.id1} ID2={self.id2} "
            f"ID6={self.id6} time={int(motion_ms)}ms",
            flush=True,
        )
        return max(0.10, float(motion_ms) / 1000.0)

    def _task3_ring_gripper(self, position, motion_ms, label):
        """Move task-three ID17 with an explicit duration."""

        status = self._send_gripper_id7(
            position,
            f"TASK3_RING_PLACE {label}",
            motion_ms=motion_ms,
        )
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            return False
        return max(0.10, float(motion_ms) / 1000.0)

    def _task3_ring_wait(self, wait_ms, label):
        """Hold the closed gripper before the final arm contraction."""

        print(
            f"TASK3_RING_PLACE {label} wait={int(wait_ms)}ms",
            flush=True,
        )
        return max(0.10, float(wait_ms) / 1000.0)

    def _task3_ring_ordered_contract(self):
        """Retract the final ring arm in ID6 -> ID2 -> ID1 order."""

        bridge = self.servo_bridge
        previous_arm_time = getattr(
            bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS
        )
        try:
            bridge.arm_time_ms = TASK3_RING_PLACE_CONTRACT_AXIS_TIME_MS
            for servo_name, position in (
                ("id6", BASE_YAW_HOME_TICK),
                ("id2", HOME_ID2_TICK),
                ("id1", HOME_ID1_TICK),
            ):
                setattr(self, servo_name, int(position))
                self.arm_preview.set_targets(
                    self.id1, self.id2, self.id7, self.id6
                )
                print(
                    "TASK3_RING_PLACE CONTRACT "
                    f"{servo_name.upper()}={int(position)} "
                    f"time={TASK3_RING_PLACE_CONTRACT_AXIS_TIME_MS}ms",
                    flush=True,
                )
                status = bridge.send_targets(**{servo_name: int(position)})
                self.last_command_time = time.monotonic()
                if not bridge.last_command_ok:
                    return f"ordered contract {servo_name} failed: {status}"
                time.sleep(TASK3_RING_PLACE_CONTRACT_AXIS_TIME_MS / 1000.0)
        finally:
            bridge.arm_time_ms = previous_arm_time

        self.id5 = CATCHER_HOME_TICK
        self.splitter_id4 = SPLITTER_RETRACT_TICK
        self._enforce_angle_gap()
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        aux_status = bridge.send_targets(
            id3=TASK1_ID3_RETRACT_TICK,
            id4=self.id7,
            id5=self.id5,
            splitter_id4=self.splitter_id4,
        )
        self.last_command_time = time.monotonic()
        if not bridge.last_command_ok:
            return f"ordered contract auxiliary retraction failed: {aux_status}"
        return (
            "ordered contract ID6->ID2->ID1 complete; "
            f"aux={aux_status}"
        )

    def _platform_ring_place_id6(self, id6):
        return self._platform_ring_single(
            "id6", id6, PLATFORM_RING_AXIS_TIME_MS, "RING_PLACE_ID6"
        )

    def _platform_letter_place_id6(self, id6):
        return self._platform_ring_single(
            "id6", id6, PLATFORM_LETTER_PLACE_TIME_MS, "LETTER_PLACE_ID6"
        )

    def _platform_letter_place_id2(self, id2):
        return self._platform_ring_single(
            "id2", id2, PLATFORM_LETTER_PLACE_TIME_MS, "LETTER_PLACE_ID2"
        )

    def _platform_letter_place_id1(self, id1):
        return self._platform_ring_single(
            "id1", id1, PLATFORM_LETTER_PLACE_TIME_MS, "LETTER_PLACE_ID1"
        )

    def _platform_letter_place_id12(self, id1, id2):
        return self._platform_ring_pair(
            "id1", id1, "id2", id2, PLATFORM_LETTER_PLACE_TIME_MS,
            "LETTER_PLACE_ID1_ID2"
        )

    def _platform_ring_place_id2(self, id2):
        return self._platform_ring_single(
            "id2", id2, PLATFORM_RING_AXIS_TIME_MS, "RING_PLACE_ID2"
        )

    def _platform_ring_place_id1(self, id1):
        """Move ring-placement ID1 after ID6/ID2, independently."""
        return self._platform_ring_single(
            "id1", id1, PLATFORM_RING_PLACE_TIME_MS, "RING_PLACE_ID1"
        )

    def _platform_ring_place_id12(self, id1, id2):
        return self._platform_ring_pair(
            "id1", id1, "id2", id2, PLATFORM_RING_PLACE_TIME_MS,
            "RING_PLACE_ID1_ID2"
        )

    def _platform_ring_return_high_id1(self, id1):
        return self._platform_ring_single(
            "id1", id1, PLATFORM_RETURN_HIGH_TIME_MS, "RING_RETURN_HIGH_ID1"
        )

    def _platform_ring_return_high_id2(self, id2):
        return self._platform_ring_single(
            "id2", id2, PLATFORM_RETURN_HIGH_TIME_MS, "RING_RETURN_HIGH_ID2"
        )

    def _platform_ring_return_high_id6(self, id6):
        return self._platform_ring_single(
            "id6", id6, PLATFORM_RETURN_HIGH_TIME_MS, "RING_RETURN_HIGH_ID6"
        )

    def _platform_letter_return_high_id1(self, id1):
        return self._platform_ring_single(
            "id1", id1, PLATFORM_RETURN_HIGH_TIME_MS, "LETTER_RETURN_HIGH_ID1"
        )

    def _platform_letter_return_high_id2(self, id2):
        return self._platform_ring_single(
            "id2", id2, PLATFORM_RETURN_HIGH_TIME_MS, "LETTER_RETURN_HIGH_ID2"
        )

    def _platform_letter_return_high_id6(self, id6):
        return self._platform_ring_single(
            "id6", id6, PLATFORM_RETURN_HIGH_TIME_MS, "LETTER_RETURN_HIGH_ID6"
        )

    def _column_pose(
        self,
        target_id1,
        target_id2,
        target_id6,
        reason,
        *,
        raising,
        id7,
        id5,
        splitter_id4,
    ):
        """Send a formal task-three pose with the shared HTD85 timings."""
        bridge = self.servo_bridge
        previous_times = (
            getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS),
            getattr(bridge, "aux_time_ms", PLATFORM_AUX_TIME_MS),
            getattr(bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS),
            getattr(bridge, "splitter_time_ms", PLATFORM_AUX_TIME_MS),
        )
        try:
            bridge.arm_time_ms = PLATFORM_ARM_TIME_MS
            bridge.aux_time_ms = PLATFORM_AUX_TIME_MS
            bridge.gripper_time_ms = COLUMN_CATCH_GRIPPER_TIME_MS
            bridge.splitter_time_ms = PLATFORM_AUX_TIME_MS
            return self._send_fixed_arm_pose_staged(
                target_id1,
                target_id2,
                target_id6,
                reason,
                raising=raising,
                id7=id7,
                id5=id5,
                splitter_id4=splitter_id4,
            )
        finally:
            (
                bridge.arm_time_ms,
                bridge.aux_time_ms,
                bridge.gripper_time_ms,
                bridge.splitter_time_ms,
            ) = previous_times

    def _column_gripper(self, target, reason):
        """Pulse formal task-three ID7 with the standalone 300 ms timing."""
        bridge = self.servo_bridge
        previous = getattr(bridge, "gripper_time_ms", COLUMN_CATCH_GRIPPER_TIME_MS)
        try:
            bridge.gripper_time_ms = COLUMN_CATCH_GRIPPER_TIME_MS
            return self._send_gripper_id7(target, reason)
        finally:
            bridge.gripper_time_ms = previous

    def _column_splitter(self, target, reason):
        """Keep formal task-three splitter timing on the shared HTD85 bus."""
        bridge = self.servo_bridge
        previous = getattr(bridge, "splitter_time_ms", PLATFORM_AUX_TIME_MS)
        try:
            bridge.splitter_time_ms = PLATFORM_AUX_TIME_MS
            return self._send_splitter_id4(target, reason)
        finally:
            bridge.splitter_time_ms = previous

    def _column_center_correction(self, reason, *, send_id2, send_id6):
        """Send task-three centering steps with the standalone 100 ms motion."""
        bridge = self.servo_bridge
        previous = getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS)
        try:
            bridge.arm_time_ms = COLUMN_CATCH_CENTER_TIME_MS
            return self._send_center_correction(
                reason,
                send_id2=send_id2,
                send_id6=send_id6,
            )
        finally:
            bridge.arm_time_ms = previous

    def _sync_platform_task(self):
        task = self.platform_task
        self.chassis_station_stage = task.stage
        self.status = task.status
        if task.selected:
            self.platform_selected_letters = frozenset(task.selected)
            self.target_letters = self.platform_selected_letters
        if task.done:
            self.chassis_station_done_reason = task.done
            task.done = None
            self.active_chassis_station = None
        if task.error:
            self.chassis_station_error_reason = task.error
            task.error = None
            self.algorithm_stage = self.state = "fault"
        self.arm_preview.publish(self.status)
        return self.status

    def _begin_task3_ring_place_station(self):
        """Start the deterministic blue-field end-ring placement transaction."""

        self.task3_ring_place_actions.clear()
        self.id1, self.id2, self.id6 = TASK3_RING_PLACE_HIGH
        self.id7 = TASK3_RING_PLACE_GRIPPER_CLOSED_TICK
        self.id5 = COLUMN_CATCH_CATCHER_HOME_TICK
        self.splitter_id4 = COLUMN_CATCH_SPLITTER_TICK
        self.chassis_station_stage = "task3_ring_place_high_wait"
        self.chassis_station_deadline = 0.0
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish("TASK3_RING_PLACE high")
        bridge = self.servo_bridge
        previous = (
            getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS),
            getattr(bridge, "aux_time_ms", PLATFORM_AUX_TIME_MS),
            getattr(bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS),
            getattr(bridge, "splitter_time_ms", PLATFORM_AUX_TIME_MS),
        )
        try:
            bridge.arm_time_ms = TASK3_RING_PLACE_INITIAL_HIGH_TIME_MS
            bridge.aux_time_ms = TASK3_RING_PLACE_GRIPPER_TIME_MS
            bridge.gripper_time_ms = TASK3_RING_PLACE_GRIPPER_TIME_MS
            bridge.splitter_time_ms = PLATFORM_AUX_TIME_MS
            status = self._send_fixed_arm_pose_staged(
                *TASK3_RING_PLACE_HIGH,
                "TASK3_RING_PLACE high",
                raising=True,
                id7=TASK3_RING_PLACE_GRIPPER_CLOSED_TICK,
                id5=COLUMN_CATCH_CATCHER_HOME_TICK,
                splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
                joint_sequence_delay_s=0.10,
                first_arm_time_ms=TASK3_RING_PLACE_INITIAL_HIGH_TIME_MS,
                second_arm_time_ms=TASK3_RING_PLACE_INITIAL_HIGH_TIME_MS,
            )
        finally:
            (
                bridge.arm_time_ms,
                bridge.aux_time_ms,
                bridge.gripper_time_ms,
                bridge.splitter_time_ms,
            ) = previous
        if bridge.write_enabled and not bridge.last_command_ok:
            self.chassis_station_stage = None
            self.state = self.algorithm_stage = "fault"
            self.chassis_station_error_reason = "TASK3_RING_PLACE_HIGH_FAILED"
            return self.status
        self.chassis_station_deadline = time.monotonic() + max(
            self._arm_settle_s(), TASK3_RING_PLACE_INITIAL_HIGH_TIME_MS / 1000.0
        )
        self.status = (
            f"TASK3_RING_PLACE high ID1={self.id1} ID2={self.id2} "
            f"ID6={self.id6} ID17={self.id7} | {status}"
        )
        print(f"CHASSIS STATION TASK3_RING_PLACE READY {self.status}", flush=True)
        return self.status

    def _update_task3_ring_place_station(self):
        """Advance the end-ring action sequence without camera dependence."""

        now = time.monotonic()
        stage = self.chassis_station_stage
        if stage == "task3_ring_place_high_wait":
            if now < self.chassis_station_deadline:
                return f"TASK3_RING_PLACE high settling {self.chassis_station_deadline - now:.1f}s"
            self.task3_ring_place_actions.extend([
                ("WAIT_BEFORE_OPEN_ID17", self._task3_ring_wait,
                 (TASK3_RING_PLACE_RELEASE_HOLD_MS,
                  "wait before final ring release")),
                ("OPEN_ID17", self._task3_ring_gripper,
                 (TASK3_RING_PLACE_GRIPPER_OPEN_TICK,
                  TASK3_RING_PLACE_GRIPPER_TIME_MS, "open ID17")),
                ("ID6_PLACE", self._task3_ring_single,
                 ("id6", TASK3_RING_PLACE_POSE[2],
                  TASK3_RING_PLACE_AXIS_TIME_MS, "place ID6")),
                ("ID2_PLACE", self._task3_ring_single,
                 ("id2", TASK3_RING_PLACE_POSE[1],
                  TASK3_RING_PLACE_AXIS_TIME_MS, "place ID2")),
                ("ID1_PLACE", self._task3_ring_single,
                 ("id1", TASK3_RING_PLACE_POSE[0],
                  TASK3_RING_PLACE_ID1_TIME_MS, "place ID1")),
                ("SLOW_CLOSE_ID17", self._task3_ring_gripper,
                 (TASK3_RING_PLACE_GRIPPER_CLOSED_TICK,
                  TASK3_RING_PLACE_SLOW_CLOSE_TIME_MS, "slow close ID17")),
                ("ID1_HIGH", self._task3_ring_single,
                 ("id1", TASK3_RING_PLACE_RETURN_HIGH[0],
                  TASK3_RING_PLACE_HIGH_TIME_MS, "return high ID1")),
                ("ID2_HIGH", self._task3_ring_single,
                 ("id2", TASK3_RING_PLACE_RETURN_HIGH[1],
                  TASK3_RING_PLACE_HIGH_TIME_MS, "return high ID2")),
                ("ID6_HIGH", self._task3_ring_single,
                 ("id6", TASK3_RING_PLACE_RETURN_HIGH[2],
                  TASK3_RING_PLACE_HIGH_TIME_MS, "return high ID6")),
                ("ID1_ID2_RELEASE_CONFIRM", self._task3_ring_release_pose,
                 (TASK3_RING_PLACE_RELEASE_ID1_TICK,
                  TASK3_RING_PLACE_RELEASE_ID2_TICK,
                  TASK3_RING_PLACE_RELEASE_ID1_TIME_MS,
                  "move ID1/ID2 for release confirmation")),
                ("WAIT_AFTER_ID1_RELEASE_CONFIRM", self._task3_ring_wait,
                 (TASK3_RING_PLACE_RELEASE_ID1_HOLD_MS,
                  "wait after ID1 release-confirm position")),
                ("OPEN_ID17_AGAIN", self._task3_ring_gripper,
                (TASK3_RING_PLACE_GRIPPER_OPEN_TICK,
                  TASK3_RING_PLACE_RELEASE_GRIPPER_TIME_MS, "open ID17 again")),
                ("RETURN_HIGH_BEFORE_CLOSE", self._task3_ring_release_high,
                 (TASK3_RING_PLACE_RELEASE_HIGH_TIME_MS,
                  "return high before final close")),
                ("CLOSE_ID17_AGAIN", self._task3_ring_gripper,
                (TASK3_RING_PLACE_GRIPPER_CLOSED_TICK,
                  TASK3_RING_PLACE_RELEASE_GRIPPER_TIME_MS, "close ID17 again")),
                ("WAIT_BEFORE_CONTRACT", self._task3_ring_wait,
                 (1000, "wait after final ID17 close")),
            ])
            self.chassis_station_stage = "task3_ring_place_actions"
            self.chassis_station_deadline = now

        if self.chassis_station_stage != "task3_ring_place_actions":
            return self.status
        if now < self.chassis_station_deadline:
            return f"TASK3_RING_PLACE action settling {self.chassis_station_deadline - now:.1f}s"
        if not self.task3_ring_place_actions:
            if self.id7 != TASK3_RING_PLACE_GRIPPER_CLOSED_TICK:
                self.chassis_station_stage = None
                self.active_chassis_station = None
                self.chassis_station_error_reason = (
                    "TASK3_RING_PLACE_FINAL_GRIPPER_NOT_CLOSED"
                )
                self.state = self.algorithm_stage = "fault"
                self.status = (
                    "TASK3_RING_PLACE failed: final ID17 close not confirmed"
                )
                self.arm_preview.publish(self.status)
                return self.status
            home_status = self._task3_ring_ordered_contract()
            home_success = (
                not self.servo_bridge.write_enabled
                or self.servo_bridge.last_command_ok
            )
            if not home_success:
                self.chassis_station_stage = None
                self.active_chassis_station = None
                self.chassis_station_error_reason = (
                    "TASK3_RING_PLACE_HOME_FAILED"
                )
                self.state = self.algorithm_stage = "fault"
                self.status = (
                    "TASK3_RING_PLACE failed: arm contraction failed; "
                    f"{home_status}"
                )
                self.arm_preview.publish(self.status)
                return self.status
            self.chassis_station_stage = None
            self.active_chassis_station = None
            self.chassis_station_done_reason = "TASK3_RING_PLACE_DONE"
            self.status = f"TASK3_RING_PLACE DONE; arm contracted | {home_status}"
            self.arm_preview.publish(self.status)
            return self.status
        label, callback, args = self.task3_ring_place_actions.popleft()
        try:
            settle = callback(*args)
            if settle is False or settle is None:
                raise RuntimeError("servo write failed")
        except Exception as exc:
            self.task3_ring_place_actions.clear()
            self.chassis_station_stage = None
            self.chassis_station_error_reason = f"TASK3_RING_PLACE_{label}_FAILED"
            self.state = self.algorithm_stage = "fault"
            self.status = f"TASK3_RING_PLACE {label} failed: {exc}"
            self.arm_preview.publish(self.status)
            return self.status
        self.chassis_station_deadline = time.monotonic() + float(settle)
        self.status = f"TASK3_RING_PLACE {label} dispatched"
        return self.status

    def begin_platform_preselect(self, count=2):
        self._reset_cycle_for_search("platform secondary preselect start")
        self.active_chassis_station = "PLATFORM_PICK"
        self.chassis_station_done_reason = self.chassis_station_error_reason = None
        self.platform_selected_letters = frozenset()
        self.platform_task.begin_preselect()
        return self._sync_platform_task()

    def update_platform_preselect(self, detections, detection_fresh=True):
        self.platform_task.preselect(detections, fresh=detection_fresh)
        return self._sync_platform_task()

    def prepare_chassis_station_high(self, prep):
        if not isinstance(prep, dict) or prep.get("task") != "DISC_CATCH":
            return "chassis prep ignored"

        target_id1 = int(prep.get("id1") or DISC_CATCH_PREP_ID1_TICK)
        target_id2 = int(prep.get("id2") or DISC_CATCH_PREP_ID2_TICK)
        if (
            self.disc_prep_high_active
            and self.id1 == target_id1
            and self.id2 == target_id2
            and self.id6 == DISC_CATCH_ID6_TICK
        ):
            return (
                f"DISC_CATCH prep high already active ID1={self.id1} "
                f"ID2={self.id2} ID6={self.id6}"
            )
        target_id1, target_id2 = enforce_angle_gap(
            target_id1,
            target_id2,
            self.id2_limits,
            self.angle_gap_degrees,
        )
        self.id7 = self.id7_closed
        # PREP_HIGH is also the white-line observation pose. Keep the catcher
        # retracted while the chassis is still correcting its station pose.
        self.id5 = TASK1_ID15_RETRACT_TICK
        self.splitter_id4 = DISC_CATCH_PREP_SPLITTER_TICK
        self.arm_preview.set_targets(target_id1, target_id2, self.id7, DISC_CATCH_ID6_TICK)
        self.arm_preview.publish("DISC_CATCH prep high")
        if not self.servo_bridge.write_enabled:
            self.id1 = target_id1
            self.id2 = target_id2
            self.id6 = DISC_CATCH_ID6_TICK
            self.disc_prep_high_active = True
            return (
                f"preview DISC_CATCH prep high ID1={self.id1} "
                f"ID2={self.id2} ID6={self.id6}"
            )
        status = self._send_fixed_arm_pose_staged(
            target_id1,
            target_id2,
            DISC_CATCH_ID6_TICK,
            "DISC_CATCH prep high",
            raising=True,
            id7=self.id7,
            id5=self.id5,
            splitter_id4=self.splitter_id4,
            aux_time_ms=TASK1_AUX_TIME_MS,
            splitter_time_ms=TASK1_ID14_TIME_MS,
            joint_sequence_delay_s=0.10,
            first_arm_time_ms=500,
            second_arm_time_ms=500,
        )
        if not self.servo_bridge.last_command_ok:
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.status = f"DISC_CATCH prep high failed: {status}"
            self.arm_preview.publish(self.status)
            return self.status
        self.status = (
            f"DISC_CATCH prep high ID1={self.id1} ID2={self.id2} "
            f"ID5={self.id5} ID6={self.id6} | {status}"
        )
        self.disc_prep_high_active = True
        print(f"CHASSIS PREP DISC_CATCH HIGH {self.status}", flush=True)
        return self.status

    def _begin_disc_catch_station(self):
        """Enter the task-one ball action used by the validated desktop app."""
        self.disc_prep_high_active = False
        self.id7 = self.id7_closed
        self.id5 = DISC_CATCH_CATCHER_READY_TICK
        self.splitter_id4 = DISC_CATCH_SPLITTER_READY_TICK
        self.chassis_station_stage = "disc_app_low_settle"
        self.disc_pulse_done = False
        self.disc_last_pulsed_color = None
        self.disc_last_pulsed_center = None
        self.disc_blue_clear_frames = 0
        self.disc_fast_blue_cycle = False
        self.disc_blue_channel_hold_deadline = 0.0
        self.chassis_station_deadline = 0.0
        self.chassis_station_no_target_deadline = 0.0
        self.arm_preview.set_targets(
            DISC_CATCH_READY_ID1_TICK,
            DISC_CATCH_READY_ID2_TICK,
            self.id7,
            DISC_CATCH_ID6_TICK,
        )
        self.arm_preview.publish("DISC_CATCH app low pose")
        if not self.servo_bridge.write_enabled:
            self.id1 = DISC_CATCH_READY_ID1_TICK
            self.id2 = DISC_CATCH_READY_ID2_TICK
            self.id6 = DISC_CATCH_ID6_TICK
            self.chassis_station_deadline = (
                time.monotonic() + self._disc_low_pose_settle_s()
            )
            self.status = (
                f"preview DISC_CATCH app low pose ID1={self.id1} "
                f"ID2={self.id2} ID4={self.splitter_id4} ID5={self.id5} "
                f"ID6={self.id6} ID7={self.id7}"
            )
            return self.status
        status = self._send_fixed_arm_pose_staged(
            DISC_CATCH_READY_ID1_TICK,
            DISC_CATCH_READY_ID2_TICK,
            DISC_CATCH_ID6_TICK,
            "DISC_CATCH app low pose",
            raising=False,
            id7=self.id7,
            id5=self.id5,
            splitter_id4=self.splitter_id4,
            aux_time_ms=TASK1_AUX_TIME_MS,
            splitter_time_ms=TASK1_ID14_TIME_MS,
            joint_sequence_delay_s=0.20,
            first_arm_time_ms=500,
            second_arm_time_ms=600,
        )
        if not self.servo_bridge.last_command_ok:
            self.chassis_station_stage = None
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.status = f"DISC_CATCH app low pose failed: {status}"
            self.arm_preview.publish(self.status)
            return self.status
        self.chassis_station_deadline = (
            time.monotonic() + self._disc_low_pose_settle_s()
        )
        self.status = (
            f"DISC_CATCH app low pose ID1={self.id1} ID2={self.id2} "
            f"ID4={self.splitter_id4} ID5={self.id5} "
            f"ID6={self.id6} ID7={self.id7} | {status}"
        )
        print(f"CHASSIS STATION DISC_CATCH APP LOW {self.status}", flush=True)
        return self.status

    def _begin_column_catch_station(self):
        self.task3_supplement_actions.clear()
        self.task3_supplement_label = None
        self.task3_supplement_active = False
        self.task3_supplement_done = False
        self.task3_supplement_hold_completed = False
        self.id7 = COLUMN_CATCH_GRIPPER_CLOSED_TICK
        self.id5 = COLUMN_CATCH_CATCHER_HOME_TICK
        self.splitter_id4 = COLUMN_CATCH_SPLITTER_TICK
        self.chassis_station_stage = "column_ready"
        self.column_grab_id2 = None
        self.column_target_armed = True
        self.column_target_absent_frames = 0
        self.column_pending_target = None
        self.column_pending_frames = 0
        self.column_locked_block = None
        self.column_locked_distance_cm = None
        self.column_capture_authorized = False
        self.column_handled_blocks = []
        self.column_classify_deadline = 0.0
        self.column_classify_votes = []
        self.column_target_lost_since = None
        self.column_pause_deadline = 0.0
        self.column_resume_deadline = 0.0
        self.chassis_station_deadline = 0.0
        self.arm_preview.set_targets(
            COLUMN_CATCH_READY_ID1_TICK,
            COLUMN_CATCH_READY_ID2_TICK,
            self.id7,
            COLUMN_CATCH_READY_ID6_TICK,
        )
        self.arm_preview.publish("COLUMN_CATCH ready")
        if not self.servo_bridge.write_enabled:
            self.id1 = COLUMN_CATCH_READY_ID1_TICK
            self.id2 = COLUMN_CATCH_READY_ID2_TICK
            self.id6 = COLUMN_CATCH_READY_ID6_TICK
            self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
            self.status = (
                f"preview COLUMN_CATCH ready ID1={self.id1} "
                f"ID2={self.id2} ID6={self.id6} ID7={self.id7}"
            )
            return self.status
        status = self._column_pose(
            COLUMN_CATCH_READY_ID1_TICK,
            COLUMN_CATCH_READY_ID2_TICK,
            COLUMN_CATCH_READY_ID6_TICK,
            "COLUMN_CATCH ready",
            raising=True,
            id7=COLUMN_CATCH_GRIPPER_CLOSED_TICK,
            id5=self.id5,
            splitter_id4=self.splitter_id4,
        )
        if not self.servo_bridge.last_command_ok:
            self.chassis_station_stage = None
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.status = f"COLUMN_CATCH ready failed: {status}"
            self.arm_preview.publish(self.status)
            return self.status
        self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
        self.status = (
            f"COLUMN_CATCH ready ID1={self.id1} ID2={self.id2} "
            f"ID4={self.splitter_id4} ID5={self.id5} "
            f"ID6={self.id6} ID7={self.id7} | {status}"
        )
        print(f"CHASSIS STATION COLUMN_CATCH READY {self.status}", flush=True)
        return self.status

    def _disc_catch_allowed_colors(self):
        return self.target_policy.disc_colors

    def _disc_catch_ball_visible(self, detections):
        allowed_colors = self._disc_catch_allowed_colors()
        allowed = []
        rejected = []
        rejected_outside_window = []
        for det in detections:
            if det.get("kind") != "ball":
                continue
            color = det.get("color")
            if color not in allowed_colors:
                if color in {"red", "blue", "yellow"}:
                    rejected.append(color)
                continue
            center = det.get("center")
            if not isinstance(center, (tuple, list)) or len(center) < 2:
                rejected_outside_window.append(color)
                continue
            try:
                center_x = float(center[0])
                center_y = float(center[1])
            except (TypeError, ValueError):
                rejected_outside_window.append(color)
                continue
            if not (
                DISC_CATCH_WINDOW_X_MIN <= center_x <= DISC_CATCH_WINDOW_X_MAX
                and DISC_CATCH_WINDOW_Y_MIN <= center_y <= DISC_CATCH_WINDOW_Y_MAX
            ):
                rejected_outside_window.append(
                    f"{color}@({center_x:.0f},{center_y:.0f})"
                )
                continue
            allowed.append(det)
        if rejected_outside_window:
            print(
                "CHASSIS STATION DISC_CATCH ignored target(s) outside "
                f"window X={DISC_CATCH_WINDOW_X_MIN}..{DISC_CATCH_WINDOW_X_MAX} "
                f"Y={DISC_CATCH_WINDOW_Y_MIN}..{DISC_CATCH_WINDOW_Y_MAX}: "
                f"{rejected_outside_window}",
                flush=True,
            )
        if rejected:
            print(
                "CHASSIS STATION DISC_CATCH ignored "
                f"{sorted(set(rejected))} ball(s) for field "
                f"{self.field_mode.wire_name}; allowed={sorted(allowed_colors)}",
                flush=True,
            )
        if not allowed:
            return None

        return max(allowed, key=lambda det: float(det.get("area_percent") or 0.0))

    def _column_letter_visible(self, detections):
        candidates = [
            det
            for det in detections
            if self.target_policy.matches_column_letter(
                det, self.target_letters
            )
        ]
        return max(
            candidates,
            key=lambda det: (
                float(det.get("confidence") or 0.0),
                float(det.get("projected_area") or 0.0),
            ),
            default=None,
        )

    def _column_any_letter_visible(self, detections):
        candidates = [
            det for det in detections
            if det.get("kind") == "letter"
            and str(det.get("letter", "")).upper() in LETTERS
            and det.get("fully_visible", True)
        ]
        return max(
            candidates,
            key=lambda det: (
                float(det.get("confidence") or 0.0),
                float(det.get("projected_area") or 0.0),
            ),
            default=None,
        )

    @staticmethod
    def _column_bbox_iou(first, second):
        if not first or not second or len(first) < 4 or len(second) < 4:
            return 0.0
        ax, ay, aw, ah = (float(value) for value in first[:4])
        bx, by, bw, bh = (float(value) for value in second[:4])
        ax2, ay2 = ax + max(0.0, aw), ay + max(0.0, ah)
        bx2, by2 = bx + max(0.0, bw), by + max(0.0, bh)
        intersection = max(0.0, min(ax2, bx2) - max(ax, bx)) * max(
            0.0, min(ay2, by2) - max(ay, by)
        )
        union = max(1.0, aw * ah + bw * bh - intersection)
        return intersection / union

    @staticmethod
    def _column_rotation_votes_stable(votes, label):
        angles = [
            float(vote["rotation_angle_deg"]) % 90.0
            for vote in votes
            if vote is not None
            and str(vote.get("letter", "")).upper() == str(label).upper()
            and vote.get("rotation_angle_deg") is not None
        ]
        if len(angles) < COLUMN_CLASSIFY_MIN_VOTES:
            return False
        radians = np.radians(np.asarray(angles, dtype=np.float64) * 4.0)
        center = (math.degrees(math.atan2(
            float(np.sin(radians).mean()), float(np.cos(radians).mean())
        )) / 4.0) % 90.0
        deviations = [
            abs((angle - center + 45.0) % 90.0 - 45.0)
            for angle in angles
        ]
        return max(deviations, default=0.0) <= COLUMN_CLASSIFY_MAX_ANGLE_JITTER_DEG

    @classmethod
    def _column_blocks_match(cls, first, second):
        if first is None or second is None:
            return False
        first_center = np.asarray(first.get("center", ()), dtype=np.float32)
        second_center = np.asarray(second.get("center", ()), dtype=np.float32)
        if first_center.shape != (2,) or second_center.shape != (2,):
            return False
        return (
            float(np.linalg.norm(first_center - second_center))
            <= COLUMN_BLOCK_MATCH_CENTER_PX
            and cls._column_bbox_iou(first.get("bbox"), second.get("bbox"))
            >= COLUMN_BLOCK_MATCH_IOU
        )

    @staticmethod
    def _column_block_tracks_match(first, second):
        """Track the locked white block while arm centering changes its view."""
        if first is None or second is None:
            return False
        first_center = np.asarray(first.get("center", ()), dtype=np.float32)
        second_center = np.asarray(second.get("center", ()), dtype=np.float32)
        first_bbox = first.get("bbox", ())
        second_bbox = second.get("bbox", ())
        if (
            first_center.shape != (2,)
            or second_center.shape != (2,)
            or len(first_bbox) < 4
            or len(second_bbox) < 4
        ):
            return False
        first_size = max(float(first_bbox[2]), float(first_bbox[3]), 1.0)
        second_size = max(float(second_bbox[2]), float(second_bbox[3]), 1.0)
        size_ratio = max(first_size, second_size) / min(first_size, second_size)
        return (
            float(np.linalg.norm(first_center - second_center))
            <= COLUMN_LOCKED_BLOCK_MAX_JUMP_PX
            and size_ratio <= COLUMN_LOCKED_BLOCK_MAX_SIZE_RATIO
        )

    @classmethod
    def _column_letters_in_block(cls, detections, block):
        if block is None:
            return []
        bbox = block.get("bbox", ())
        if len(bbox) < 4:
            return []
        x, y, width, height = (float(value) for value in bbox[:4])
        pad_x = max(12.0, width * 0.22)
        pad_y = max(12.0, height * 0.22)
        candidates = []
        for det in detections:
            if (
                det.get("kind") != "letter"
                or not det.get("fully_visible", True)
                or str(det.get("letter", "")).upper() not in LETTERS
            ):
                continue
            center = np.asarray(det.get("center", ()), dtype=np.float32)
            if center.shape != (2,):
                continue
            if (
                x - pad_x <= float(center[0]) <= x + width + pad_x
                and y - pad_y <= float(center[1]) <= y + height + pad_y
            ):
                candidates.append(det)
        return candidates

    def _column_letter_quota_available(self, label):
        return (
            self.letter_success_counts.get(str(label).upper(), 0)
            < PLATFORM_LETTER_SUCCESS_QUOTA
        )

    @classmethod
    def _column_locked_letter_fallback(cls, detections, reference, block):
        """Recover the locked glyph if its white border is briefly hidden."""
        if reference is None or block is None:
            return None
        label = str(reference.get("letter", "")).upper()
        reference_center = np.asarray(reference.get("center", ()), dtype=np.float32)
        block_center = np.asarray(block.get("center", ()), dtype=np.float32)
        if reference_center.shape != (2,):
            return None
        reference_bbox = reference.get("bbox", ())
        block_bbox = block.get("bbox", ())
        reference_size = max(
            float(reference_bbox[2]), float(reference_bbox[3]), 1.0
        ) if len(reference_bbox) >= 4 else None
        block_size = max(
            float(block_bbox[2]), float(block_bbox[3]), 1.0
        ) if len(block_bbox) >= 4 else 120.0
        candidates = []
        for det in detections:
            if (
                det.get("kind") != "letter"
                or not det.get("fully_visible", True)
                or str(det.get("letter", "")).upper() != label
            ):
                continue
            center = np.asarray(det.get("center", ()), dtype=np.float32)
            if center.shape != (2,):
                continue
            if float(np.linalg.norm(center - reference_center)) > COLUMN_LOCKED_LETTER_MAX_JUMP_PX:
                continue
            if block_center.shape == (2,) and float(np.linalg.norm(center - block_center)) > max(
                COLUMN_LOCKED_BLOCK_MAX_JUMP_PX * 2.0, block_size * 3.0
            ):
                continue
            bbox = det.get("bbox", ())
            if reference_size is not None and len(bbox) >= 4:
                size = max(float(bbox[2]), float(bbox[3]), 1.0)
                if max(reference_size, size) / min(reference_size, size) > COLUMN_LOCKED_LETTER_MAX_SIZE_RATIO:
                    continue
            candidates.append(det)
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda det: float(
                np.linalg.norm(
                    np.asarray(det.get("center"), dtype=np.float32) - reference_center
                )
            ),
        )

    def _column_block_visible(self, detections):
        candidates = [
            det for det in detections
            if det.get("kind") == "column_block"
            and det.get("fully_visible", True)
            and not self._column_block_has_processed_letter(detections, det)
        ]
        return max(
            candidates,
            key=lambda det: float(det.get("projected_area") or 0.0),
            default=None,
        )

    def _column_block_has_processed_letter(self, detections, block):
        """Return whether this visible block belongs to a completed letter."""
        if block is None or not self.column_handled_blocks:
            return False
        return any(
            self._column_block_tracks_match(
                handled.get("block"), block
            )
            for handled in self.column_handled_blocks
        )

    def _column_update_handled_blocks(self, detections):
        """Keep processed block locks while visible, then re-arm them."""
        blocks = [
            det for det in detections
            if det.get("kind") == "column_block"
            and det.get("fully_visible", True)
        ]
        active = []
        for handled in self.column_handled_blocks:
            visible = any(
                self._column_block_tracks_match(handled.get("block"), block)
                for block in blocks
            )
            if visible:
                handled["missing_frames"] = 0
                active.append(handled)
                continue
            handled["missing_frames"] = int(
                handled.get("missing_frames", 0)
            ) + 1
            if handled["missing_frames"] < COLUMN_HANDLED_BLOCK_REARM_FRAMES:
                active.append(handled)
            else:
                print(
                    f"CHASSIS STATION COLUMN_CATCH block rearmed "
                    f"label={handled.get('label', '?')} reason=NOT_VISIBLE",
                    flush=True,
                )
        self.column_handled_blocks = active

    def _column_record_handled_block(self, label, block):
        """Suppress one processed physical block without blocking new blocks."""
        if block is None:
            return
        bbox = tuple(block.get("bbox", ()))
        center = tuple(block.get("center", ()))
        if len(bbox) < 4 or len(center) != 2:
            return
        if any(
            self._column_block_tracks_match(item.get("block"), block)
            for item in self.column_handled_blocks
        ):
            return
        self.column_handled_blocks.append(
            {
                "label": str(label or "?").upper(),
                "block": {"bbox": bbox, "center": center},
                "missing_frames": 0,
            }
        )
        print(
            f"CHASSIS STATION COLUMN_CATCH block locked "
            f"label={str(label or '?').upper()} center={center} "
            "action=SKIP_WHILE_VISIBLE",
            flush=True,
        )

    def _finish_chassis_station_after_retract(self, reason):
        # Every completed station must leave the arm fully retracted. In
        # particular, task-one completion must not leave ID5 at the catcher
        # ready position while H7 transfers to task two.
        self.platform_high_hold = False
        self.platform_high_pose_sent = False
        station = self.active_chassis_station
        result = self.shutdown_contract(
            raise_before_home=station == "DISC_CATCH"
        )
        success = (
            not self.servo_bridge.write_enabled
            or self.servo_bridge.last_command_ok
        )
        self.chassis_station_stage = None
        self.active_chassis_station = None
        if success:
            self.chassis_station_done_reason = reason
            self.chassis_station_error_reason = None
        else:
            self.chassis_station_done_reason = None
            self.chassis_station_error_reason = "HOME_FAILED"
            self.state = "fault"
            self.algorithm_stage = "fault"
        return f"{reason}; {result}"

    def reset_from_chassis(self):
        self.platform_task.reset()
        self.task3_ring_place_actions.clear()
        """Retract and discard every task/cycle state for a new H7 route."""

        station = self.active_chassis_station
        result = self.shutdown_contract(
            raise_before_home=station == "DISC_CATCH"
        )
        success = (
            not self.servo_bridge.write_enabled
            or self.servo_bridge.last_command_ok
        )
        self.active_chassis_station = None
        self.chassis_station_done_reason = None
        self.chassis_station_error_reason = None
        self.chassis_station_stage = None
        self.chassis_station_deadline = 0.0
        self.chassis_station_no_target_deadline = 0.0
        self.disc_pulse_done = False
        self.disc_last_pulsed_color = None
        self.disc_last_pulsed_center = None
        self.disc_fast_blue_cycle = False
        self.disc_blue_channel_hold_deadline = 0.0
        self.disc_prep_high_active = False
        self.platform_high_hold = False
        self.platform_high_pose_sent = False
        self.column_target_armed = True
        self.column_target_absent_frames = 0
        self.column_pending_target = None
        self.column_pending_frames = 0
        self.column_locked_block = None
        self.column_locked_distance_cm = None
        self.column_capture_authorized = False
        self.column_target_lost_since = None
        self.column_pause_deadline = 0.0
        self.column_resume_deadline = 0.0
        self._reset_cycle_for_search("chassis reset; arm home")
        return success, result

    def consume_chassis_station_done(self):
        reason = self.chassis_station_done_reason
        self.chassis_station_done_reason = None
        return reason

    def consume_chassis_station_error(self):
        reason = self.chassis_station_error_reason
        self.chassis_station_error_reason = None
        return reason

    def skip_platform_slot(self, reason="TARGET_NOT_SELECTED"):
        if self.active_chassis_station != "PLATFORM_PICK":
            return False
        self.platform_task.skip(reason)
        self._sync_platform_task()
        return True

    def stop_chassis_station(self, station):
        self.platform_task.reset()
        self.task3_ring_place_actions.clear()
        if self.active_chassis_station != station:
            return f"chassis station {station} stop ignored; active={self.active_chassis_station}"
        if station == "COLUMN_CATCH":
            return self.hold_formal_task3_arm()
        return self._finish_chassis_station_after_retract("STOPPED_BY_CHASSIS")

    def hold_formal_task3_arm(self):
        """Hold the formal task-three arm high until H7 requests retract."""
        if self.active_chassis_station != "COLUMN_CATCH":
            self.chassis_station_error_reason = "TASK3_HOLD_WITHOUT_COLUMN_CATCH"
            return "task-three hold ignored; no active COLUMN_CATCH"
        # H7 may retransmit HOLD while the supplement is running. Keep the
        # current action queue intact; restarting it would replay OPEN_HIGH
        # forever and prevent the descend/close/place steps from running.
        if self.task3_supplement_active:
            return "HOLD_EXPANDED_HIGH; supplement in progress"
        if self.task3_supplement_hold_completed:
            return "HOLD_EXPANDED_HIGH; supplement already complete"
        self.platform_task.reset()
        self.task3_ring_place_actions.clear()
        status = self._column_pose(
            *COLUMN_CATCH_HOLD_HIGH,
            "COLUMN_CATCH orbit boundary; hold expanded high",
            raising=True,
            id7=COLUMN_CATCH_GRIPPER_CLOSED_TICK,
            id5=COLUMN_CATCH_CATCHER_HOME_TICK,
            splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
        )
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            self.chassis_station_error_reason = "TASK3_HOLD_HIGH_FAILED"
            return f"COLUMN_CATCH hold high failed: {status}"
        self.id1, self.id2, self.id6 = COLUMN_CATCH_HOLD_HIGH
        self.platform_high_hold = True
        self.platform_high_pose_sent = True
        self.task3_supplement_label = (
            self._task3_supplement_label()
            if self.field_mode == FieldMode.BLUE
            else None
        )
        self.task3_supplement_done = False
        if self.task3_supplement_label is not None:
            self.task3_supplement_active = True
            self.task3_supplement_hold_completed = False
            self.task3_supplement_actions.clear()
            self.chassis_station_stage = "task3_supplement_start"
        else:
            self.task3_supplement_active = False
            self.task3_supplement_hold_completed = True
            self.chassis_station_stage = None
        # Keep COLUMN_CATCH active until H7 sends RETRACT after white-line
        # alignment. No generic station cleanup is allowed in this state.
        self.chassis_station_done_reason = None
        self.chassis_station_error_reason = None
        self.status = (
            "COLUMN_CATCH arm held expanded "
            f"ID1={self.id1} ID2={self.id2} ID6={self.id6}"
        )
        if self.task3_supplement_label is not None:
            self.status += (
                " | fixed supplement pending "
                f"letter={self.task3_supplement_label}"
            )
        self.arm_preview.publish(self.status)
        return f"HOLD_EXPANDED_HIGH; {status}"

    def hold_blue_task3_arm(self):
        """Backward-compatible entry point for existing BLUE callers/tests."""
        if self.field_mode != FieldMode.BLUE:
            self.chassis_station_error_reason = "BLUE_HOLD_WITHOUT_COLUMN_CATCH"
            return "blue task-three hold ignored outside BLUE field"
        return self.hold_formal_task3_arm()

    def _task3_supplement_label(self):
        """Choose one deficient letter from the locked secondary-camera pair."""
        selected = tuple(sorted(self.platform_selected_letters))
        if len(selected) != 2:
            print(
                "COLUMN_CATCH supplement skipped: secondary pair is not locked",
                flush=True,
            )
            return None
        deficient = [
            label for label in selected
            if self.letter_success_counts.get(label, 0)
            < PLATFORM_LETTER_SUCCESS_QUOTA
        ]
        if not deficient:
            print(
                "COLUMN_CATCH supplement not required: selected pair quotas complete",
                flush=True,
            )
            return None
        return min(
            deficient,
            key=lambda label: (
                self.letter_success_counts.get(label, 0),
                label,
            ),
        )

    def task3_supplement_pending(self):
        return bool(self.task3_supplement_active)

    def consume_task3_supplement_done(self):
        if not self.task3_supplement_done:
            return False
        self.task3_supplement_done = False
        return True

    def _task3_supplement_command(
        self, reason, motion_ms, *, id1=None, id2=None, id6=None, id7=None
    ):
        """Send one fixed supplement motion with explicit timing."""
        targets = {}
        if id1 is not None:
            targets["id1"] = int(id1)
        if id2 is not None:
            targets["id2"] = int(id2)
        if id6 is not None:
            targets["id6"] = int(id6)
        if id7 is not None:
            targets["id4"] = int(id7)
        if not targets:
            return False
        bridge = self.servo_bridge
        previous_arm = getattr(bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS)
        previous_gripper = getattr(
            bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS
        )
        try:
            bridge.arm_time_ms = int(motion_ms)
            bridge.gripper_time_ms = int(motion_ms)
            status = bridge.send_targets(**targets)
        finally:
            bridge.arm_time_ms = previous_arm
            bridge.gripper_time_ms = previous_gripper
        self.last_command_time = time.monotonic()
        if bridge.write_enabled and not bridge.last_command_ok:
            return False
        if id1 is not None:
            self.id1 = int(id1)
        if id2 is not None:
            self.id2 = int(id2)
        if id6 is not None:
            self.id6 = int(id6)
        if id7 is not None:
            self.id7 = int(id7)
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        print(
            f"COLUMN_CATCH SUPPLEMENT {reason} "
            f"ID1={self.id1} ID2={self.id2} ID6={self.id6} ID17={self.id7} "
            f"time={int(motion_ms)}ms | {status}",
            flush=True,
        )
        return max(0.10, int(motion_ms) / 1000.0)

    def _task3_supplement_wait(self, wait_ms, label):
        print(
            f"COLUMN_CATCH SUPPLEMENT {label} wait={int(wait_ms)}ms",
            flush=True,
        )
        return max(0.10, int(wait_ms) / 1000.0)

    def _task3_supplement_pose(self, pose, label, *, raising):
        status = self._column_pose(
            *pose,
            f"COLUMN_CATCH SUPPLEMENT {label}",
            raising=raising,
            id7=COLUMN_CATCH_GRIPPER_CLOSED_TICK,
            id5=COLUMN_CATCH_CATCHER_HOME_TICK,
            splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
        )
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            return False
        print(f"COLUMN_CATCH SUPPLEMENT {label} | {status}", flush=True)
        return self._arm_settle_s()

    def _task3_supplement_gripper(self, target, motion_ms, label):
        status = self._send_gripper_id7(
            target,
            f"COLUMN_CATCH SUPPLEMENT {label}",
            motion_ms=motion_ms,
        )
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            return False
        print(f"COLUMN_CATCH SUPPLEMENT {label} | {status}", flush=True)
        return max(0.10, int(motion_ms) / 1000.0)

    def _update_task3_supplement(self):
        """Run the fixed post-orbit grab before releasing H7's hold."""
        now = time.monotonic()
        if self.chassis_station_stage == "task3_supplement_start":
            self.task3_supplement_actions.extend([
                (
                    "OPEN_HIGH",
                    self._task3_supplement_command,
                    ("open high", TASK3_SUPPLEMENT_OPEN_HOLD_MS),
                    {
                        "id1": COLUMN_CATCH_HOLD_HIGH[0],
                        "id2": COLUMN_CATCH_HOLD_HIGH[1],
                        "id6": COLUMN_CATCH_HOLD_HIGH[2],
                        "id7": COLUMN_CATCH_GRIPPER_OPEN_TICK,
                    },
                ),
                (
                    "DESCEND_ID1",
                    self._task3_supplement_command,
                    ("fixed ID1 descend", TASK3_SUPPLEMENT_DESCEND_TIME_MS),
                    {"id1": TASK3_SUPPLEMENT_DESCEND_ID1_TICK},
                ),
                (
                    "CLOSE_ID17",
                    self._task3_supplement_gripper,
                    (
                        COLUMN_CATCH_GRIPPER_CLOSED_TICK,
                        TASK3_SUPPLEMENT_GRIPPER_TIME_MS,
                        "close ID17",
                    ),
                    {},
                ),
                (
                    "RETURN_HIGH_ID1",
                    self._task3_supplement_command,
                    ("return regular high ID1", TASK3_SUPPLEMENT_RETURN_HIGH_TIME_MS),
                    {"id1": TASK3_SUPPLEMENT_HIGH[0]},
                ),
                (
                    "RETURN_HIGH_ID2",
                    self._task3_supplement_command,
                    ("return regular high ID2", TASK3_SUPPLEMENT_RETURN_HIGH_TIME_MS),
                    {"id2": TASK3_SUPPLEMENT_HIGH[1]},
                ),
                (
                    "RETURN_HIGH_ID6",
                    self._task3_supplement_command,
                    ("return regular high ID6", TASK3_SUPPLEMENT_RETURN_HIGH_TIME_MS),
                    {"id6": TASK3_SUPPLEMENT_HIGH[2]},
                ),
                (
                    "PLACE_WORK_ID6",
                    self._task3_supplement_command,
                    ("place block ID6", TASK3_SUPPLEMENT_PLACE_TIME_MS),
                    {"id6": TASK3_SUPPLEMENT_PLACE[2]},
                ),
                (
                    "PLACE_WORK_ID2",
                    self._task3_supplement_command,
                    ("place block ID2", TASK3_SUPPLEMENT_PLACE_TIME_MS),
                    {"id2": TASK3_SUPPLEMENT_PLACE[1]},
                ),
                (
                    "PLACE_WORK_ID1",
                    self._task3_supplement_command,
                    ("place block ID1", TASK3_SUPPLEMENT_PLACE_TIME_MS),
                    {"id1": TASK3_SUPPLEMENT_PLACE[0]},
                ),
                (
                    "OPEN_RELEASE",
                    self._task3_supplement_gripper,
                    (
                        COLUMN_CATCH_GRIPPER_OPEN_TICK,
                        TASK3_SUPPLEMENT_GRIPPER_TIME_MS,
                        "release block",
                    ),
                    {},
                ),
                (
                    "CLOSE_RELEASE",
                    self._task3_supplement_gripper,
                    (
                        COLUMN_CATCH_GRIPPER_CLOSED_TICK,
                        TASK3_SUPPLEMENT_GRIPPER_TIME_MS,
                        "close after release",
                    ),
                    {},
                ),
                (
                    "FINAL_HIGH_ID1",
                    self._task3_supplement_command,
                    ("final regular high ID1", TASK3_SUPPLEMENT_AXIS_TIME_MS),
                    {"id1": TASK3_SUPPLEMENT_HIGH[0]},
                ),
                (
                    "FINAL_HIGH_ID2",
                    self._task3_supplement_command,
                    ("final regular high ID2", TASK3_SUPPLEMENT_AXIS_TIME_MS),
                    {"id2": TASK3_SUPPLEMENT_HIGH[1]},
                ),
                (
                    "FINAL_HIGH_ID6",
                    self._task3_supplement_command,
                    ("final regular high ID6", TASK3_SUPPLEMENT_AXIS_TIME_MS),
                    {"id6": TASK3_SUPPLEMENT_HIGH[2]},
                ),
            ])
            self.chassis_station_stage = "task3_supplement_actions"
            self.chassis_station_deadline = now

        if self.chassis_station_stage != "task3_supplement_actions":
            return self.status
        if now < self.chassis_station_deadline:
            return (
                "COLUMN_CATCH supplement settling "
                f"{self.chassis_station_deadline - now:.1f}s"
            )
        if not self.task3_supplement_actions:
            label = self.task3_supplement_label
            if label is not None:
                self.letter_success_counts[label] = (
                    self.letter_success_counts.get(label, 0) + 1
                )
                print(
                    "COLUMN_CATCH SUPPLEMENT_SUCCESS "
                    f"letter={label} count={self.letter_success_counts[label]}/"
                    f"{PLATFORM_LETTER_SUCCESS_QUOTA}",
                    flush=True,
                )
            self.task3_supplement_active = False
            self.task3_supplement_done = True
            self.task3_supplement_hold_completed = True
            self.chassis_station_stage = None
            self.status = (
                "COLUMN_CATCH supplement complete; arm high and closed; "
                "ready for H7 HOLD_DONE"
            )
            self.arm_preview.publish(self.status)
            return self.status
        label, callback, args, kwargs = self.task3_supplement_actions.popleft()
        try:
            settle = callback(*args, **kwargs)
            if settle is False or settle is None:
                raise RuntimeError("servo write failed")
        except Exception as exc:
            self.task3_supplement_actions.clear()
            self.task3_supplement_active = False
            self.chassis_station_stage = None
            self.chassis_station_error_reason = f"TASK3_SUPPLEMENT_{label}_FAILED"
            self.state = self.algorithm_stage = "fault"
            self.status = f"COLUMN_CATCH supplement {label} failed: {exc}"
            self.arm_preview.publish(self.status)
            return self.status
        self.chassis_station_deadline = time.monotonic() + float(settle)
        self.status = f"COLUMN_CATCH supplement {label} dispatched"
        return self.status

    def retract_formal_task3_arm(self):
        self.task3_supplement_actions.clear()
        self.task3_supplement_label = None
        self.task3_supplement_active = False
        self.task3_supplement_done = False
        self.task3_supplement_hold_completed = False
        self.platform_high_hold = False
        status = self.shutdown_contract()
        success = not self.servo_bridge.write_enabled or self.servo_bridge.last_command_ok
        self.active_chassis_station = None
        self.chassis_station_stage = None
        self.chassis_station_done_reason = None
        self.chassis_station_error_reason = (
            None if success else "TASK3_RETRACT_FAILED"
        )
        return success, status

    def retract_blue_task3_arm(self):
        """Backward-compatible BLUE wrapper around the formal retract path."""
        if self.field_mode != FieldMode.BLUE:
            return False, "blue task-three retract ignored outside BLUE field"
        return self.retract_formal_task3_arm()

    def freeze_formal_task3_arm(self, reason="H7_HOLD_FAILED"):
        """Keep the task-three arm high if H7 reports a hold failure."""
        if self.active_chassis_station != "COLUMN_CATCH":
            return f"task-three freeze ignored; active={self.active_chassis_station}"
        self.platform_task.reset()
        self.task3_ring_place_actions.clear()
        status = self._column_pose(
            *COLUMN_CATCH_HOLD_HIGH,
            f"COLUMN_CATCH hold failure; freeze high ({reason})",
            raising=True,
            id7=COLUMN_CATCH_GRIPPER_CLOSED_TICK,
            id5=COLUMN_CATCH_CATCHER_HOME_TICK,
            splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
        )
        self.id1, self.id2, self.id6 = COLUMN_CATCH_HOLD_HIGH
        self.platform_high_hold = True
        self.platform_high_pose_sent = True
        self.chassis_station_stage = None
        self.chassis_station_done_reason = None
        self.chassis_station_error_reason = reason
        self.locked_target = None
        self.locked_plan = None
        self.column_target_armed = False
        self.column_capture_authorized = False
        self.status = (
            "COLUMN_CATCH arm frozen expanded after H7 hold failure "
            f"ID1={self.id1} ID2={self.id2} ID6={self.id6}"
        )
        self.arm_preview.publish(self.status)
        return f"FROZEN_EXPANDED_HIGH; {status}"

    def freeze_blue_task3_arm(self, reason="H7_HOLD_FAILED"):
        """Backward-compatible BLUE wrapper around the formal freeze path."""
        if self.field_mode != FieldMode.BLUE:
            return f"blue task-three freeze ignored outside BLUE field"
        return self.freeze_formal_task3_arm(reason)

    def _hold_platform_high_pose(self):
        """Keep the already-issued formal task-three high pose unchanged.

        The H7 route remains in COLUMN_CATCH while it travels to the white
        line. This branch must not run the generic target-search fallback,
        because that fallback intentionally changes the arm pose.
        """
        self.id1, self.id2, self.id6 = COLUMN_CATCH_HOLD_HIGH
        self.id7 = COLUMN_CATCH_GRIPPER_CLOSED_TICK
        self.id5 = COLUMN_CATCH_CATCHER_HOME_TICK
        self.splitter_id4 = COLUMN_CATCH_SPLITTER_TICK
        if not self.platform_high_pose_sent:
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            self.arm_preview.publish(
                "COLUMN_CATCH arm hold high "
                f"ID1={self.id1} ID2={self.id2} ID6={self.id6}"
            )
            self.platform_high_pose_sent = True
        return (
            "COLUMN_CATCH arm held expanded "
            f"ID1={self.id1} ID2={self.id2} ID6={self.id6}"
        )

    def abort_chassis_station(self, reason):
        """Best-effort home before reporting a station control failure to H7."""

        self.platform_task.reset()
        self.task3_ring_place_actions.clear()
        station = self.active_chassis_station or "UNKNOWN"
        result = self.shutdown_contract(
            raise_before_home=station == "DISC_CATCH"
        )
        home_ok = (
            not self.servo_bridge.write_enabled
            or self.servo_bridge.last_command_ok
        )
        self.active_chassis_station = None
        self.chassis_station_stage = None
        self.chassis_station_deadline = 0.0
        self.chassis_station_no_target_deadline = 0.0
        self.chassis_station_done_reason = None
        final_reason = str(reason)
        if home_ok:
            self._reset_cycle_for_search(
                f"chassis station {station} aborted; arm retracted: {final_reason}"
            )
        else:
            final_reason = f"{final_reason}_HOME_FAILED"
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.status = (
                f"chassis station {station} abort failed to home: {result}"
            )
            self.arm_preview.publish(self.status)
        self.chassis_station_error_reason = final_reason
        print(
            f"CHASSIS STATION {station} ABORT reason={final_reason} "
            f"home={'ok' if home_ok else 'failed'} | {result}",
            flush=True,
        )
        return result

    def update_chassis_station(
        self, station, detections, frame_shape, detection_fresh=True,
        chassis_link=None,
    ):
        if station == "PLATFORM_PICK":
            self.platform_task.tick(detections, frame_shape, fresh=detection_fresh)
            return self._sync_platform_task()
        if (
            station == "COLUMN_CATCH"
            and self.chassis_station_stage in {
                "task3_supplement_start",
                "task3_supplement_actions",
            }
        ):
            return self._update_task3_supplement()
        if station == "COLUMN_CATCH" and self.chassis_station_stage is not None:
            return self._update_column_catch_station(
                detections,
                frame_shape,
                detection_fresh=detection_fresh,
                chassis_link=chassis_link,
            )
        if station == "TASK3_RING_PLACE" and self.chassis_station_stage is not None:
            return self._update_task3_ring_place_station()
        if station != "DISC_CATCH" or self.chassis_station_stage is None:
            return None
        now = time.monotonic()
        ball = self._disc_catch_ball_visible(detections) if detection_fresh else None
        if detection_fresh and ball is not None:
            self.chassis_station_no_target_deadline = (
                now + DISC_CATCH_TARGET_TIMEOUT_S
            )
        if (
            self.chassis_station_stage == "disc_detect"
            and now >= self.chassis_station_no_target_deadline
            and ball is None
        ):
            self.state = "DISC_CATCH no target"
            colors = f"{self.field_mode.wire_name}_OR_YELLOW"
            return self._finish_chassis_station_after_retract(
                f"NO_{colors}_BALL_{DISC_CATCH_TARGET_TIMEOUT_S:.1f}S"
            )

        if self.chassis_station_stage == "disc_app_low_settle":
            self.state = "DISC_CATCH app low pose"
            if now < self.chassis_station_deadline:
                return f"DISC_CATCH app low pose settling {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "disc_detect"
            self.chassis_station_deadline = 0.0
            self.chassis_station_no_target_deadline = now + DISC_CATCH_TARGET_TIMEOUT_S
            return (
                "DISC_CATCH detecting "
                f"{self.field_mode.wire_name.lower()}/yellow ball at task point 1"
            )

        if self.chassis_station_stage == "disc_detect":
            self.state = "DISC_CATCH detect ball"
            if now < self.chassis_station_deadline:
                return f"DISC_CATCH descend settling {self.chassis_station_deadline - now:.1f}s"
            if ball is None:
                if (
                    self.field_mode == FieldMode.BLUE
                    and self.disc_pulse_done
                    and self.disc_last_pulsed_color == "blue"
                ):
                    self.disc_blue_clear_frames += 1
                    if self.disc_blue_clear_frames >= DISC_CATCH_BLUE_CLEAR_FRAMES:
                        self.disc_pulse_done = False
                        self.disc_last_pulsed_color = None
                        self.disc_last_pulsed_center = None
                        self.disc_blue_clear_frames = 0
                        return (
                            "DISC_CATCH blue target cleared; ready for next target frame"
                        )
                else:
                    self.disc_pulse_done = False
                    self.disc_last_pulsed_color = None
                    self.disc_last_pulsed_center = None
                remaining = max(0.0, self.chassis_station_no_target_deadline - now)
                return (
                    f"DISC_CATCH waiting {self.field_mode.wire_name.lower()}/yellow "
                    f"ball {remaining:.1f}s"
                )
            ball_color = ball.get("color")
            if ball_color not in self._disc_catch_allowed_colors():
                remaining = max(0.0, self.chassis_station_no_target_deadline - now)
                return (
                    f"DISC_CATCH rejected {ball_color} ball for "
                    f"field {self.field_mode.wire_name}; waiting allowed color "
                    f"{remaining:.1f}s"
                )
            if (
                self.field_mode == FieldMode.BLUE
                and ball.get("color") == "blue"
                and self.disc_pulse_done
                and self.disc_last_pulsed_color == "blue"
            ):
                self.disc_blue_clear_frames = 0
                return (
                    "DISC_CATCH blue target already pulsed; waiting for target "
                    "to leave the detection area"
                )
            if self.disc_pulse_done and ball_color == self.disc_last_pulsed_color:
                return (
                    f"DISC_CATCH already pulsed {ball_color} ball; waiting new target"
                )
            splitter_target = (
                DISC_CATCH_SPLITTER_YELLOW_TICK
                if ball_color == "yellow"
                else DISC_CATCH_SPLITTER_FIELD_TICK
            )
            catcher_target = (
                DISC_CATCH_CATCHER_YELLOW_TICK
                if ball_color == "yellow"
                else DISC_CATCH_CATCHER_FIELD_TICK
            )
            self.status = (
                f"DISC_CATCH {ball_color} ball detected at ID1={self.id1} "
                f"ID2={self.id2}; sync ID4 with ID7 open"
            )
            print(f"CHASSIS STATION {self.status}", flush=True)
            self.disc_fast_blue_cycle = self.field_mode == FieldMode.BLUE
            self.disc_blue_channel_hold_deadline = (
                now + DISC_CATCH_BLUE_CHANNEL_HOLD_S
                if self.disc_fast_blue_cycle
                else 0.0
            )
            if ball.get("center") and len(ball["center"]) >= 2:
                self.disc_last_pulsed_center = (
                    float(ball["center"][0]),
                    float(ball["center"][1]),
                )
            if not self.servo_bridge.write_enabled:
                self.disc_last_pulsed_color = ball_color
                self.disc_pulse_done = True
                self.splitter_id4 = splitter_target
                self.id5 = catcher_target
                self.id7 = DISC_CATCH_GRIPPER_OPEN_TICK
                self.chassis_station_stage = "disc_open_wait"
                self.chassis_station_deadline = now + self._disc_open_hold_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return (
                    f"preview DISC_CATCH ID14={self.splitter_id4} "
                    f"ID15={self.id5} ID17 open pulse"
                )
            trigger_status = self._send_disc_open_phase(
                splitter_target,
                catcher_target,
                "DISC_CATCH synchronized ID14/15/17 open phase",
            )
            if self.servo_bridge.last_command_ok:
                self.disc_last_pulsed_color = ball_color
                self.disc_pulse_done = True
                self.chassis_station_stage = "disc_open_wait"
                self.chassis_station_deadline = time.monotonic() + self._disc_open_hold_s()
            return trigger_status

        if self.chassis_station_stage == "disc_open":
            self.state = "DISC_CATCH open claw"
            if not self.servo_bridge.write_enabled:
                self.id7 = DISC_CATCH_GRIPPER_OPEN_TICK
                self.chassis_station_stage = "disc_open_wait"
                self.chassis_station_deadline = now + self._disc_open_hold_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview DISC_CATCH open ID7 pulse"
            status = self._send_gripper_id7(
                DISC_CATCH_GRIPPER_OPEN_TICK,
                "DISC_CATCH open ID7 pulse",
                motion_ms=(
                    DISC_CATCH_GRIPPER_TIME_MS
                    if self.disc_fast_blue_cycle
                    else DISC_CATCH_NORMAL_GRIPPER_TIME_MS
                ),
            )
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "disc_open_wait"
                self.chassis_station_deadline = time.monotonic() + self._disc_open_hold_s()
            return status

        if self.chassis_station_stage == "disc_open_wait":
            self.state = "DISC_CATCH open wait"
            if now < self.chassis_station_deadline:
                return f"DISC_CATCH open wait {self.chassis_station_deadline - now:.1f}s"
            # Complete the synchronized ID4/ID7 open phase before closing ID7.
            self.chassis_station_stage = "disc_close"

        if self.chassis_station_stage == "disc_close":
            self.state = "DISC_CATCH close claw"
            if not self.servo_bridge.write_enabled:
                self.id7 = self.id7_closed
                self.chassis_station_stage = "disc_close_confirm"
                self.chassis_station_deadline = now + DISC_CATCH_CLOSE_CONFIRM_DELAY_S
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview DISC_CATCH close ID7 pulse"
            status = self._send_gripper_id7(
                self.id7_closed,
                "DISC_CATCH close ID7 pulse",
                motion_ms=(
                    DISC_CATCH_GRIPPER_TIME_MS
                    if self.disc_fast_blue_cycle
                    else DISC_CATCH_NORMAL_GRIPPER_TIME_MS
                ),
            )
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "disc_close_confirm"
                self.chassis_station_deadline = (
                    time.monotonic() + DISC_CATCH_CLOSE_CONFIRM_DELAY_S
                )
            return status

        if self.chassis_station_stage == "disc_close_confirm":
            self.state = "DISC_CATCH close confirm"
            if now < self.chassis_station_deadline:
                return (
                    "DISC_CATCH close confirm wait "
                    f"{self.chassis_station_deadline - now:.2f}s"
                )
            if not self.servo_bridge.write_enabled:
                self.chassis_station_stage = "disc_close_wait"
                self.chassis_station_deadline = now + self._disc_gripper_motion_s()
                return "preview DISC_CATCH close ID7 confirm"
            status = self._send_gripper_id7(
                self.id7_closed,
                "DISC_CATCH close ID7 confirm",
                motion_ms=(
                    DISC_CATCH_GRIPPER_TIME_MS
                    if self.disc_fast_blue_cycle
                    else DISC_CATCH_NORMAL_GRIPPER_TIME_MS
                ),
            )
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "disc_close_wait"
                self.chassis_station_deadline = (
                    time.monotonic() + self._disc_gripper_motion_s()
                )
            return status

        if self.chassis_station_stage == "disc_close_wait":
            self.state = "DISC_CATCH close wait"
            if now < self.chassis_station_deadline:
                return f"DISC_CATCH close wait {self.chassis_station_deadline - now:.1f}s"
            if self.field_mode == FieldMode.BLUE:
                self.chassis_station_stage = "disc_blue_channel_wait"
                self.chassis_station_deadline = max(
                    now,
                    self.disc_blue_channel_hold_deadline,
                )
                return (
                    "DISC_CATCH blue-field ID17 closed; holding current ID14 "
                    f"channel for {max(0.0, self.chassis_station_deadline - now):.2f}s"
                )
            if self.disc_last_pulsed_color == "yellow":
                self.chassis_station_stage = "disc_yellow_reset"
                return "DISC_CATCH yellow pulse closed; resetting splitter"
            self.disc_pulse_done = False
            self.disc_last_pulsed_color = None
            self.disc_last_pulsed_center = None
            self.chassis_station_stage = "disc_detect"
            self.chassis_station_deadline = 0.0
            self.chassis_station_no_target_deadline = (
                time.monotonic() + DISC_CATCH_TARGET_TIMEOUT_S
            )
            return "DISC_CATCH pulse complete; ready for next target frame"

        if self.chassis_station_stage == "disc_blue_channel_wait":
            self.state = "DISC_CATCH blue channel hold"
            if now < self.chassis_station_deadline:
                return (
                    "DISC_CATCH blue-field ID14 target channel hold "
                    f"{self.chassis_station_deadline - now:.2f}s"
                )
            # A blue-ball pulse leaves ID14 on the blue channel.  Only a
            # yellow pulse needs the post-hold switch back to blue; switching
            # after blue was the source of repeated blue-channel triggers.
            status = "DISC_CATCH blue-field ID14 remains on blue channel"
            if self.disc_last_pulsed_color == "yellow":
                status = self._send_splitter_id4(
                    DISC_CATCH_SPLITTER_FIELD_TICK,
                    "DISC_CATCH blue-field ID14 switch yellow->blue",
                    splitter_time_ms=TASK1_ID14_TIME_MS,
                )
                if not self.servo_bridge.last_command_ok:
                    return status
            self.disc_blue_channel_hold_deadline = 0.0
            if self.disc_last_pulsed_color == "blue":
                self.disc_blue_clear_frames = 0
                self.chassis_station_stage = "disc_detect"
                self.chassis_station_deadline = 0.0
                self.chassis_station_no_target_deadline = (
                    time.monotonic() + DISC_CATCH_TARGET_TIMEOUT_S
                )
                return (
                    status + "; cooldown complete; waiting for blue target to clear"
                )
            self.disc_pulse_done = False
            self.disc_last_pulsed_color = None
            self.disc_last_pulsed_center = None
            self.chassis_station_stage = "disc_detect"
            self.chassis_station_deadline = 0.0
            self.chassis_station_no_target_deadline = (
                time.monotonic() + DISC_CATCH_TARGET_TIMEOUT_S
            )
            return status + "; cooldown complete; ready for next target frame"

        if self.chassis_station_stage == "disc_yellow_reset":
            self.state = "DISC_CATCH yellow splitter reset"
            if not self.servo_bridge.write_enabled:
                self.splitter_id4 = DISC_CATCH_SPLITTER_RESET_TICK
                self.chassis_station_stage = "disc_yellow_cooldown"
                self.chassis_station_deadline = (
                    now + DISC_CATCH_YELLOW_COOLDOWN_S
                )
                self.arm_preview.set_targets(
                    self.id1, self.id2, self.id7, self.id6
                )
                return "preview DISC_CATCH yellow splitter reset"
            status = self._send_splitter_id4(
                DISC_CATCH_SPLITTER_RESET_TICK,
                "DISC_CATCH yellow splitter reset",
                splitter_time_ms=TASK1_ID14_TIME_MS,
            )
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "disc_yellow_cooldown"
                self.chassis_station_deadline = (
                    time.monotonic() + DISC_CATCH_YELLOW_COOLDOWN_S
                )
            return status

        if self.chassis_station_stage == "disc_yellow_cooldown":
            self.state = "DISC_CATCH yellow cooldown"
            if now < self.chassis_station_deadline:
                return (
                    "DISC_CATCH yellow cooldown "
                    f"{self.chassis_station_deadline - now:.2f}s"
                )
            self.disc_pulse_done = False
            self.disc_last_pulsed_color = None
            self.disc_last_pulsed_center = None
            self.chassis_station_stage = "disc_detect"
            self.chassis_station_deadline = 0.0
            self.chassis_station_no_target_deadline = (
                time.monotonic() + DISC_CATCH_TARGET_TIMEOUT_S
            )
            return "DISC_CATCH pulse complete; ready for next target frame"

        return "DISC_CATCH station idle"

    def _update_column_catch_station(
        self,
        detections,
        frame_shape,
        detection_fresh=True,
        chassis_link=None,
    ):
        """Run the formal letter pause/center/grasp/resume transaction."""

        now = time.monotonic()
        if detection_fresh:
            self._column_update_handled_blocks(detections)
        block_target = (
            self._column_block_visible(detections) if detection_fresh else None
        )
        active_locked_block = self.column_locked_block
        if detection_fresh and self.column_locked_block is not None:
            tracked_blocks = [
                det for det in detections
                if det.get("kind") == "column_block"
                and det.get("fully_visible", True)
                and self._column_block_tracks_match(self.column_locked_block, det)
            ]
            if tracked_blocks:
                active_locked_block = min(
                    tracked_blocks,
                    key=lambda det: float(
                        np.linalg.norm(
                            np.asarray(det.get("center", ()), dtype=np.float32)
                            - np.asarray(self.column_locked_block.get("center", ()), dtype=np.float32)
                        )
                    ),
                )
                self.column_locked_block = self._copy_target(active_locked_block)
            else:
                # Keep the PAUSED block ROI for transient contour loss.  This
                # matches the standalone classifier: a dark glyph can hide or
                # split the white border for a frame, but an unrelated
                # full-frame letter must never become the grasp target.
                active_locked_block = self.column_locked_block
        # Before PAUSE, only the geometric block detector is authoritative.
        # After PAUSE, bind classification to the block that caused PAUSE;
        # never promote an unrelated full-frame letter into a grasp target.
        column_letter_candidates = (
            self._column_letters_in_block(detections, active_locked_block)
            if detection_fresh and active_locked_block is not None
            else []
        )
        if (
            detection_fresh
            and self.column_locked_block is not None
            and not column_letter_candidates
            and self.chassis_station_stage in {"column_classify", "column_centering"}
        ):
            recovered = self._column_locked_letter_fallback(
                detections,
                self.locked_target or self.last_visual_target,
                self.column_locked_block,
            )
            if recovered is not None:
                column_letter_candidates = [recovered]
        if (
            self.chassis_station_stage == "column_centering"
            and self.locked_target is not None
            and self.locked_target.get("kind") == "letter"
        ):
            locked_label = str(self.locked_target.get("letter", "")).upper()
            column_letter_candidates = [
                det for det in column_letter_candidates
                if str(det.get("letter", "")).upper() == locked_label
            ]
            if not column_letter_candidates:
                recovered = self._column_locked_letter_fallback(
                    detections, self.locked_target, active_locked_block
                )
                if recovered is not None:
                    column_letter_candidates = [recovered]
        any_letter_target = max(
            column_letter_candidates,
            key=lambda det: (
                float(det.get("confidence") or 0.0),
                float(det.get("projected_area") or 0.0),
            ),
            default=None,
        )
        selected_letter_candidates = [
            det for det in column_letter_candidates
            if self.target_policy.matches_column_letter(det, self.target_letters)
            and self._column_letter_quota_available(det.get("letter", ""))
        ]
        letter_target = max(
            selected_letter_candidates,
            key=lambda det: (
                float(det.get("confidence") or 0.0),
                float(det.get("projected_area") or 0.0),
            ),
            default=None,
        )
        stable_letter_target = None
        if self.chassis_station_stage == "column_classify" and detection_fresh:
            self.column_classify_votes.append(
                self._copy_target(any_letter_target)
                if any_letter_target is not None else None
            )
            self.column_classify_votes = self.column_classify_votes[
                -COLUMN_CLASSIFY_VOTE_WINDOW:
            ]
            vote_counts = {}
            for vote in self.column_classify_votes:
                if vote is not None:
                    label = str(vote.get("letter", "")).upper()
                    vote_counts[label] = vote_counts.get(label, 0) + 1
            if vote_counts:
                winner, votes = max(vote_counts.items(), key=lambda item: item[1])
                if (
                    votes >= COLUMN_CLASSIFY_MIN_VOTES
                    and self._column_rotation_votes_stable(
                        self.column_classify_votes, winner
                    )
                ):
                    stable_letter_target = next(
                        vote for vote in reversed(self.column_classify_votes)
                        if vote is not None
                        and str(vote.get("letter", "")).upper() == winner
                    )

        if self.splitter_id4 != COLUMN_CATCH_SPLITTER_TICK:
            if not self.servo_bridge.write_enabled:
                self.splitter_id4 = COLUMN_CATCH_SPLITTER_TICK
            else:
                status = self._column_splitter(
                    COLUMN_CATCH_SPLITTER_TICK,
                    "COLUMN_CATCH hold splitter",
                )
                if not self.servo_bridge.last_command_ok:
                    return status

        if self.chassis_station_stage == "column_ready":
            self.state = "COLUMN_CATCH ready"
            if now < self.chassis_station_deadline:
                return (
                    "COLUMN_CATCH ready settling "
                    f"{self.chassis_station_deadline - now:.1f}s"
                )
            # The orbit is already running. First acquire a geometric white
            # block; letter classification is intentionally deferred until
            # H7 is paused on a fresh frame.
            self.chassis_station_stage = "column_detect"
            self.column_target_armed = True
            return "COLUMN_CATCH detecting parallel white-edged block during orbit"

        if self.chassis_station_stage == "column_detect":
            self.state = "COLUMN_CATCH detect white-edged block"
            if not detection_fresh:
                return "COLUMN_CATCH waiting for fresh detection"
            if block_target is None:
                self.column_target_absent_frames += 1
                self.column_pending_target = None
                self.column_pending_frames = 0
                if self.column_target_absent_frames >= 3:
                    self.column_target_armed = True
                return "COLUMN_CATCH orbit detect; no parallel white-edged block"
            self.column_target_absent_frames = 0
            if not self.column_target_armed:
                return "COLUMN_CATCH waiting white block to leave before rearm"
            if self._column_blocks_match(self.column_pending_target, block_target):
                self.column_pending_frames += 1
            else:
                self.column_pending_target = self._copy_target(block_target)
                self.column_pending_frames = 1
            if self.column_pending_frames < COLUMN_BLOCK_CONFIRM_FRAMES:
                return (
                    "COLUMN_CATCH candidate confirmation "
                    f"{self.column_pending_frames}/{COLUMN_BLOCK_CONFIRM_FRAMES}"
                )
            if chassis_link is None:
                return self._column_abort("NO_FORMAL_CHASSIS_LINK")
            self.column_target_armed = False
            self.column_pending_target = None
            self.column_pending_frames = 0
            self.column_locked_block = self._copy_target(block_target)
            self.column_locked_distance_cm = None
            self.column_capture_authorized = False
            self.column_target_lost_since = None
            self.locked_target = self._copy_target(block_target)
            self.last_visual_target = self._copy_target(block_target)
            self.centered_frames = 0
            self.center_distance_samples = []
            self.visual_lost_frames = 0
            self.locked_plan = None
            self.chassis_station_stage = "column_pause_request"
            self.column_pause_deadline = now + COLUMN_H7_PAUSE_TIMEOUT_S
            self.status = (
                f"COLUMN_CATCH white block angle={block_target.get('angle', '?')} "
                "detected; request H7 PAUSE"
            )
            print(f"CHASSIS STATION {self.status}", flush=True)

        if self.chassis_station_stage == "column_pause_request":
            self.state = "COLUMN_CATCH waiting H7 PAUSED"
            if chassis_link is None:
                return self._column_abort("NO_FORMAL_CHASSIS_LINK")
            state = chassis_link.formal_column_pause_state()
            if state == "PAUSED":
                self.chassis_station_stage = "column_classify"
                self.column_classify_deadline = now + COLUMN_BLOCK_CLASSIFY_TIMEOUT_S
                self.column_classify_votes = []
                self.column_pause_deadline = 0.0
                return (
                    "COLUMN_CATCH H7 PAUSED; classify fresh letter "
                    f"for up to {COLUMN_BLOCK_CLASSIFY_TIMEOUT_S:.1f}s"
                )
            if state == "STOPPED":
                return self._column_abort("H7_STOPPED_DURING_PAUSE")
            if now >= self.column_pause_deadline:
                return self._column_timeout_recover(
                    "H7_PAUSE_TIMEOUT", chassis_link
                )
            chassis_link.request_formal_column_pause()
            return f"COLUMN_CATCH waiting H7 PAUSED state={state or 'NONE'}"

        if self.chassis_station_stage == "column_classify":
            self.state = "COLUMN_CATCH classify paused block"
            if stable_letter_target is not None:
                stable_selected = (
                    self.target_policy.matches_column_letter(
                        stable_letter_target, self.target_letters
                    )
                    and self._column_letter_quota_available(
                        stable_letter_target.get("letter", "")
                    )
                )
                if not stable_selected:
                    self.status = (
                        "COLUMN_CATCH ignored stable non-target letter "
                        f"{stable_letter_target.get('letter', '?')}; resume H7"
                    )
                    self.chassis_station_stage = "column_resume_abort"
                    return self.status
                self.locked_target = self._copy_target(stable_letter_target)
                letter_target = stable_letter_target
                # Only a paused, ROI-bound, selected letter can authorize
                # centering and the later open/retreat/descend stages.
                self.column_capture_authorized = True
                if stable_letter_target.get("distance_cm") is not None:
                    try:
                        self.column_locked_distance_cm = float(
                            stable_letter_target["distance_cm"]
                        )
                    except (TypeError, ValueError):
                        self.column_locked_distance_cm = None
                self.last_visual_target = self._copy_target(stable_letter_target)
                self.centered_frames = 0
                self.center_distance_samples = []
                self.visual_lost_frames = 0
                self.column_target_lost_since = None
                self.locked_plan = None
                self.chassis_station_stage = "column_centering"
                print(
                    f"CHASSIS STATION COLUMN_CATCH target letter "
                    f"{stable_letter_target.get('letter')} accepted after "
                    f"{COLUMN_CLASSIFY_MIN_VOTES}/{COLUMN_CLASSIFY_VOTE_WINDOW} "
                    "paused-frame votes",
                    flush=True,
                )
            elif now >= self.column_classify_deadline:
                self.status = "COLUMN_CATCH white block has no valid letter; resume H7"
                self.chassis_station_stage = "column_resume_abort"
                return self.status
            else:
                return (
                    "COLUMN_CATCH paused; waiting stable rotated-letter vote "
                    f"({len(self.column_classify_votes)}/"
                    f"{COLUMN_CLASSIFY_VOTE_WINDOW})"
                )

        if self.chassis_station_stage == "column_centering":
            self.state = "COLUMN_CATCH center target"
            if not self.column_capture_authorized:
                self.status = (
                    "COLUMN_CATCH blocked arm motion: no accepted letter; resume H7"
                )
                self.chassis_station_stage = "column_resume_abort"
                return self.status
            if not detection_fresh or letter_target is None:
                if self.column_target_lost_since is None:
                    self.column_target_lost_since = now
                lost_s = now - self.column_target_lost_since
                if lost_s >= COLUMN_BLOCK_TRACK_LOST_TIMEOUT_S:
                    self.chassis_station_stage = "column_resume_abort"
                    self.status = (
                        "COLUMN_CATCH target lost while paused for "
                        f"{lost_s:.1f}s; resume H7"
                    )
                    return self.status
                return (
                    "COLUMN_CATCH paused; waiting fresh letter for centering "
                    f"({lost_s:.1f}/{COLUMN_BLOCK_TRACK_LOST_TIMEOUT_S:.1f}s)"
                )
            self.column_target_lost_since = None
            letter_target = self._copy_target(letter_target)
            if letter_target.get("distance_cm") is None and self.column_locked_distance_cm is not None:
                letter_target["distance_cm"] = self.column_locked_distance_cm
            elif letter_target.get("distance_cm") is not None:
                try:
                    self.column_locked_distance_cm = float(letter_target["distance_cm"])
                except (TypeError, ValueError):
                    pass
            letter_target["task3_block_bbox"] = tuple(
                self.column_locked_block.get("bbox", ())
            ) if self.column_locked_block else ()
            self.locked_target = letter_target
            centered, message = self._visual_center_step(
                letter_target,
                frame_shape,
                now,
                now - self.last_preview_step_time >= self.preview_step_interval_s,
                "COLUMN_CATCH center",
            )
            if not centered:
                return message
            self.centered_frames += 1
            median_distance_cm = self._record_center_distance_sample(
                letter_target.get("distance_cm")
            )
            if self.centered_frames < max(1, self.stable_frames_required):
                return (
                    f"{message} confirm {self.centered_frames}/"
                    f"{self.stable_frames_required}"
                )
            locked = self._copy_target(letter_target)
            if median_distance_cm is not None:
                locked["distance_cm"] = median_distance_cm
            self.locked_target = locked
            plan, plan_text = self._post_center_plan(
                locked, frame_shape, "descend"
            )
            self.locked_plan = plan
            self.arm_preview.publish_plan_marker(plan, plan_text)
            if plan.get("ik_error_mm", float("inf")) > self.post_center_ik_error_mm:
                self.chassis_station_stage = "column_resume_abort"
                self.status = f"COLUMN_CATCH IK invalid; resume H7: {plan_text}"
                return self.status
            self.column_grab_id2 = int(self.id2)
            self.chassis_station_stage = "column_open"
            return f"COLUMN_CATCH centered; {plan_text}"

        if self.chassis_station_stage == "column_resume_abort":
            self.state = "COLUMN_CATCH resume after abort"
            self.column_capture_authorized = False
            if chassis_link is None:
                return self._column_abort("NO_FORMAL_CHASSIS_LINK")
            if self.column_resume_deadline <= 0.0:
                self.column_resume_deadline = now + COLUMN_H7_RESUME_TIMEOUT_S
            state = chassis_link.formal_column_pause_state()
            if state == "RESUMED":
                completed_label = str(
                    (self.locked_target or {}).get("letter", "")
                ).upper()
                self._column_record_handled_block(
                    completed_label, self.column_locked_block
                )
                self.locked_target = None
                self.locked_plan = None
                self.column_locked_block = None
                self.column_locked_distance_cm = None
                self.column_resume_deadline = 0.0
                self.column_target_lost_since = None
                self.chassis_station_stage = "column_detect"
                self.column_target_armed = False
                return "COLUMN_CATCH resumed after skipped target"
            if now >= self.column_resume_deadline:
                return self._column_timeout_recover(
                    "H7_RESUME_TIMEOUT_AFTER_SKIP", chassis_link
                )
            chassis_link.request_formal_column_resume()
            return "COLUMN_CATCH waiting H7 RESUMED after skipped target"

        if self.chassis_station_stage == "column_timeout_resume":
            self.state = "COLUMN_CATCH waiting H7 RESUMED after local recovery"
            if chassis_link is None:
                return self._column_abort("NO_FORMAL_CHASSIS_LINK")
            state = chassis_link.formal_column_pause_state()
            if state == "RESUMED":
                self.column_timeout_recovering = False
                self.column_resume_deadline = 0.0
                self.column_target_armed = False
                self.column_target_absent_frames = 0
                self.column_pending_target = None
                self.column_pending_frames = 0
                self.column_locked_block = None
                self.column_locked_distance_cm = None
                self.column_capture_authorized = False
                self.column_target_lost_since = None
                self.locked_target = None
                self.locked_plan = None
                self.last_visual_target = None
                self.chassis_station_stage = "column_detect"
                return "COLUMN_CATCH local recovery high; H7 RESUMED; continue orbit"
            if state == "STOPPED":
                return self._column_abort("H7_STOPPED_AFTER_LOCAL_RECOVERY")
            if self.column_resume_deadline <= 0.0:
                self.column_resume_deadline = now + COLUMN_H7_RESUME_TIMEOUT_S
            if now >= self.column_resume_deadline:
                self.chassis_station_stage = None
                self.state = "fault"
                self.algorithm_stage = "fault"
                self.chassis_station_error_reason = "H7_RESUME_TIMEOUT_AFTER_LOCAL_RECOVERY"
                self.status = f"COLUMN_CATCH {self.chassis_station_error_reason}"
                self.arm_preview.publish(self.status)
                return self.status
            chassis_link.request_formal_column_resume()
            return f"COLUMN_CATCH waiting H7 RESUMED after local recovery state={state or 'NONE'}"

        if self.chassis_station_stage == "column_open":
            self.state = "COLUMN_CATCH open claw"
            if not self.column_capture_authorized:
                self.status = (
                    "COLUMN_CATCH blocked open/descend: no accepted letter; resume H7"
                )
                self.chassis_station_stage = "column_resume_abort"
                return self.status
            if not self.servo_bridge.write_enabled:
                self.id7 = COLUMN_CATCH_GRIPPER_OPEN_TICK
                self.chassis_station_stage = "column_open_wait"
                self.chassis_station_deadline = now + self._aux_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview COLUMN_CATCH open ID7"
            status = self._column_gripper(
                COLUMN_CATCH_GRIPPER_OPEN_TICK,
                "COLUMN_CATCH open ID7 before descend",
            )
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_open_wait"
                self.chassis_station_deadline = time.monotonic() + self._aux_settle_s()
            return status

        if self.chassis_station_stage == "column_open_wait":
            self.state = "COLUMN_CATCH open wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH open wait {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_open_retreat"

        if self.chassis_station_stage == "column_open_retreat":
            self.state = "COLUMN_CATCH post-open ID2 retreat"
            if not self.column_capture_authorized:
                self.status = (
                    "COLUMN_CATCH blocked ID2 retreat: no accepted letter; resume H7"
                )
                self.chassis_station_stage = "column_resume_abort"
                return self.status
            previous_id2 = int(self.id2)
            target_id2 = max(
                self.id2_limits[0],
                previous_id2 - POST_OPEN_ID2_RETREAT_TICKS,
            )
            if not self.servo_bridge.write_enabled:
                self.id2 = target_id2
                self.chassis_station_stage = "column_open_retreat_wait"
                self.chassis_station_deadline = now + self._arm_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return (
                    f"preview COLUMN_CATCH post-open ID2 retreat "
                    f"ID2={previous_id2}->{target_id2} DELTA=-"
                    f"{POST_OPEN_ID2_RETREAT_TICKS}"
                )
            status = self.servo_bridge.send_targets(id2=target_id2)
            self.last_command_time = now
            if not self.servo_bridge.last_command_ok:
                self.state = "fault"
                self.algorithm_stage = "fault"
                return f"COLUMN_CATCH post-open ID2 retreat failed: {status}"
            self.id2 = target_id2
            self.chassis_station_stage = "column_open_retreat_wait"
            self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
            return (
                f"COLUMN_CATCH post-open ID2 retreat ID2={previous_id2}->{target_id2} "
                f"DELTA=-{POST_OPEN_ID2_RETREAT_TICKS} | {status}"
            )

        if self.chassis_station_stage == "column_open_retreat_wait":
            self.state = "COLUMN_CATCH post-open ID2 retreat wait"
            if now < self.chassis_station_deadline:
                return (
                    "COLUMN_CATCH post-open ID2 retreat settling "
                    f"{self.chassis_station_deadline - now:.1f}s"
                )
            self.chassis_station_stage = "column_descend"

        if self.chassis_station_stage == "column_descend":
            self.state = "COLUMN_CATCH IK descend"
            if not self.column_capture_authorized:
                self.status = (
                    "COLUMN_CATCH blocked IK descend: no accepted letter; resume H7"
                )
                self.chassis_station_stage = "column_resume_abort"
                return self.status
            if self.locked_plan is None:
                return self._column_abort("NO_VALID_IK_PLAN")
            target_id1 = self._clamp(self.locked_plan["id1"], self.id1_limits)
            target_id2 = self._clamp(self.locked_plan["id2"], self.id2_limits)
            target_id6 = self._clamp(self.id6, COLUMN_CATCH_ID6_CENTER_RANGE)
            status = self._column_pose(
                target_id1,
                target_id2,
                target_id6,
                "COLUMN_CATCH IK descend after ID2 retreat",
                raising=False,
                id7=COLUMN_CATCH_GRIPPER_OPEN_TICK,
                id5=COLUMN_CATCH_CATCHER_HOME_TICK,
                splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
            )
            if not self.servo_bridge.write_enabled:
                self.chassis_station_stage = "column_descend_wait"
                self.chassis_station_deadline = now + self._arm_settle_s()
            elif self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_descend_wait"
                self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
            else:
                self.state = "fault"
                self.algorithm_stage = "fault"
            return f"COLUMN_CATCH descend endpoint | {status}"

        if self.chassis_station_stage == "column_descend_wait":
            self.state = "COLUMN_CATCH descend wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH descend settling {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_close"

        if self.chassis_station_stage == "column_close":
            self.state = "COLUMN_CATCH close claw"
            if not self.servo_bridge.write_enabled:
                self.id7 = COLUMN_CATCH_GRIPPER_CLOSED_TICK
                self.chassis_station_stage = "column_close_wait"
                self.chassis_station_deadline = now + self._aux_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview COLUMN_CATCH close ID7"
            status = self._column_gripper(
                COLUMN_CATCH_GRIPPER_CLOSED_TICK,
                "COLUMN_CATCH close ID7 after descend",
            )
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_close_wait"
                self.chassis_station_deadline = time.monotonic() + self._aux_settle_s()
            return status

        if self.chassis_station_stage == "column_close_wait":
            self.state = "COLUMN_CATCH close wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH close wait {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_return_high"

        if self.chassis_station_stage == "column_return_high":
            self.state = "COLUMN_CATCH return high"
            status = self._column_pose(
                COLUMN_CATCH_READY_ID1_TICK,
                COLUMN_CATCH_READY_ID2_TICK,
                COLUMN_CATCH_READY_ID6_TICK,
                "COLUMN_CATCH return high before RESUME",
                raising=True,
                id7=COLUMN_CATCH_GRIPPER_CLOSED_TICK,
                id5=COLUMN_CATCH_CATCHER_HOME_TICK,
                splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
            )
            if not self.servo_bridge.write_enabled:
                self.chassis_station_stage = "column_return_high_wait"
                self.chassis_station_deadline = now + self._arm_settle_s()
            elif self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_return_high_wait"
                self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
            else:
                self.state = "fault"
                self.algorithm_stage = "fault"
            return f"COLUMN_CATCH return high | {status}"

        if self.chassis_station_stage == "column_return_high_wait":
            self.state = "COLUMN_CATCH high wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH high settling {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_place_work"

        if self.chassis_station_stage == "column_place_work":
            self.state = "COLUMN_CATCH letter place work pose"
            status = self._column_pose(
                COLUMN_CATCH_LETTER_PLACE[0],
                COLUMN_CATCH_LETTER_PLACE[1],
                COLUMN_CATCH_LETTER_PLACE[2],
                "COLUMN_CATCH letter placement pose",
                raising=False,
                id7=COLUMN_CATCH_GRIPPER_CLOSED_TICK,
                id5=COLUMN_CATCH_CATCHER_HOME_TICK,
                splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
            )
            if not self.servo_bridge.write_enabled:
                self.chassis_station_stage = "column_place_work_wait"
                self.chassis_station_deadline = now + COLUMN_CATCH_LETTER_PLACE_TIME_MS / 1000.0
            elif self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_place_work_wait"
                self.chassis_station_deadline = time.monotonic() + COLUMN_CATCH_LETTER_PLACE_TIME_MS / 1000.0
            return f"COLUMN_CATCH letter placement pose | {status}"

        if self.chassis_station_stage == "column_place_work_wait":
            self.state = "COLUMN_CATCH letter place wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH letter place wait {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_place_open"

        if self.chassis_station_stage == "column_place_open":
            self.state = "COLUMN_CATCH letter place open"
            status = self._column_gripper(
                COLUMN_CATCH_GRIPPER_OPEN_TICK,
                "COLUMN_CATCH open ID17 to place letter",
            )
            if not self.servo_bridge.write_enabled or self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_place_open_wait"
                self.chassis_station_deadline = time.monotonic() + self._aux_settle_s()
            return status

        if self.chassis_station_stage == "column_place_open_wait":
            self.state = "COLUMN_CATCH letter place open wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH letter place open wait {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_place_close"

        if self.chassis_station_stage == "column_place_close":
            self.state = "COLUMN_CATCH letter place close"
            status = self._column_gripper(
                COLUMN_CATCH_GRIPPER_CLOSED_TICK,
                "COLUMN_CATCH close ID17 after letter placement",
            )
            if not self.servo_bridge.write_enabled or self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_place_close_wait"
                self.chassis_station_deadline = time.monotonic() + self._aux_settle_s()
            return status

        if self.chassis_station_stage == "column_place_close_wait":
            self.state = "COLUMN_CATCH letter place close wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH letter place close wait {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_place_return_high"

        if self.chassis_station_stage == "column_place_return_high":
            self.state = "COLUMN_CATCH letter place return high"
            status = self._column_pose(
                COLUMN_CATCH_READY_ID1_TICK,
                COLUMN_CATCH_READY_ID2_TICK,
                COLUMN_CATCH_READY_ID6_TICK,
                "COLUMN_CATCH return high after letter placement",
                raising=True,
                id7=COLUMN_CATCH_GRIPPER_CLOSED_TICK,
                id5=COLUMN_CATCH_CATCHER_HOME_TICK,
                splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
            )
            if not self.servo_bridge.write_enabled or self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_place_return_high_wait"
                self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
            return f"COLUMN_CATCH return high after letter placement | {status}"

        if self.chassis_station_stage == "column_place_return_high_wait":
            self.state = "COLUMN_CATCH letter place high wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH letter place high wait {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_resume_request"

        if self.chassis_station_stage == "column_resume_request":
            self.state = "COLUMN_CATCH waiting H7 RESUMED"
            if chassis_link is None:
                return self._column_abort("NO_FORMAL_CHASSIS_LINK")
            state = chassis_link.formal_column_pause_state()
            if state == "RESUMED":
                completed_label = str(
                    (self.locked_target or {}).get("letter", "")
                ).upper()
                completed_capture = bool(self.column_capture_authorized)
                self._column_record_handled_block(
                    completed_label, self.column_locked_block
                )
                if (
                    completed_capture
                    and completed_label
                    and self._column_letter_quota_available(completed_label)
                ):
                    self.letter_success_counts[completed_label] = (
                        self.letter_success_counts.get(completed_label, 0) + 1
                    )
                    print(
                        "COLUMN_CATCH LETTER_SUCCESS "
                        f"letter={completed_label} "
                        f"count={self.letter_success_counts[completed_label]}/"
                        f"{PLATFORM_LETTER_SUCCESS_QUOTA}",
                        flush=True,
                    )
                self.locked_target = None
                self.locked_plan = None
                self.column_locked_block = None
                self.column_locked_distance_cm = None
                self.column_capture_authorized = False
                self.column_target_lost_since = None
                self.last_visual_target = None
                self.center_distance_samples = []
                self.centered_frames = 0
                self.column_resume_deadline = 0.0
                self.chassis_station_stage = "column_detect"
                self.column_target_armed = False
                return "COLUMN_CATCH grasp complete; H7 RESUMED; continue orbit"
            if state == "STOPPED":
                return self._column_abort("H7_STOPPED_BEFORE_RESUME")
            if self.column_resume_deadline <= 0.0:
                self.column_resume_deadline = now + COLUMN_H7_RESUME_TIMEOUT_S
            if now >= self.column_resume_deadline:
                return self._column_timeout_recover(
                    "H7_RESUME_TIMEOUT", chassis_link
                )
            chassis_link.request_formal_column_resume()
            return f"COLUMN_CATCH waiting H7 RESUMED state={state or 'NONE'}"

        return "COLUMN_CATCH station idle"

    def _column_timeout_recover(self, reason, chassis_link=None):
        """Return to task-three high and resume orbit after a local timeout."""
        high_status = self._column_pose(
            COLUMN_CATCH_READY_ID1_TICK,
            COLUMN_CATCH_READY_ID2_TICK,
            COLUMN_CATCH_READY_ID6_TICK,
            f"COLUMN_CATCH timeout recovery {reason}",
            raising=True,
            id7=COLUMN_CATCH_GRIPPER_CLOSED_TICK,
            id5=COLUMN_CATCH_CATCHER_HOME_TICK,
            splitter_id4=COLUMN_CATCH_SPLITTER_TICK,
        )
        home_ok = (
            not self.servo_bridge.write_enabled
            or self.servo_bridge.last_command_ok
        )
        if not home_ok:
            self.chassis_station_stage = None
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.chassis_station_error_reason = f"{reason}_HIGH_FAILED"
            self.status = f"COLUMN_CATCH {self.chassis_station_error_reason}"
            self.arm_preview.publish(self.status)
            return self.status

        self.id1 = COLUMN_CATCH_READY_ID1_TICK
        self.id2 = COLUMN_CATCH_READY_ID2_TICK
        self.id6 = COLUMN_CATCH_READY_ID6_TICK
        self.id7 = COLUMN_CATCH_GRIPPER_CLOSED_TICK
        self.id5 = COLUMN_CATCH_CATCHER_HOME_TICK
        self.splitter_id4 = COLUMN_CATCH_SPLITTER_TICK
        self.column_capture_authorized = False
        self.column_target_lost_since = None
        self.column_locked_block = None
        self.column_locked_distance_cm = None
        self.locked_target = None
        self.locked_plan = None
        self.column_timeout_recovering = True
        self.chassis_station_stage = "column_timeout_resume"
        self.status = f"COLUMN_CATCH {reason}; arm high and closed; resume orbit"
        self.arm_preview.publish(self.status)
        if chassis_link is not None:
            try:
                chassis_link.request_formal_column_resume()
            except Exception:
                pass
        print(f"CHASSIS STATION {self.status} | {high_status}", flush=True)
        return self.status

    def _column_abort(self, reason):
        self.column_capture_authorized = False
        self.state = "fault"
        self.algorithm_stage = "fault"
        self.status = f"COLUMN_CATCH {reason}"
        self.arm_preview.publish(self.status)
        return self.status

    def retract_for_chassis_timeout(self, station):
        result = self.shutdown_contract()
        success = (
            not self.servo_bridge.write_enabled
            or self.servo_bridge.last_command_ok
        )
        self.chassis_station_stage = None
        self.active_chassis_station = None
        if success:
            self.chassis_station_error_reason = None
        else:
            self.chassis_station_error_reason = "HOME_FAILED_AFTER_TIMEOUT"
        self._reset_cycle_for_search(
            f"chassis station {station} no target timeout; arm retracted"
        )
        if not success:
            self.state = "fault"
            self.algorithm_stage = "fault"
        return result

    def shutdown_contract(
        self,
        catcher_target=None,
        splitter_target=None,
        *,
        raise_before_home=False,
    ):
        if not self.enabled or not self.servo_bridge.write_enabled:
            return "shutdown contract skipped; servo writes disabled"
        task1_contract = bool(
            raise_before_home or self.active_chassis_station == "DISC_CATCH"
        )
        home_id5 = (
            TASK1_ID15_RETRACT_TICK if task1_contract else CATCHER_HOME_TICK
        )
        home_splitter = (
            TASK1_ID14_RETRACT_TICK
            if task1_contract
            else SPLITTER_RETRACT_TICK
        )
        aux_timing = (
            {
                "aux_time_ms": TASK1_AUX_TIME_MS,
                "splitter_time_ms": TASK1_ID14_TIME_MS,
            }
            if task1_contract
            else {}
        )
        self.id7 = self.id7_closed
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish("shutdown contract close claw")
        print(
            "SHUTDOWN CONTRACT close claw first "
            f"ID7={self.id7}",
            flush=True,
        )
        previous_gripper_time = getattr(
            self.servo_bridge, "gripper_time_ms", PLATFORM_GRIPPER_TIME_MS
        )
        try:
            if task1_contract:
                self.servo_bridge.gripper_time_ms = DISC_CATCH_NORMAL_GRIPPER_TIME_MS
            claw_status = self.servo_bridge.send_targets(id4=self.id7)
        finally:
            self.servo_bridge.gripper_time_ms = previous_gripper_time
        self.last_command_time = time.monotonic()
        if not self.servo_bridge.last_command_ok:
            self.status = f"shutdown contract close claw failed: {claw_status}"
            self.arm_preview.publish(self.status)
            print(self.status, flush=True)
            return self.status
        time.sleep(max(0.10, min(1.0, self._aux_settle_s())))

        high_status = "not requested"
        if raise_before_home:
            high_status = self._send_fixed_arm_pose_staged(
                *PLATFORM_HIGH_POSE,
                "DISC_CATCH post-catch high",
                raising=True,
                id7=self.id7,
                id5=home_id5,
                splitter_id4=(
                    home_splitter
                    if splitter_target is None
                    else int(splitter_target)
                ),
                joint_sequence_delay_s=0.10 if task1_contract else None,
                first_arm_time_ms=500 if task1_contract else None,
                second_arm_time_ms=500 if task1_contract else None,
                **aux_timing,
            )
            if not self.servo_bridge.last_command_ok:
                self.status = f"shutdown contract post-catch high failed: {high_status}"
                self.arm_preview.publish(self.status)
                print(self.status, flush=True)
                return self.status
            time.sleep(max(0.15, min(1.2, self._arm_settle_s())))

        # After task-one completion, retract ID2 and ID6 together only after
        # the high pose has settled; return ID1 last to clear the work area.
        self.id2 = HOME_ID2_TICK
        self.id6 = BASE_YAW_HOME_TICK
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish("shutdown contract ID2 ID6 home first")
        print(
            f"SHUTDOWN CONTRACT ID2_ID6_FIRST ID2={self.id2} ID6={self.id6} "
            "wait_ms=200",
            flush=True,
        )
        previous_arm_time = getattr(
            self.servo_bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS
        )
        try:
            self.servo_bridge.arm_time_ms = PLATFORM_ARM_TIME_MS
            id2_id6_status = self.servo_bridge.send_targets(
                id2=self.id2,
                id6=self.id6,
            )
        finally:
            self.servo_bridge.arm_time_ms = previous_arm_time
        self.last_command_time = time.monotonic()
        if not self.servo_bridge.last_command_ok:
            self.status = f"shutdown contract ID2/ID6 failed: {id2_id6_status}"
            self.arm_preview.publish(self.status)
            print(self.status, flush=True)
            return self.status
        time.sleep(0.10 if task1_contract else ARM_JOINT_SEQUENCE_DELAY_S)

        self.id1 = HOME_ID1_TICK
        self.id6 = BASE_YAW_HOME_TICK
        self.id5 = int(
            home_id5 if catcher_target is None else catcher_target
        )
        self.splitter_id4 = int(
            home_splitter
            if splitter_target is None
            else splitter_target
        )
        self._enforce_angle_gap()
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish("shutdown contract arm home")
        print(
            "SHUTDOWN CONTRACT "
            f"ID1={self.id1} ID2={self.id2} ID3={TASK1_ID3_RETRACT_TICK} "
            f"ID4={self.splitter_id4} "
            f"ID5={self.id5} ID6={self.id6} ID7={self.id7}",
            flush=True,
        )
        previous_arm_time = getattr(
            self.servo_bridge, "arm_time_ms", PLATFORM_ARM_TIME_MS
        )
        try:
            self.servo_bridge.arm_time_ms = PLATFORM_ARM_TIME_MS
            status = self.servo_bridge.send_targets(
                id1=self.id1,
                id3=TASK1_ID3_RETRACT_TICK,
                id4=self.id7,
                id6=self.id6,
                id5=self.id5,
                splitter_id4=self.splitter_id4,
                **aux_timing,
            )
        finally:
            self.servo_bridge.arm_time_ms = previous_arm_time
        self.last_command_time = time.monotonic()
        if not self.servo_bridge.last_command_ok:
            self.status = f"shutdown contract failed: {status}"
            self.arm_preview.publish(self.status)
            print(self.status, flush=True)
            return self.status
        self.status = (
            f"shutdown contracted: claw={claw_status}; "
            f"high={high_status}; id2_id6_first={id2_id6_status}; arm={status}"
        )
        self.arm_preview.publish(self.status)
        time.sleep(max(0.15, min(1.2, self._arm_settle_s())))
        return self.status

    def _startup_fault(self, detail):
        self.startup_stage = "fault"
        self.state = "fault"
        self.feedback_pending = False
        self.status = f"startup fault: {detail}"
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        return self.status

    def _startup_send_targets(self, id1, id2, id4, label, *, raising):
        self.id7 = int(id4)
        self.id6 = BASE_YAW_CENTER_TICK
        self.splitter_id4 = SPLITTER_RETRACT_TICK
        self.id5 = CATCHER_HOME_TICK
        status = self._send_fixed_arm_pose_staged(
            id1,
            id2,
            self.id6,
            label,
            raising=raising,
            id7=self.id7,
            id5=self.id5,
            splitter_id4=self.splitter_id4,
        )
        if not self.servo_bridge.last_command_ok:
            return self._startup_fault(f"{label} {status}")
        self.status = (
            f"{label} ACK ID1={self.id1} ID2={self.id2} "
            f"ID4={self.splitter_id4} ID5={self.id5} "
            f"ID6={self.id6} ID7={self.id7}"
        )
        return self.status

    def _startup_positions_ready(self, feedback):
        if feedback is None:
            return False
        return (
            abs(feedback[0] - READY_ID1_TICK) <= self.startup_position_tolerance
            and abs(feedback[1] - READY_ID2_TICK) <= self.startup_position_tolerance
        )

    def _update_startup(self, now):
        if self.startup_stage == "fault":
            return self.status

        if self.startup_stage == "wait_ready":
            self.state = "startup_wait_ready"
            if not self.servo_bridge.wait_ready(0.12):
                if now >= self.startup_ready_timeout:
                    return self._startup_fault("Hiwonder HTD85 board ready timeout")
                return "startup_wait_ready: waiting for Hiwonder HTD85 board"
            feedback = self.servo_bridge.command_arm_ready()
            if feedback is None:
                return self._startup_fault(self.servo_bridge.status)
            self.id7 = self.id7_closed
            self.id6 = BASE_YAW_CENTER_TICK
            self.splitter_id4 = SPLITTER_RETRACT_TICK
            self.id5 = CATCHER_HOME_TICK
            yaw_status = self.servo_bridge.send_targets(id6=self.id6)
            if not self.servo_bridge.last_command_ok:
                return self._startup_fault(f"startup ID6 center {yaw_status}")
            aux_status = self.servo_bridge.send_targets(
                splitter_id4=self.splitter_id4,
                id5=self.id5,
            )
            if not self.servo_bridge.last_command_ok:
                return self._startup_fault(f"startup auxiliaries home {aux_status}")
            claw_status = self.servo_bridge.send_targets(id4=self.id7_closed)
            if not self.servo_bridge.last_command_ok:
                return self._startup_fault(f"startup claw close {claw_status}")
            if not self._apply_feedback(feedback):
                return self._startup_fault(self.status)
            if getattr(self.servo_bridge, "assumed_feedback", False):
                self.startup_stage = "direct_ready_settle"
                self.startup_deadline = time.monotonic() + max(
                    self._arm_settle_s(),
                    self._aux_settle_s(),
                )
                self.state = "startup_direct_ready_settle"
                self.status = (
                    f"startup_ready commands sent; settling ID1={self.id1} "
                    f"ID2={self.id2} ID4={self.splitter_id4} ID5={self.id5} "
                    f"ID6={self.id6} ID7={self.id7}"
                )
                return self.status
            self.startup_stage = "complete"
            self.state = "startup_ready"
            self.status = (
                f"startup_ready atomic ID1={self.id1} "
                f"ID2={self.id2} ID4={self.splitter_id4} ID5={self.id5} "
                f"ID6={self.id6} ID7={self.id7}"
            )
            return self.status

        if self.startup_stage == "direct_ready_settle":
            self.state = "startup_direct_ready_settle"
            if now < self.startup_deadline:
                return f"startup_direct_ready_settle {self.startup_deadline - now:.1f}s"
            self.startup_stage = "complete"
            self.state = "startup_ready"
            self.status = (
                f"startup_ready settled ID1={self.id1} ID2={self.id2} "
                f"ID4={self.splitter_id4} ID5={self.id5} "
                f"ID6={self.id6} ID7={self.id7}"
            )
            return self.status

        if self.startup_stage == "send_home":
            self.state = "startup_home"
            result = self._startup_send_targets(
                HOME_ID1_TICK,
                HOME_ID2_TICK,
                self.id7_closed,
                "startup_home",
                raising=False,
            )
            if self.startup_stage == "fault":
                return result
            id3_status = "preview"
            if self.servo_bridge.write_enabled:
                id3_status = self.servo_bridge.send_aux_request(
                    servo_id=3,
                    pulse=TASK1_ID3_RETRACT_TICK,
                    time_ms=TASK1_ID3_RETRACT_TIME_MS,
                )
                if not self.servo_bridge.last_command_ok:
                    return self._startup_fault(
                        f"startup ID3 retract failed: {id3_status}"
                    )
                print(
                    f"STARTUP HOME ID3={TASK1_ID3_RETRACT_TICK} "
                    f"time={TASK1_ID3_RETRACT_TIME_MS}ms",
                    flush=True,
                )
            self.startup_deadline = now + max(0.8, self._arm_settle_s())
            self.startup_stage = "home_settle"
            return f"{result}; ID3={TASK1_ID3_RETRACT_TICK} ({id3_status})"

        if self.startup_stage == "home_settle":
            self.state = "startup_home"
            if now < self.startup_deadline:
                return f"startup_home settling {self.startup_deadline - now:.1f}s"
            hold_s = 0.4 if getattr(self.servo_bridge, "assumed_feedback", False) else 5.0
            self.startup_deadline = now + hold_s
            self.startup_stage = "home_hold"

        if self.startup_stage == "home_hold":
            self.state = "startup_home_hold"
            if now < self.startup_deadline:
                return f"startup_home_hold {self.startup_deadline - now:.1f}s"
            self.startup_stage = "send_ready"

        if self.startup_stage == "send_ready":
            self.state = "startup_extend"
            result = self._startup_send_targets(
                READY_ID1_TICK,
                READY_ID2_TICK,
                self.id7_closed,
                "startup_extend",
                raising=True,
            )
            if self.startup_stage == "fault":
                return result
            self.startup_deadline = now + max(0.8, self._arm_settle_s())
            self.startup_verify_deadline = now + max(2.0, self._arm_settle_s() + 0.8)
            self.startup_next_feedback = self.startup_deadline
            self.startup_ready_resends = 0
            self.startup_stage = "ready_settle"
            return result

        if self.startup_stage == "ready_settle":
            self.state = "startup_extend"
            if now < self.startup_deadline:
                return f"startup_extend settling {self.startup_deadline - now:.1f}s"
            self.startup_stage = "verify_ready"
            self.startup_feedback_attempts = 0

        if self.startup_stage == "verify_ready":
            self.state = "startup_verify"
            if now < self.startup_next_feedback:
                return f"startup_verify waiting {self.startup_next_feedback - now:.1f}s"
            feedback = self.servo_bridge.query_positions()
            self.startup_feedback_attempts += 1
            if self._startup_positions_ready(feedback):
                self.id7 = self.id7_closed
                if not self._apply_feedback(feedback):
                    return self._startup_fault(self.status)
                self.startup_stage = "complete"
                self.state = "startup_ready"
                self.status = (
                    f"startup_ready ID1={self.id1} ID2={self.id2} "
                    f"ID4={self.splitter_id4} ID5={self.id5} "
                    f"ID6={self.id6} ID7={self.id7}"
                )
                return self.status
            if now >= self.startup_verify_deadline:
                return self._startup_fault(
                    f"ready feedback outside +/-{self.startup_position_tolerance}: "
                    f"{feedback or self.servo_bridge.status}"
                )
            if self.startup_ready_resends < 2:
                resend_status = self.servo_bridge.send_targets(
                    id1=READY_ID1_TICK,
                    id2=READY_ID2_TICK,
                    id6=BASE_YAW_CENTER_TICK,
                )
                if not self.servo_bridge.last_command_ok:
                    return self._startup_fault(f"ready resend {resend_status}")
                self.startup_ready_resends += 1
            self.startup_next_feedback = time.monotonic() + 1.0
            return (
                f"startup_verify moving feedback={feedback} "
                f"attempt={self.startup_feedback_attempts} "
                f"resend={self.startup_ready_resends}/2"
            )

        return self.status

    @staticmethod
    def _copy_target(target):
        if target is None:
            return None
        copied = dict(target)
        if "center" in copied:
            copied["center"] = tuple(copied["center"])
        if "bbox" in copied:
            copied["bbox"] = tuple(copied["bbox"])
        return copied

    def _target_ready(self, target):
        label = "target" if target is None else f"{target.get('color', '')} {target.get('kind', 'target')}".strip()
        if target is None:
            return False, "grasp searching target"
        if not target.get("fully_visible", True):
            return False, f"{label} touches frame edge; move it fully into view"
        if target.get("area_percent", 0.0) < self.min_target_area_percent:
            return (
                False,
                f"{label} area below {self.min_target_area_percent:.2f}%; grasp disabled",
            )
        return True, f"{label} locked"

    @staticmethod
    def _target_size_mm(target):
        if target.get("kind") == "ball":
            return GOLF_BALL_DIAMETER_MM
        if target.get("kind") == "ring":
            return RED_RING_OUTER_DIAMETER_MM
        return LETTER_CUBE_SIDE_MM

    @staticmethod
    def _estimate_target_offsets_mm(target, frame_shape):
        distance_cm = target.get("distance_cm")
        if distance_cm is None:
            return None
        distance_mm = distance_cm * 10.0
        height, width = frame_shape[:2]
        cx, cy = target.get("center", (width / 2.0, height / 2.0))
        _, _, bbox_w, bbox_h = target.get("bbox", (0, 0, 0, 0))
        apparent_side_px = max(1.0, float(max(bbox_w, bbox_h)))
        target_size_mm = TargetGraspController._target_size_mm(target)
        focal_px = apparent_side_px * distance_mm / target_size_mm
        lateral_mm = (float(cx) - (width / 2.0)) * distance_mm / focal_px
        vertical_mm = (float(cy) - (height / 2.0)) * distance_mm / focal_px
        return {
            "distance_mm": distance_mm,
            "focal_px": focal_px,
            "lateral_mm": lateral_mm,
            "vertical_mm": vertical_mm,
            "center_x_px": float(cx),
            "center_y_px": float(cy),
            "target_kind": target.get("kind", "letter"),
            "target_source": target.get("source"),
            "target_size_mm": target_size_mm,
        }

    def _one_shot_plan(self, target, frame_shape):
        offsets = self._estimate_target_offsets_mm(target, frame_shape)
        if offsets is None:
            return None, "one-shot locked; waiting distance estimate"

        lateral_mm = offsets["lateral_mm"]
        if abs(lateral_mm) > self.max_lateral_offset_mm:
            return None, (
                f"target lateral offset {lateral_mm:.0f}mm exceeds "
                f"{self.max_lateral_offset_mm:.0f}mm; center with base first"
            )

        distance_cm = offsets["distance_mm"] / 10.0
        try:
            solved_id1, solved_id2 = calibrated_grasp_ticks(distance_cm)
        except ValueError as exc:
            return None, str(exc)
        plan = {
            **offsets,
            "forward_mm": 0.0,
            "vertical_target_mm": 0.0,
            "target_distance_mm": offsets["distance_mm"],
            "current_x_mm": 0.0,
            "current_z_mm": 0.0,
            "id1": solved_id1,
            "id2": solved_id2,
            "ik_error_mm": 0.0,
            "calibration_model": "measured_7_30cm",
        }
        return plan, (
            f"measured model distance={distance_cm:.1f}cm "
            f"lat={lateral_mm:.0f}mm -> ID1={solved_id1} ID2={solved_id2}"
        )

    def _post_center_plan(self, target=None, frame_shape=None, phase="descend"):
        offsets = self._estimate_target_offsets_mm(target, frame_shape) \
            if target is not None and frame_shape is not None else None
        if offsets is None:
            return {
                "id1": self.id1,
                "id2": self.id2,
                "ik_error_mm": float("inf"),
                "progress_ratio": 0.0,
                "phase": phase,
            }, "measured grasp model waiting for target distance"

        distance_cm = offsets["distance_mm"] / 10.0
        try:
            measured_id1, measured_id2 = calibrated_grasp_ticks(distance_cm)
        except ValueError as exc:
            return {
                **offsets,
                "id1": self.id1,
                "id2": self.id2,
                "ik_error_mm": float("inf"),
                "progress_ratio": 0.0,
                "phase": phase,
            }, str(exc)

        if phase == "overhead":
            solved_id1, solved_id2 = self.id1, self.id2
        else:
            solved_id1 = self._clamp(measured_id1, self.id1_limits)
            solved_id2 = self._clamp(measured_id2, self.id2_limits)
        plan = {
            **offsets,
            "current_x_mm": 0.0,
            "current_z_mm": 0.0,
            "forward_mm": 0.0,
            "vertical_target_mm": 0.0,
            "requested_forward_mm": 0.0,
            "requested_vertical_mm": 0.0,
            "progress_ratio": 1.0,
            "target_distance_mm": offsets["distance_mm"],
            "id1": solved_id1,
            "id2": solved_id2,
            "ik_error_mm": 0.0,
            "phase": phase,
            "calibration_model": "measured_7_30cm",
        }
        return plan, (
            f"measured model distance={distance_cm:.1f}cm phase={phase} "
            f"ID1={solved_id1} ID2={solved_id2}"
        )

    def _post_open_retreat_target(self):
        """Move ID2 back by 100 ticks while ID7 is open."""
        source_id1 = self._clamp(self.id1, self.id1_limits)
        source_id2 = self._clamp(self.id2, self.id2_limits)
        target_id2 = self._clamp(
            source_id2 - POST_OPEN_ID2_RETREAT_TICKS,
            self.id2_limits,
        )
        target_id1, target_id2 = enforce_angle_gap(
            source_id1,
            target_id2,
            self.id2_limits,
            self.angle_gap_degrees,
        )
        text = (
            "post-open ID2 retreat "
            f"ID1={source_id1}->{target_id1} "
            f"ID2={source_id2}->{target_id2} "
            f"DELTA=-{POST_OPEN_ID2_RETREAT_TICKS}"
        )
        return target_id1, target_id2, text

    def _update_locked_grasp(
        self,
        frame_shape,
        now,
        can_preview_step,
        live_target=None,
        live_target_fresh=True,
    ):
        can_command = now - self.last_command_time >= self.command_interval_s

        if self.algorithm_stage == "open":
            self.state = "locked target open claw"
            if not self.servo_bridge.write_enabled:
                next_id4 = min(self.id7_open, self.id7 + 20)
                if can_preview_step:
                    self.id7 = next_id4
                    self.last_preview_step_time = now
                if self.id7 >= self.id7_open:
                    self.id7 = self.id7_open
                    if self.simple_vertical_grasp:
                        self.algorithm_stage = "vertical_wait_open"
                        self.stage_deadline = time.monotonic() + self._aux_settle_s()
                    elif self.post_center_direct_descend:
                        self.next_stage_after_open = "post_open_retreat"
                        self.next_stage_after_retreat = "descend"
                        self.algorithm_stage = "open_wait"
                        self.stage_deadline = time.monotonic() + self._aux_settle_s()
                    else:
                        self.next_stage_after_open = "post_lock_visual_confirm"
                        self.algorithm_stage = "open_wait"
                        self.stage_deadline = time.monotonic() + self._aux_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return (
                    f"preview locked target; opening claw "
                    f"ID7={self.id7}->{self.id7_open}"
                )
            if can_command:
                self.state = "claw_open_ack"
                result = self._send_gripper_id7(self.id7_open, "locked target; open claw")
                if self.servo_bridge.last_command_ok:
                    if self.simple_vertical_grasp:
                        self.algorithm_stage = "vertical_wait_open"
                        self.stage_deadline = time.monotonic() + self._aux_settle_s()
                    elif self.post_center_direct_descend:
                        self.next_stage_after_open = "post_open_retreat"
                        self.next_stage_after_retreat = "descend"
                        self.algorithm_stage = "open_wait"
                        self.stage_deadline = time.monotonic() + self._aux_settle_s()
                    else:
                        self.next_stage_after_open = "post_lock_visual_confirm"
                        self.algorithm_stage = "open_wait"
                        self.stage_deadline = time.monotonic() + self._aux_settle_s()
                return result
            return "locked target; waiting to open claw"

        if self.algorithm_stage == "open_wait":
            self.state = "locked target waiting open claw"
            if now < self.stage_deadline:
                return f"locked target; waiting open claw {self.stage_deadline - now:.1f}s"
            if self.next_stage_after_open is not None:
                next_stage = self.next_stage_after_open
            elif self.simple_vertical_grasp:
                next_stage = "vertical_descend"
            elif self.post_center_direct_descend:
                next_stage = "post_open_retreat"
                self.next_stage_after_retreat = "descend"
            else:
                next_stage = "post_lock_visual_confirm"
            print(
                "GRASP STAGE open_wait complete "
                f"next={next_stage} direct_descend={self.post_center_direct_descend} "
                f"locked={self.locked_target is not None}",
                flush=True,
            )
            self.algorithm_stage = next_stage
            self.next_stage_after_open = None

        if self.algorithm_stage == "post_open_retreat":
            self.state = "post-open ID2 retreat"
            if self.locked_plan is None:
                plan, plan_text = self._post_center_plan(
                    self.locked_target,
                    frame_shape,
                    "descend",
                )
                self.locked_plan = plan
                self.arm_preview.publish_plan_marker(plan, plan_text)
                print(
                    "GRASP LOCK BEFORE ID2 RETREAT "
                    f"target={json.dumps(self.locked_target, ensure_ascii=True, default=str)} "
                    f"plan={plan_text}",
                    flush=True,
                )
                if plan["ik_error_mm"] > self.post_center_ik_error_mm:
                    self.state = "post-center unreachable"
                    self.algorithm_stage = "fault"
                    self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                    return f"claw opened; locked target IK failed before ID2 retreat: {plan_text}"
            target_id1, target_id2, retreat_text = self._post_open_retreat_target()
            if target_id2 >= self.id2:
                return self._abort_to_standby(
                    f"post-open ID2 retreat blocked: {retreat_text}"
                )
            if not self.servo_bridge.write_enabled:
                if can_preview_step:
                    self.id1 = target_id1
                    self.id2 = target_id2
                    self.last_preview_step_time = now
                    self.algorithm_stage = "post_open_retreat_wait"
                    self.stage_deadline = time.monotonic() + self._arm_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return f"preview {retreat_text}"
            if not can_command:
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return f"waiting {retreat_text}"
            self.id1 = target_id1
            self.id2 = target_id2
            result = self._send(
                f"{retreat_text}; keep ID7 open before IK descend",
                require_feedback=True,
            )
            if self.servo_bridge.last_command_ok:
                self.algorithm_stage = "post_open_retreat_wait"
                self.stage_deadline = time.monotonic() + self._arm_settle_s()
            return result

        if self.algorithm_stage == "post_open_retreat_wait":
            self.state = "post-open ID2 retreat waiting"
            if now < self.stage_deadline:
                return f"post-open ID2 retreat waiting {self.stage_deadline - now:.1f}s"
            next_stage = self.next_stage_after_retreat or "descend"
            print(
                "GRASP STAGE post_open_retreat complete "
                f"next={next_stage} ID1={self.id1} ID2={self.id2} ID6={self.id6}",
                flush=True,
            )
            self.algorithm_stage = next_stage
            self.next_stage_after_retreat = None

        if self.algorithm_stage == "post_lock_visual_confirm":
            self.state = "post-lock visual confirm"
            if not live_target_fresh:
                return "post-lock visual confirm waiting for fresh detection"
            accepted_target, reason = self._accept_live_target(live_target)
            if accepted_target is None:
                self.visual_confirm_frames = 0
                if self.visual_lost_frames <= max(3, self.stable_frames_required):
                    self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                    return f"post-lock visual confirm waiting: {reason}"
                return self._abort_to_standby(f"post-lock visual confirm failed: {reason}")
            centered, message = self._visual_center_step(
                accepted_target,
                frame_shape,
                now,
                can_preview_step,
                "post-lock visual confirm",
            )
            if not centered:
                self.visual_confirm_frames = 0
                if "blocked by limits" in message:
                    return self._abort_to_standby(message)
                return message
            self.visual_confirm_frames += 1
            if self.visual_confirm_frames < max(2, self.stable_frames_required):
                return (
                    f"{message} confirm "
                    f"{self.visual_confirm_frames}/{max(2, self.stable_frames_required)}"
                )
            plan, plan_text = self._post_center_plan(
                accepted_target,
                frame_shape,
                "descend",
            )
            self.locked_plan = plan
            self.arm_preview.publish_plan_marker(plan, plan_text)
            print(
                "GRASP VISUAL LOCK "
                f"target={json.dumps(self._copy_target(accepted_target), ensure_ascii=True, default=str)} "
                f"plan={plan_text}",
                flush=True,
            )
            if plan["ik_error_mm"] > self.post_center_ik_error_mm:
                self.state = "post-center unreachable"
                self.algorithm_stage = "fault"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return f"visual locked target IK failed: {plan_text}"
            self.visual_descend_start = (self.id1, self.id2)
            self.visual_descend_step_index = 0
            self.visual_lost_frames = 0
            self.algorithm_stage = "visual_descend"

        if self.algorithm_stage == "visual_step_wait":
            self.state = "visual descend waiting arm"
            if now < self.stage_deadline:
                return f"visual descend; waiting arm {self.stage_deadline - now:.1f}s"
            self.visual_descend_step_index += 1
            if self.visual_descend_step_index >= self.visual_descend_steps:
                self.algorithm_stage = "final_grab"
            else:
                self.algorithm_stage = "visual_descend"

        if self.algorithm_stage == "visual_descend":
            self.state = "visual descend"
            if self.locked_plan is None or self.visual_descend_start is None:
                self.algorithm_stage = "post_lock_visual_confirm"
                return "visual descend waiting for plan"
            step_index = self.visual_descend_step_index
            step_total = max(1, self.visual_descend_steps)
            ratio = min(1.0, float(step_index + 1) / float(step_total))
            start_id1, start_id2 = self.visual_descend_start
            goal_id1 = self._clamp(self.locked_plan["id1"], self.id1_limits)
            goal_id2 = self._clamp(self.locked_plan["id2"], self.id2_limits)
            target_id1 = self._clamp(
                start_id1 + (goal_id1 - start_id1) * ratio,
                self.id1_limits,
            )
            target_id2 = self._clamp(
                start_id2 + (goal_id2 - start_id2) * ratio,
                self.id2_limits,
            )
            target_id6 = self.id6
            visual_note = "no live visual correction"
            if live_target_fresh:
                accepted_target, reason = self._accept_live_target(live_target)
            else:
                accepted_target, reason = None, "waiting for fresh detection"
            if accepted_target is not None:
                error_x, error_y = self._target_error(accepted_target, frame_shape)
                delta_id6 = self._id6_centering_delta(error_x)
                delta_id2 = self._centering_delta(-error_y, self.id2_pixel_gain)
                target_id2 = self._clamp(target_id2 + delta_id2, self.id2_limits)
                target_id6 = self._clamp(self.id6 + delta_id6, ID6_SAFE_LIMITS)
                visual_note = (
                    f"live dx={error_x:.0f} dy={error_y:.0f} "
                    f"dID2={delta_id2} dID6={delta_id6}"
                )
            elif live_target_fresh:
                if step_index == 0:
                    return self._abort_to_standby(f"visual descend failed before first step: {reason}")
                if self.visual_lost_frames > 4 and step_index < step_total - 1:
                    return self._abort_to_standby(f"visual descend lost target: {reason}")

            target_id1, target_id2 = enforce_angle_gap(
                target_id1,
                target_id2,
                self.id2_limits,
                self.angle_gap_degrees,
            )
            if not self.servo_bridge.write_enabled:
                if can_preview_step:
                    self.id1 = target_id1
                    self.id2 = target_id2
                    self.id6 = target_id6
                    self.visual_descend_step_index += 1
                    self.last_preview_step_time = now
                    if self.visual_descend_step_index >= step_total:
                        self.algorithm_stage = "final_grab"
                self.arm_preview.set_targets(target_id1, target_id2, self.id7, target_id6)
                return (
                    f"preview visual descend step={step_index + 1}/{step_total} "
                    f"ID1={target_id1} ID2={target_id2} ID6={target_id6} | {visual_note}"
                )
            if not can_command:
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return (
                    f"waiting visual descend step={step_index + 1}/{step_total} | "
                    f"{visual_note}"
                )
            self.id1 = target_id1
            self.id2 = target_id2
            self.id6 = target_id6
            result = self._send(
                f"visual descend step={step_index + 1}/{step_total} | {visual_note}",
                require_feedback=True,
            )
            if self.servo_bridge.last_command_ok:
                self.algorithm_stage = "visual_step_wait"
                self.stage_deadline = time.monotonic() + self._arm_settle_s()
            return result

        if self.algorithm_stage == "final_grab":
            self.state = "final grab"
            self.post_center_move_complete = True
            self.algorithm_stage = "close"

        if self.algorithm_stage == "vertical_wait_open":
            self.state = "vertical grasp waiting open claw"
            if now < self.stage_deadline:
                return f"vertical grasp; waiting open claw {self.stage_deadline - now:.1f}s"
            self.algorithm_stage = "vertical_descend"

        if self.algorithm_stage == "vertical_descend":
            self.state = "vertical grasp descend"
            target_id1 = self._clamp(self.vertical_grasp_id1, self.id1_limits)
            target_id2 = self._clamp(self.vertical_grasp_id2, self.id2_limits)
            target_id1, target_id2 = enforce_angle_gap(
                target_id1,
                target_id2,
                self.id2_limits,
                self.angle_gap_degrees,
            )
            if not self.servo_bridge.write_enabled:
                if can_preview_step:
                    self.id1 = target_id1
                    self.id2 = target_id2
                    self.last_preview_step_time = now
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                self.algorithm_stage = "close"
                return f"preview vertical descend ID1={self.id1} ID2={self.id2}"
            if can_command:
                self.id1 = target_id1
                self.id2 = target_id2
                result = self._send(
                    f"vertical descend only ID1={self.id1} ID2={self.id2}",
                    require_feedback=False,
                )
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "vertical_wait_descend"
                    self.stage_deadline = time.monotonic() + self._arm_settle_s()
                return result
            return "vertical grasp; waiting to descend"

        if self.algorithm_stage == "vertical_wait_descend":
            self.state = "vertical grasp waiting descend"
            if now < self.stage_deadline:
                return f"vertical grasp; waiting descend {self.stage_deadline - now:.1f}s"
            self.algorithm_stage = "close"

        if self.algorithm_stage == "motion_wait":
            motion_stage = self.motion_stage_after_wait or "descend"
            self.state = f"locked target waiting {motion_stage}"
            if now < self.stage_deadline:
                return (
                    f"locked target; waiting {motion_stage} "
                    f"{self.stage_deadline - now:.1f}s"
                )
            if motion_stage == "overhead":
                self.algorithm_stage = "descend"
                self.locked_plan = None
                self.approach_attempts = 0
                self.state = "overhead_reached"
            else:
                self.post_center_move_complete = True
                self.algorithm_stage = "close"
                self.state = "descend_reached"
            self.motion_stage_after_wait = None

        if self.algorithm_stage in ("overhead", "descend") and self.locked_plan is None:
            plan, plan_text = self._post_center_plan(
                self.locked_target,
                frame_shape,
                self.algorithm_stage,
            )
            self.locked_plan = plan
            self.arm_preview.publish_plan_marker(plan, plan_text)
            print(
                "GRASP LOCK "
                f"target={json.dumps(self.locked_target, ensure_ascii=True, default=str)} "
                f"plan={plan_text}",
                flush=True,
            )
            if plan["ik_error_mm"] > self.post_center_ik_error_mm:
                self.state = "post-center unreachable"
                self.algorithm_stage = "fault"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return f"claw opened; locked target IK failed: {plan_text}"
        elif self.algorithm_stage in ("overhead", "descend"):
            plan = self.locked_plan
            plan_text = (
                f"locked IK ID1={plan['id1']} ID2={plan['id2']} "
                f"progress={plan.get('progress_ratio', 1.0) * 100:.0f}% "
                f"error={plan['ik_error_mm']:.1f}mm"
            )

        if self.algorithm_stage in ("overhead", "descend"):
            motion_stage = self.algorithm_stage
            self.state = f"locked target {motion_stage}"
            if not self.servo_bridge.write_enabled:
                next_id1 = self._clamp(
                    self.id1 + self._limited_delta(plan["id1"] - self.id1),
                    self.id1_limits,
                )
                next_id2 = self._clamp(
                    self.id2 + self._limited_delta(plan["id2"] - self.id2),
                    self.id2_limits,
                )
                next_id1, next_id2 = enforce_angle_gap(
                    next_id1,
                    next_id2,
                    self.id2_limits,
                    self.angle_gap_degrees,
                )
                if can_preview_step:
                    self.id1, self.id2 = next_id1, next_id2
                    self.last_preview_step_time = now
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                if self.id1 == plan["id1"] and self.id2 == plan["id2"]:
                    if motion_stage == "overhead":
                        self.algorithm_stage = "descend"
                        self.locked_plan = None
                        self.approach_attempts = 0
                    else:
                        self.post_center_move_complete = True
                        self.algorithm_stage = "close"
                return (
                    f"preview algorithm {motion_stage} ID1={self.id1}->{plan['id1']} "
                    f"ID2={self.id2}->{plan['id2']} | {plan_text}"
                )
            target_id1 = self._clamp(plan["id1"], self.id1_limits)
            target_id2 = self._clamp(plan["id2"], self.id2_limits)
            if self.approach_attempts > 0:
                error_id1 = abs(self.id1 - target_id1)
                error_id2 = abs(self.id2 - target_id2)
                if (
                    error_id1 <= self.approach_feedback_tolerance
                    and error_id2 <= self.approach_feedback_tolerance
                ):
                    if motion_stage == "overhead":
                        self.algorithm_stage = "descend"
                        self.locked_plan = None
                        self.approach_attempts = 0
                        self.state = "overhead_reached"
                    else:
                        self.post_center_move_complete = True
                        self.algorithm_stage = "close"
                        self.state = "descend_reached"
                    return (
                        f"{motion_stage} reached ID1={self.id1}/{target_id1} "
                        f"ID2={self.id2}/{target_id2}"
                    )
                if self.approach_attempts >= 3:
                    self.algorithm_stage = "fault"
                    self.state = "fault"
                    self.status = (
                        f"{motion_stage} feedback outside +/-{self.approach_feedback_tolerance}: "
                        f"ID1={self.id1}/{target_id1} ID2={self.id2}/{target_id2}"
                    )
                    return self.status
            if can_command:
                self.id1 = target_id1
                self.id2 = target_id2
                self.approach_attempts += 1
                self.state = "approach_feedback"
                result = self._send(
                    f"locked target algorithm {motion_stage} attempt={self.approach_attempts}/3 | {plan_text}",
                    require_feedback=True,
                )
                if (
                    self.servo_bridge.last_command_ok
                    and getattr(self.servo_bridge, "assumed_feedback", False)
                ):
                    self.motion_stage_after_wait = motion_stage
                    self.algorithm_stage = "motion_wait"
                    self.stage_deadline = time.monotonic() + self._arm_settle_s()
                return result
            return f"locked target; waiting algorithm {motion_stage}"

        if self.algorithm_stage == "close":
            self.state = "locked target close claw"
            if not self.servo_bridge.write_enabled:
                self.id7 = self.id7_closed
                if (
                    self.active_chassis_station == "PLATFORM_PICK"
                    and not self.abort_after_return
                ):
                    self.algorithm_stage = "platform_post_grab_id6"
                elif self.post_center_direct_descend and not self.abort_after_return:
                    self.algorithm_stage = "post_grab_id2_retreat"
                else:
                    self.algorithm_stage = "return"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview claw closed"
            if can_command:
                self.state = "claw_close_ack"
                result = self._send_gripper_id7(self.id7_closed, "locked target; close claw")
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "close_wait"
                    self.stage_deadline = time.monotonic() + self._aux_settle_s()
                return result
            return "locked target; waiting to close claw"

        if self.algorithm_stage == "close_wait":
            self.state = "locked target waiting close claw"
            if now < self.stage_deadline:
                return f"locked target; waiting close claw {self.stage_deadline - now:.1f}s"
            if self.post_center_direct_descend and not self.abort_after_return:
                self.algorithm_stage = "post_grab_id2_retreat"
            else:
                self.algorithm_stage = "return"
            self.return_attempts = 0

        if self.algorithm_stage == "post_grab_id2_retreat":
            self.state = "post-grab retreat"
            target_id1 = self._clamp(READY_ID1_TICK, self.id1_limits)
            target_id2 = self._clamp(POST_GRAB_ID2_RETREAT_TICK, self.id2_limits)
            target_id6 = BASE_YAW_CENTER_TICK
            target_id1, target_id2 = enforce_angle_gap(
                target_id1,
                target_id2,
                self.id2_limits,
                self.angle_gap_degrees,
            )
            if not self.servo_bridge.write_enabled:
                self.id1 = target_id1
                self.id2 = target_id2
                self.id6 = target_id6
                self.id5 = CATCHER_RELEASE_READY_TICK
                self.algorithm_stage = "release"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return (
                    f"preview post-grab retreat ID1={self.id1} "
                    f"ID2={self.id2} ID5={self.id5} ID6={self.id6}"
                )
            if can_command:
                self.id1 = target_id1
                self.id2 = target_id2
                self.id6 = target_id6
                result = self._send_retreat_with_catcher_open(
                    f"post-grab retreat/open catcher ID1={self.id1} "
                    f"ID2={self.id2} ID5={CATCHER_RELEASE_READY_TICK} ID6={self.id6}",
                    require_feedback=True,
                )
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "post_grab_id2_retreat_wait"
                    self.stage_deadline = time.monotonic() + self._arm_settle_s()
                return result
            return "post-grab retreat; waiting to move ID2"

        if self.algorithm_stage == "post_grab_id2_retreat_wait":
            self.state = "post-grab ID2 retreat wait"
            if now < self.stage_deadline:
                return f"post-grab retreat; waiting arm {self.stage_deadline - now:.1f}s"
            self.algorithm_stage = "release"

        if self.algorithm_stage == "return":
            self.state = "returning to standby"
            target_id1 = READY_ID1_TICK
            target_id2 = READY_ID2_TICK
            target_id6 = BASE_YAW_CENTER_TICK
            if not self.servo_bridge.write_enabled:
                self.id1 = target_id1
                self.id2 = target_id2
                self.id6 = target_id6
                self.id7 = self.id7_closed
                if self.abort_after_return:
                    return self._reset_cycle_for_search("preview abort returned to standby")
                self.algorithm_stage = "complete"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview grasp complete; returned to standby"
            if self.return_attempts > 0:
                error_id1 = abs(self.id1 - target_id1)
                error_id2 = abs(self.id2 - target_id2)
                if (
                    error_id1 <= self.approach_feedback_tolerance
                    and error_id2 <= self.approach_feedback_tolerance
                ):
                    if self.abort_after_return:
                        return self._reset_cycle_for_search("abort returned to standby")
                    self.algorithm_stage = "catcher"
                    self.state = "standby prepare catcher"
                    self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                    return (
                        f"standby reached; preparing catcher ID1={self.id1} "
                        f"ID2={self.id2} ID6={self.id6} ID7={self.id7}"
                    )
                if self.return_attempts >= 3:
                    self.algorithm_stage = "fault"
                    self.state = "fault"
                    self.status = (
                        f"standby feedback outside +/-{self.approach_feedback_tolerance}: "
                        f"ID1={self.id1}/{target_id1} ID2={self.id2}/{target_id2}"
                    )
                    return self.status
            if can_command:
                self.return_attempts += 1
                result = self._send_fixed_arm_pose_staged(
                    target_id1,
                    target_id2,
                    target_id6,
                    f"return standby attempt={self.return_attempts}/3",
                    raising=True,
                    id7=self.id7_closed,
                    id5=self.id5,
                    splitter_id4=self.splitter_id4,
                )
                if (
                    self.servo_bridge.last_command_ok
                    and getattr(self.servo_bridge, "assumed_feedback", False)
                ):
                    self.algorithm_stage = "return_wait"
                    self.stage_deadline = time.monotonic() + self._arm_settle_s()
                return result
            return "grasp complete; waiting to return standby"

        if self.algorithm_stage == "return_wait":
            self.state = "returning to standby"
            if now < self.stage_deadline:
                return f"return standby; waiting arm {self.stage_deadline - now:.1f}s"
            if self.abort_after_return:
                return self._reset_cycle_for_search("abort returned to standby")
            self.algorithm_stage = "catcher"
            self.state = "standby prepare catcher"
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            return (
                f"standby reached; preparing catcher ID1={self.id1} "
                f"ID2={self.id2} ID6={self.id6} ID7={self.id7}"
            )

        if self.algorithm_stage == "catcher":
            self.state = "extend catcher before release"
            if not self.servo_bridge.write_enabled:
                self.id5 = CATCHER_RELEASE_READY_TICK
                self.algorithm_stage = "release"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview catcher ready; releasing next"
            if can_command:
                result = self._send_id5(
                    CATCHER_RELEASE_READY_TICK,
                    "extend catcher before release",
                    critical=True,
                )
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "release"
                return result
            return "standby; waiting to extend catcher"

        if self.algorithm_stage == "release":
            release_reason = (
                "release target after ID2 retreat"
                if self.post_center_direct_descend and not self.abort_after_return
                else "release target at standby"
            )
            self.state = release_reason
            if not self.servo_bridge.write_enabled:
                self.id7 = self.id7_open
                self.algorithm_stage = "close_catcher_after_release"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview release target; closing catcher next"
            if can_command:
                result = self._send_gripper_id7(self.id7_open, release_reason)
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "release_wait"
                    self.stage_deadline = time.monotonic() + self._aux_settle_s() + 0.30
                return result
            return "standby; waiting to release target"

        if self.algorithm_stage == "release_wait":
            self.state = "release target at standby"
            if now < self.stage_deadline:
                return f"standby; waiting release {self.stage_deadline - now:.1f}s"
            self.algorithm_stage = "close_catcher_after_release"

        if self.algorithm_stage == "close_catcher_after_release":
            self.state = "close catcher after release"
            if not self.servo_bridge.write_enabled:
                self.id5 = CATCHER_HOME_TICK
                self.algorithm_stage = "reclose"
                return "preview catcher closed after release"
            if self.id5 == CATCHER_HOME_TICK:
                self.algorithm_stage = "reclose"
            elif can_command:
                result = self._send_id5(
                    CATCHER_HOME_TICK,
                    "close catcher 0.3s after ID7 release",
                    critical=False,
                )
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "reclose"
                return result
            else:
                return "release complete; waiting to close catcher"

        if self.algorithm_stage == "reclose":
            self.state = "close claw for next search"
            if not self.servo_bridge.write_enabled:
                self.id7 = self.id7_closed
                self.id5 = CATCHER_HOME_TICK
                return self._reset_cycle_for_search(
                    "preview ready for next target",
                    start_retrigger_cooldown=True,
                )
            if can_command:
                result = self._send_gripper_id7(self.id7_closed, "close claw for next search")
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "reclose_wait"
                    self.stage_deadline = time.monotonic() + self._aux_settle_s()
                return result
            return "release complete; waiting to close claw for next search"

        if self.algorithm_stage == "reclose_wait":
            self.state = "close claw for next search"
            if now < self.stage_deadline:
                return f"waiting close claw for next search {self.stage_deadline - now:.1f}s"
            if self.post_center_direct_descend and not self.abort_after_return:
                self.algorithm_stage = "final_return"
            else:
                if self.id5 != CATCHER_HOME_TICK:
                    home_result = self._send_id5(
                        CATCHER_HOME_TICK,
                        "catcher home for next search",
                        critical=False,
                    )
                    if not self.servo_bridge.last_command_ok:
                        return home_result
                return self._reset_cycle_for_search(
                    f"ready for next target ID1={self.id1} ID2={self.id2} "
                    f"ID5={self.id5} ID6={self.id6} ID7={self.id7}",
                    start_retrigger_cooldown=True,
                )

        if self.algorithm_stage == "final_return":
            self.state = "final return to standby"
            target_id1 = READY_ID1_TICK
            target_id2 = READY_ID2_TICK
            target_id6 = BASE_YAW_CENTER_TICK
            if not self.servo_bridge.write_enabled:
                self.id1 = target_id1
                self.id2 = target_id2
                self.id6 = target_id6
                return self._reset_cycle_for_search(
                    "preview final return to standby",
                    start_retrigger_cooldown=True,
                )
            if can_command:
                result = self._send_fixed_arm_pose_staged(
                    target_id1,
                    target_id2,
                    target_id6,
                    "final return standby after release/reclose",
                    raising=True,
                    id7=self.id7,
                    id5=self.id5,
                    splitter_id4=self.splitter_id4,
                )
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "final_return_wait"
                    self.stage_deadline = time.monotonic() + self._arm_settle_s()
                return result
            return "final return; waiting to move standby"

        if self.algorithm_stage == "final_return_wait":
            self.state = "final return waiting arm"
            if now < self.stage_deadline:
                return f"final return; waiting arm {self.stage_deadline - now:.1f}s"
            if self.id5 != CATCHER_HOME_TICK:
                home_result = self._send_id5(
                    CATCHER_HOME_TICK,
                    "catcher home for next search",
                    critical=False,
                )
                if not self.servo_bridge.last_command_ok:
                    return home_result
            return self._reset_cycle_for_search(
                f"ready for next target ID1={self.id1} ID2={self.id2} "
                f"ID5={self.id5} ID6={self.id6} ID7={self.id7}",
                start_retrigger_cooldown=True,
            )

        if self.algorithm_stage == "fault":
            return self.status

        self.state = "algorithm grasp complete"
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        return (
            f"algorithm grasp complete ID1={self.id1} "
            f"ID2={self.id2} ID6={self.id6} ID7={self.id7}; camera ignored"
        )

    def _update_one_shot(self, target, frame_shape):
        if self.one_shot_complete:
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            return "one-shot grasp complete; servos stopped"

        if self.one_shot_target is None:
            ready, reason = self._target_ready(target)
            if not ready:
                self.arm_preview.set_targets(READY_ID1_TICK, READY_ID2_TICK, self.id7, self.id6)
                return reason
            self.one_shot_target = self._copy_target(target)
            self.centered_frames = self.stable_frames_required

        target = self.one_shot_target
        plan, plan_text = self._one_shot_plan(target, frame_shape)

        if not self.servo_bridge.write_enabled:
            if plan is None:
                self.arm_preview.set_targets(READY_ID1_TICK, READY_ID2_TICK, self.id7, self.id6)
                return plan_text
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            self.arm_preview.publish_plan_marker(plan, plan_text)
            return f"preview one-shot {plan_text}"

        now = time.monotonic()
        can_command = now - self.last_command_time >= self.command_interval_s
        if not can_command:
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            return "one-shot waiting command interval"

        if self.id7 != self.id7_open:
            self.state = "one-shot open claw"
            return self._send_gripper_id7(self.id7_open, "one-shot open claw")

        if not self.one_shot_approach_sent:
            if plan is None:
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return plan_text
            if plan["ik_error_mm"] > self.max_one_shot_ik_error_mm:
                self.state = "unreachable"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                self.arm_preview.publish_plan_marker(plan, plan_text)
                return (
                    f"IK simulation failed: {plan_text}; "
                    "real servos not commanded"
                )
            self.id1 = self._clamp(plan["id1"], self.id1_limits)
            self.id2 = self._clamp(plan["id2"], self.id2_limits)
            self.one_shot_approach_sent = True
            self.state = "one-shot approach"
            self.arm_preview.publish_plan_marker(plan, plan_text)
            return self._send(
                f"one-shot simulated approach {plan_text}",
                require_feedback=True,
            )

        if self.id7 != self.id7_closed:
            self.state = "one-shot close claw"
            return self._send_gripper_id7(self.id7_closed, "one-shot close claw")

        self.one_shot_complete = True
        self.state = "one-shot complete"
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish("one-shot grasp complete; servos stopped")
        if plan is None:
            return "one-shot grasp complete; servos stopped"
        return f"one-shot grasp complete {plan_text}; servos stopped"

    def update(self, target, frame_shape, detection_fresh=True):
        if self.platform_task.stage is not None:
            # White-line entry and inter-slot gaps retain the expanded high pose.
            return self._sync_platform_task()
        if not self.enabled:
            return self.status
        now = time.monotonic()
        can_preview_step = (
            now - self.last_preview_step_time >= self.preview_step_interval_s
        )
        if self.read_only_sync:
            feedback_ready = False
            feedback = None
            with self.read_only_feedback_lock:
                if self.read_only_feedback_ready:
                    feedback_ready = True
                    feedback = self.read_only_feedback
                    self.read_only_feedback_ready = False
            if feedback_ready:
                if feedback is not None and self._apply_feedback(feedback):
                    self.state = "read_only_sync"
                    self.status = (
                        f"read-only synchronized ID1={self.id1} ID2={self.id2} "
                        f"ID7={self.id7} ID6={self.arm_preview.id6}"
                    )
                elif feedback is None:
                    self.status = f"read-only sync waiting | {self.servo_bridge.status}"
            if now - self.last_feedback_attempt >= 1.0:
                self.last_feedback_attempt = now
                self._start_read_only_feedback()
            self.arm_preview.publish(self.status, state="READ_ONLY_SYNC")
            return self.status
        if self.servo_bridge.write_enabled and self.startup_stage != "complete":
            return self._update_startup(now)
        if self.servo_bridge.write_enabled and not self.synchronized:
            if now - self.last_feedback_attempt < 0.8:
                return self.status
            self.last_feedback_attempt = now
            feedback = self.servo_bridge.query_positions()
            if feedback is None:
                self.status = "waiting for Hiwonder HTD85 position synchronization"
                return f"{self.status} | {self.servo_bridge.status}"
            if not self._apply_feedback(feedback):
                return self.status
            self.state = "searching"
            self.status = f"synchronized ID1={self.id1} ID2={self.id2}"
            return self.status

        if self.feedback_pending:
            if now < self.feedback_due:
                return f"moving; feedback in {self.feedback_due - now:.1f}s"
            feedback = self.servo_bridge.query_positions()
            if feedback is None:
                self.feedback_failures += 1
                if self.feedback_failures >= 3:
                    self.state = "fault"
                    self.feedback_pending = False
                    self.status = "servo feedback timeout; automatic motion stopped"
                    return self.status
                self.feedback_due = now + 0.5
                return f"servo feedback retry {self.feedback_failures}/3"
            if not self._apply_feedback(feedback):
                return self.status

        if self.state == "fault":
            return self.status
        if self.one_shot:
            return self._update_one_shot(target, frame_shape)
        if self.platform_high_hold and self.active_chassis_station in {
            None, "COLUMN_CATCH",
        }:
            # BLUE COLUMN_CATCH stays active until H7 requests RETRACT after
            # white-line alignment. Never fall through to generic search,
            # which can slowly change ID1/ID2 away from the high pose.
            return self._hold_platform_high_pose()
        if (
            self.active_chassis_station == "PLATFORM_PICK"
            and self.chassis_station_stage in {
                "platform_preselect",
                "platform_entry_hold",
            }
        ):
            if self.chassis_station_stage == "platform_entry_hold":
                self.id1, self.id2, self.id6 = PLATFORM_HIGH_POSE
                self.id7 = self.id7_closed
                self.id5 = CATCHER_HOME_TICK
                self.splitter_id4 = SPLITTER_RETRACT_TICK
                self._enforce_angle_gap()
                self.arm_preview.set_targets(
                    self.id1, self.id2, self.id7, self.id6
                )
            return self.status
        live_target = target
        if self.locked_target is not None:
            return self._update_locked_grasp(
                frame_shape,
                now,
                can_preview_step,
                live_target=live_target,
                live_target_fresh=detection_fresh,
            )
        if not detection_fresh:
            # Servo wait stages above still advance on every loop.  Target
            # acquisition and centering, however, may only consume a new
            # detector result; reusing a cached result causes false stability.
            return self.status
        cooldown_remaining = self.ignore_new_targets_until - now
        ignoring_new_target = target is not None and cooldown_remaining > 0.0
        if ignoring_new_target:
            target = None
            self.centered_frames = 0
            self.last_visual_target = None
        if target is None:
            self.centered_frames = 0
            self.centering_correction_count = 0
            if not ignoring_new_target:
                self.visual_lost_frames += 1
                if self.visual_lost_frames > 6:
                    self.last_visual_target = None
            self.state = "post-grasp cooldown" if ignoring_new_target else "searching"
            can_command = now - self.last_command_time >= self.command_interval_s
            if (
                self.active_chassis_station == "PLATFORM_PICK"
                and self.chassis_station_stage is None
            ):
                expand_id1, expand_id2, expand_id6 = PLATFORM_HIGH_POSE
                self.id7 = self.id7_closed
                self.id5 = CATCHER_HOME_TICK
                self.splitter_id4 = SPLITTER_RETRACT_TICK
                self._enforce_angle_gap()
                self.arm_preview.set_targets(
                    expand_id1, expand_id2, self.id7, expand_id6
                )
                if not self.servo_bridge.write_enabled:
                    return (
                        "preview PLATFORM_PICK high hold "
                        f"ID1={expand_id1} ID2={expand_id2} ID6={expand_id6}"
                    )
                if can_command and (
                    (self.id1, self.id2, self.id6)
                    != (expand_id1, expand_id2, expand_id6)
                ):
                    self.id1, self.id2, self.id6 = (
                        expand_id1,
                        expand_id2,
                        expand_id6,
                    )
                    return self._send(
                        "PLATFORM_PICK keep expanded high pose",
                        require_feedback=False,
                    )
                return "PLATFORM_PICK waiting selected target; high pose held"
            expand_id1 = self._clamp(
                self.id1 + self._limited_delta(READY_ID1_TICK - self.id1),
                self.id1_limits,
            )
            expand_id2 = self._clamp(
                self.id2 + self._limited_delta(READY_ID2_TICK - self.id2),
                self.id2_limits,
            )
            expand_id6 = self._clamp(
                self.id6 + self._limited_delta(BASE_YAW_CENTER_TICK - self.id6),
                ID6_SAFE_LIMITS,
            )
            expand_id1, expand_id2 = enforce_angle_gap(
                expand_id1,
                expand_id2,
                self.id2_limits,
                self.angle_gap_degrees,
            )
            if not self.servo_bridge.write_enabled:
                if can_preview_step:
                    self.id1, self.id2, self.id6 = expand_id1, expand_id2, expand_id6
                    self.last_preview_step_time = now
                self.arm_preview.set_targets(expand_id1, expand_id2, self.id7, expand_id6)
                return (
                    f"preview {self.state}; slow expand "
                    f"ID1={expand_id1} ID2={expand_id2} ID6={expand_id6}"
                )
            if can_command and (
                expand_id1 != self.id1
                or expand_id2 != self.id2
                or expand_id6 != self.id6
            ):
                self.id1, self.id2, self.id6 = expand_id1, expand_id2, expand_id6
                reason = (
                    "post-grasp cooldown; slow expand"
                    if ignoring_new_target
                    else "searching red target; slow expand"
                )
                return self._send(reason, require_feedback=False)
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            if ignoring_new_target:
                return (
                    f"post-grasp cooldown {max(0.0, cooldown_remaining):.1f}s; "
                    "new target ignored"
                )
            return "grasp searching red target"
        edge_note = ""
        hold_reason = self._centering_target_hold_reason(target, frame_shape)
        if hold_reason is not None:
            self.centered_frames = 0
            self.state = "target unstable"
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            return hold_reason
        if target.get("area_percent", 0.0) < self.min_target_area_percent:
            self.centered_frames = 0
            self.state = "target too small"
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            return (
                f"letter target area below {self.min_target_area_percent:.2f}%; "
                "grasp disabled"
            )
        distance_cm = target.get("distance_cm")
        if (
            distance_cm is None
            or not self.min_target_distance_cm
            <= distance_cm
            <= self.max_target_distance_cm
        ):
            self.centered_frames = 0
            self.state = "target outside calibrated range"
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            distance_text = "unknown" if distance_cm is None else f"{distance_cm:.1f}cm"
            return (
                f"target distance {distance_text} outside calibrated "
                f"{self.min_target_distance_cm:.0f}-"
                f"{self.max_target_distance_cm:.0f}cm range"
            )

        height, width = frame_shape[:2]
        cx, cy = target["center"]
        error_x = cx - width / 2.0
        error_y = cy - height / 2.0
        target_distance_mm = None if distance_cm is None else distance_cm * 10.0
        gripper_distance_mm = None
        if target_distance_mm is not None:
            gripper_distance_mm = target_distance_mm - self.camera_gripper_offset_mm

        can_command = now - self.last_command_time >= self.command_interval_s
        if self.locked_target is None:
            centering_profile = self._centering_profile()
            delta_id6 = self._id6_centering_delta(
                error_x,
                centering_profile["id6_gain"],
                centering_profile["id6_max_step_ticks"],
            )
            delta_id2 = self._centering_delta(
                -error_y,
                centering_profile["id2_gain"],
                centering_profile["id2_max_step_ticks"],
            )
            target_id2 = self._clamp_center_id2(self.id2 + delta_id2)
            id2_center_limited = self._id2_center_limit_blocks(delta_id2, target_id2)
            if id2_center_limited:
                delta_id2 = 0
                if delta_id6 == 0:
                    self.state = "centered at ID2 limit"
            delta_id2, delta_id6, axis_note = self._vector_centering_deltas(
                delta_id2,
                delta_id6,
                id2_center_limited,
                error_x,
                error_y,
                centering_profile["id2_max_step_ticks"],
                centering_profile["id6_max_step_ticks"],
            )
            target_id2 = self._clamp_center_id2(self.id2 + delta_id2)
            if delta_id2 != 0 or delta_id6 != 0:
                self.centered_frames = 0
                self.center_distance_samples = []
                self.horizontal_correction_done = False
                target_id6 = self._clamp(self.id6 + delta_id6, ID6_SAFE_LIMITS)
                target_id1 = self.id1
                target_id1, target_id2 = enforce_angle_gap(
                    target_id1,
                    target_id2,
                    self.id2_limits,
                    self.angle_gap_degrees,
                )
                self.state = "centering target"
                if not self.servo_bridge.write_enabled:
                    if can_preview_step:
                        self.id2 = target_id2
                        self.id6 = target_id6
                        self.last_preview_step_time = now
                    self.arm_preview.set_targets(target_id1, target_id2, self.id7, target_id6)
                    return (
                        f"preview centering dx={error_x:.0f} dy={error_y:.0f} "
                        f"ID2={target_id2} ID6={target_id6}"
                    )
                can_command = now - self.last_command_time >= centering_profile["command_interval_s"]
                if can_command:
                    previous_id2 = self.id2
                    previous_id6 = self.id6
                    self.id2 = target_id2
                    self.id6 = target_id6
                    send_id2 = self.id2 != previous_id2
                    send_id6 = self.id6 != previous_id6
                    if not send_id2 and not send_id6:
                        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                        return (
                            f"centering blocked by servo limits dx={error_x:.0f} dy={error_y:.0f} "
                            f"ID2={self.id2} ID6={self.id6}"
                        )
                    return self._send_center_correction(
                        f"centering target{edge_note} dx={error_x:.0f} dy={error_y:.0f} "
                        f"axis={axis_note} dID2={self.id2 - previous_id2} "
                        f"dID6={self.id6 - previous_id6} {centering_profile['note']}",
                        send_id2=send_id2,
                        send_id6=send_id6,
                    )
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return f"waiting centering dx={error_x:.0f} dy={error_y:.0f}"

            self.centered_frames += 1
            median_distance_cm = self._record_center_distance_sample(distance_cm)
            self.horizontal_correction_done = True
            if self.centered_frames < self.stable_frames_required:
                self.state = "confirming center"
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                distance_note = (
                    "unknown"
                    if median_distance_cm is None
                    else f"{median_distance_cm:.1f}cm"
                )
                return (
                    f"center confirm {self.centered_frames}/{self.stable_frames_required} "
                    f"dx={error_x:.0f} dy={error_y:.0f} d_med={distance_note}"
                )
            locked_target = self._copy_target(target)
            if median_distance_cm is not None:
                locked_target["distance_cm_raw"] = distance_cm
                locked_target["distance_cm"] = median_distance_cm
                locked_target["distance_cm_samples"] = tuple(
                    round(sample, 2) for sample in self.center_distance_samples
                )
            self.locked_target = locked_target
            self.last_visual_target = self._copy_target(target)
            self.visual_confirm_frames = 0
            self.visual_lost_frames = 0
            self.visual_descend_start = None
            self.visual_descend_step_index = 0
            self.locked_plan = None
            self.approach_attempts = 0
            self.return_attempts = 0
            self.abort_after_return = False
            self.algorithm_stage = "open"
            self.state = "target locked"

        return self._update_locked_grasp(
            frame_shape,
            now,
            can_preview_step,
            live_target=live_target,
        )


def arm_joint_positions(id1_tick, id2_tick, id4_tick, id6_tick=BASE_YAW_CENTER_TICK):
    return joint_positions(id1_tick, id2_tick, id4_tick, id6_tick)


class ArmPreviewPublisher:
    def __init__(self, enabled, id1, id2, id4, id6=BASE_YAW_CENTER_TICK):
        self.enabled = enabled
        self.id1 = id1
        self.id2 = id2
        self.id7 = id4
        self.id6 = id6
        self.rclpy = None
        self.String = None
        self.JointState = None
        self.Marker = None
        self.node = None
        self.joint_pub = None
        self.status_pub = None
        self.target_pub = None
        self.marker_pub = None
        self.last_joint_publish_time = 0.0
        self.last_joint_ticks = None
        self.joint_publish_interval_s = 0.12

        if self.enabled:
            self._open()

    def set_targets(self, id1, id2, id4, id6=None):
        self.id1 = int(id1)
        self.id2 = int(id2)
        self.id7 = int(id4)
        if id6 is not None:
            self.id6 = int(id6)

    def _open(self):
        try:
            import rclpy
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
            from visualization_msgs.msg import Marker
        except ImportError as exc:
            self.enabled = False
            print(f"arm preview disabled: ROS import failed: {exc}")
            return

        self.rclpy = rclpy
        self.JointState = JointState
        self.String = String
        self.Marker = Marker
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("target_vision_arm_preview")
        self.joint_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.node.create_publisher(String, "/arm/grasp_status", 10)
        self.target_pub = self.node.create_publisher(String, "/arm/servo_targets", 10)
        self.marker_pub = self.node.create_publisher(Marker, "/arm/vision_target_marker", 10)
        self.default_marker_pub = self.node.create_publisher(Marker, "/visualization_marker", 10)
        print("arm preview publishing /joint_states, /arm/grasp_status, and target markers")

    def _publish_marker(self, marker):
        self.marker_pub.publish(marker)
        self.default_marker_pub.publish(marker)

    def publish_plan_marker(self, plan, detail="vision target model"):
        if not self.enabled or plan is None or self.marker_pub is None:
            return

        try:
            now = self.node.get_clock().now().to_msg()
            target_x_m = (plan["current_x_mm"] + plan["forward_mm"] + plan.get("target_distance_mm", 0.0)) / 1000.0
            target_y_m = plan["lateral_mm"] / 1000.0
            target_z_m = (plan["current_z_mm"] + plan["vertical_target_mm"]) / 1000.0
            gripper_x_m = (plan["current_x_mm"] + plan["forward_mm"]) / 1000.0
            gripper_z_m = (plan["current_z_mm"] + plan["vertical_target_mm"]) / 1000.0

            target = self.Marker()
            target.header.frame_id = "base_link"
            target.header.stamp = now
            target.ns = "vision_target"
            target.id = 1
            # Debug visualization is intentionally larger than the real object
            # so it remains visible on the small RK screen.
            real_target_size_m = plan.get("target_size_mm", LETTER_CUBE_SIDE_MM) / 1000.0
            target_size_m = max(0.08, real_target_size_m * 1.8)
            target.type = (
                self.Marker.SPHERE
                if plan.get("target_kind") == "ball"
                else self.Marker.CUBE
            )
            target.action = self.Marker.ADD
            target.pose.position.x = target_x_m
            target.pose.position.y = target_y_m
            target.pose.position.z = target_z_m
            target.pose.orientation.w = 1.0
            target.scale.x = target_size_m
            target.scale.y = target_size_m
            target.scale.z = target_size_m
            target.color.r = 1.0
            target.color.g = 0.05
            target.color.b = 0.02
            target.color.a = 0.9
            self._publish_marker(target)

            goal = self.Marker()
            goal.header.frame_id = "base_link"
            goal.header.stamp = now
            goal.ns = "vision_target"
            goal.id = 2
            goal.type = self.Marker.SPHERE
            goal.action = self.Marker.ADD
            goal.pose.position.x = gripper_x_m
            goal.pose.position.y = 0.0
            goal.pose.position.z = gripper_z_m
            goal.pose.orientation.w = 1.0
            goal.scale.x = 0.055
            goal.scale.y = 0.055
            goal.scale.z = 0.055
            goal.color.r = 0.0
            goal.color.g = 1.0
            goal.color.b = 0.15
            goal.color.a = 0.95
            self._publish_marker(goal)

            line = self.Marker()
            line.header.frame_id = "base_link"
            line.header.stamp = now
            line.ns = "vision_target"
            line.id = 3
            line.type = self.Marker.LINE_STRIP
            line.action = self.Marker.ADD
            line.scale.x = 0.012
            line.color.r = 0.1
            line.color.g = 0.65
            line.color.b = 1.0
            line.color.a = 0.9
            point_type = type(line.points[0]) if line.points else None
            if point_type is None:
                from geometry_msgs.msg import Point
                point_type = Point
            p1 = point_type()
            p1.x = gripper_x_m
            p1.y = 0.0
            p1.z = gripper_z_m
            p2 = point_type()
            p2.x = target_x_m
            p2.y = target_y_m
            p2.z = target_z_m
            line.points = [p1, p2]
            self._publish_marker(line)

            label = self.Marker()
            label.header.frame_id = "base_link"
            label.header.stamp = now
            label.ns = "vision_target"
            label.id = 4
            label.type = self.Marker.TEXT_VIEW_FACING
            label.action = self.Marker.ADD
            label.pose.position.x = target_x_m
            label.pose.position.y = target_y_m
            label.pose.position.z = target_z_m + 0.09
            label.pose.orientation.w = 1.0
            label.scale.z = 0.045
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 0.15
            label.color.a = 1.0
            label.text = (
                f"d={plan['distance_mm']:.0f}mm "
                f"lat={plan['lateral_mm']:.0f} "
                f"dz={plan['vertical_target_mm']:.0f}\n"
                f"ID1={plan['id1']} ID2={plan['id2']} "
                f"err={plan['ik_error_mm']:.1f}mm"
            )
            self._publish_marker(label)
        except Exception as exc:
            print(f"target marker publish failed: {exc}")

    def publish(self, detail, target=None, state=None):
        if not self.enabled:
            return

        try:
            now = time.monotonic()
            joint_ticks = (self.id1, self.id2, self.id7, self.id6)
            should_publish_joint = joint_ticks != self.last_joint_ticks
            if should_publish_joint:
                msg = self.JointState()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = "base_mount"
                msg.name = JOINT_NAMES
                msg.position = arm_joint_positions(self.id1, self.id2, self.id7, self.id6)
                self.joint_pub.publish(msg)
                self.last_joint_ticks = joint_ticks
                self.last_joint_publish_time = now

            payload = {
                "state": state or "VISION_PREVIEW",
                "detail": detail,
                "id1": self.id1,
                "id2": self.id2,
                "id4": self.id7,
                "id6": self.id6,
            }
            if target is not None:
                payload.update(
                    {
                        "color": target.get("color"),
                        "kind": target.get("kind"),
                        "center": target.get("center"),
                        "bbox": target.get("bbox"),
                        "area_percent": round(target.get("area_percent", 0.0), 2),
                        "distance_cm": None
                        if target.get("distance_cm") is None
                        else round(target.get("distance_cm"), 1),
                    }
                )
            status = self.String(data=json.dumps(payload, ensure_ascii=False))
            self.status_pub.publish(status)
            self.target_pub.publish(status)
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            self.enabled = False
            if "context is invalid" not in str(exc):
                print(f"arm preview stopped: {exc}")

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.rclpy is not None and self.rclpy.ok():
            self.rclpy.shutdown()


class TargetDetector:
    def __init__(self):
        # Loading the ABCD model is unnecessary during task-one ball search.
        # Create it only when a letter-capable mode is actually selected.
        self.letter_detector = None

    def _detect_letters(self, frame):
        if self.letter_detector is None:
            self.letter_detector = ABCDDetector()
        return self.letter_detector.detect(frame)

    def _detect_task3_rotated_letters(self, frame, roi=None):
        if self.letter_detector is None:
            self.letter_detector = ABCDDetector()
        if roi is None or len(roi) < 4:
            return self.letter_detector.detect_task3_rotated(frame)
        height, width = frame.shape[:2]
        x, y, roi_width, roi_height = (int(value) for value in roi[:4])
        x0, y0 = max(0, x), max(0, y)
        x1 = min(width, x + roi_width)
        y1 = min(height, y + roi_height)
        if x1 <= x0 or y1 <= y0:
            return []
        detections = self.letter_detector.detect_task3_rotated(
            frame[y0:y1, x0:x1], frame_shape=frame.shape
        )
        for detection in detections:
            cx, cy = detection["center"]
            detection["center"] = (cx + x0, cy + y0)
            bx, by, bw, bh = detection["bbox"]
            detection["bbox"] = (bx + x0, by + y0, bw, bh)
            if "box" in detection:
                detection["box"] = (
                    np.asarray(detection["box"], dtype=np.int32)
                    + np.asarray((x0, y0), dtype=np.int32)
                )
        return detections

    @staticmethod
    def _column_block_angle(box):
        points = np.asarray(box, dtype=np.float32).reshape(4, 2)
        edge = points[1] - points[0]
        angle = math.degrees(math.atan2(float(edge[1]), float(edge[0])))
        return ((angle + 45.0) % 90.0) - 45.0

    @staticmethod
    def _ordered_quad(points):
        points = np.asarray(points, dtype=np.float32).reshape(4, 2)
        center = points.mean(axis=0)
        angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
        ordered = points[np.argsort(angles)]
        start = int(np.argmin(ordered.sum(axis=1)))
        return np.roll(ordered, -start, axis=0)

    def _detect_column_blocks(self, frame):
        """Return geometric white-square candidates without classifying glyphs."""
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(
            hsv,
            np.array(
                (
                    COLUMN_BLOCK_GREEN_HUE_RANGE[0],
                    COLUMN_BLOCK_GREEN_MIN_SATURATION,
                    COLUMN_BLOCK_GREEN_MIN_VALUE,
                ),
                dtype=np.uint8,
            ),
            np.array((COLUMN_BLOCK_GREEN_HUE_RANGE[1], 255, 255), dtype=np.uint8),
        )
        white = cv2.inRange(
            hsv,
            np.array((0, 0, 190), dtype=np.uint8),
            np.array((180, 135, 255), dtype=np.uint8),
        )
        white = cv2.morphologyEx(
            white,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
        )
        contours, _ = cv2.findContours(
            white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        frame_area = float(max(1, width * height))
        blocks = []
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = float(cv2.contourArea(contour))
            if not COLUMN_BLOCK_MIN_AREA <= area <= frame_area * COLUMN_BLOCK_MAX_AREA_RATIO:
                continue
            rect = cv2.minAreaRect(contour)
            rect_w, rect_h = rect[1]
            x, y, box_w, box_h = cv2.boundingRect(contour)
            if min(box_w, box_h) < COLUMN_BLOCK_MIN_SIDE:
                continue
            if box_w / float(max(1, box_h)) < 0.55 or box_w / float(max(1, box_h)) > 1.80:
                continue
            if max(box_w, box_h) > min(width, height) * COLUMN_BLOCK_MAX_SIDE_RATIO:
                continue
            rectangularity = area / max(1.0, rect_w * rect_h)
            if rectangularity < COLUMN_BLOCK_MIN_RECTANGULARITY:
                continue
            hull_area = max(
                1.0, float(cv2.contourArea(cv2.convexHull(contour)))
            )
            if area / hull_area < COLUMN_BLOCK_MIN_SOLIDITY:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            approx_vertices = len(
                cv2.approxPolyDP(
                    contour,
                    COLUMN_BLOCK_APPROX_EPSILON * perimeter,
                    True,
                )
            )
            if not (
                COLUMN_BLOCK_MIN_APPROX_VERTICES
                <= approx_vertices
                <= COLUMN_BLOCK_MAX_APPROX_VERTICES
            ):
                continue
            edge_margin_x = int(round(width * COLUMN_BLOCK_EDGE_MARGIN_RATIO))
            edge_margin_y = int(round(height * COLUMN_BLOCK_EDGE_MARGIN_RATIO))
            if (
                x < edge_margin_x
                or y < edge_margin_y
                or x + box_w > width - edge_margin_x
                or y + box_h > height - edge_margin_y
            ):
                continue
            quad = self._ordered_quad(cv2.boxPoints(rect))
            destination = np.array(
                ((0, 0), (127, 0), (127, 127), (0, 127)), dtype=np.float32
            )
            transform = cv2.getPerspectiveTransform(quad, destination)
            rectified_white = cv2.warpPerspective(white, transform, (128, 128))
            inner_margin = 26
            inner = rectified_white[
                inner_margin:128 - inner_margin,
                inner_margin:128 - inner_margin,
            ]
            inner_white_ratio = cv2.countNonZero(inner) / float(max(1, inner.size))
            if not (
                COLUMN_BLOCK_MIN_INNER_WHITE_RATIO
                <= inner_white_ratio
                <= COLUMN_BLOCK_MAX_INNER_WHITE_RATIO
            ):
                continue
            edge_band = 15
            edge_ratios = (
                cv2.countNonZero(rectified_white[:edge_band, :])
                / float(edge_band * 128),
                cv2.countNonZero(rectified_white[128 - edge_band:, :])
                / float(edge_band * 128),
                cv2.countNonZero(rectified_white[:, :edge_band])
                / float(edge_band * 128),
                cv2.countNonZero(rectified_white[:, 128 - edge_band:])
                / float(edge_band * 128),
            )
            if min(edge_ratios) < COLUMN_BLOCK_MIN_EDGE_WHITE_RATIO:
                continue
            corner_side = 20
            corner_ratios = (
                cv2.countNonZero(rectified_white[:corner_side, :corner_side])
                / float(corner_side * corner_side),
                cv2.countNonZero(rectified_white[:corner_side, -corner_side:])
                / float(corner_side * corner_side),
                cv2.countNonZero(rectified_white[-corner_side:, :corner_side])
                / float(corner_side * corner_side),
                cv2.countNonZero(rectified_white[-corner_side:, -corner_side:])
                / float(corner_side * corner_side),
            )
            if min(corner_ratios) < COLUMN_BLOCK_MIN_CORNER_WHITE_RATIO:
                continue
            if 1.0 - inner_white_ratio < COLUMN_BLOCK_MIN_INNER_NONWHITE_RATIO:
                continue
            if (4.0 * math.pi * area / float(max(1.0, perimeter * perimeter))) > COLUMN_BLOCK_MAX_CIRCULARITY:
                continue
            contour_mask = np.zeros_like(white)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            ring_kernel_size = max(7, int(round(max(box_w, box_h) * 0.16)))
            if ring_kernel_size % 2 == 0:
                ring_kernel_size += 1
            expanded = cv2.dilate(
                contour_mask,
                cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE,
                    (ring_kernel_size, ring_kernel_size),
                ),
            )
            green_ring = cv2.bitwise_and(
                expanded,
                cv2.bitwise_not(contour_mask),
            )
            green_ring_pixels = green_ring > 0
            green_ring_support = cv2.countNonZero(
                green[green_ring_pixels]
            ) / float(max(1, int(np.count_nonzero(green_ring_pixels))))
            if green_ring_support < COLUMN_BLOCK_MIN_GREEN_RING_SUPPORT:
                continue
            pad = max(10, int(round(max(box_w, box_h) * 0.18)))
            x0, y0 = max(0, x - pad), max(0, y - pad)
            x1, y1 = min(width, x + box_w + pad), min(height, y + box_h + pad)
            green_support = cv2.countNonZero(green[y0:y1, x0:x1]) / float(
                max(1, (y1 - y0) * (x1 - x0))
            )
            if green_support < COLUMN_BLOCK_MIN_GREEN_SUPPORT:
                continue
            # minAreaRect may choose either edge as the reference for a
            # nearly square target. Keep the smallest signed edge tilt so a
            # real -9 degree block is not reported as an 81 degree block.
            angle = float(rect[2])
            while angle <= -45.0:
                angle += 90.0
            while angle > 45.0:
                angle -= 90.0
            blocks.append({
                "kind": "column_block",
                "color": "white_edge",
                "source": "column_white_edge_geometry",
                "center": (int(round(rect[0][0])), int(round(rect[0][1]))),
                "box": quad.astype(np.int32),
                "bbox": (int(x), int(y), int(box_w), int(box_h)),
                "projected_area": max(area, float(rect_w * rect_h)),
                "approx_vertices": approx_vertices,
                "inner_white_ratio": round(float(inner_white_ratio), 4),
                "green_ring_support": round(float(green_ring_support), 4),
                "fully_visible": x > 2 and y > 2 and x + box_w < width - 2 and y + box_h < height - 2,
                "angle": round(float(angle), 1),
                "parallel": True,
                "orientation_agnostic": True,
            })
        return blocks

    def _infer_column_block_from_letter(self, frame, letter):
        """Build a local white-block candidate when the glyph splits the border."""
        if letter.get("kind") != "letter" or not letter.get("fully_visible", True):
            return None
        bbox = letter.get("bbox", ())
        if len(bbox) < 4:
            return None
        height, width = frame.shape[:2]
        x, y, box_width, box_height = (int(value) for value in bbox[:4])
        if min(box_width, box_height) < COLUMN_BLOCK_MIN_SIDE:
            return None
        edge_x = int(width * COLUMN_BLOCK_EDGE_MARGIN_RATIO)
        edge_y = int(height * COLUMN_BLOCK_EDGE_MARGIN_RATIO)
        if x < edge_x or y < edge_y or x + box_width > width - edge_x or y + box_height > height - edge_y:
            return None
        aspect = box_width / float(max(1, box_height))
        if not 0.65 <= aspect <= 1.55:
            return None
        angle = float(letter.get("angle", 0.0))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(
            hsv,
            np.array((0, 0, 190), dtype=np.uint8),
            np.array((180, 135, 255), dtype=np.uint8),
        )
        green = cv2.inRange(
            hsv,
            np.array((COLUMN_BLOCK_GREEN_HUE_RANGE[0], COLUMN_BLOCK_GREEN_MIN_SATURATION, COLUMN_BLOCK_GREEN_MIN_VALUE), dtype=np.uint8),
            np.array((COLUMN_BLOCK_GREEN_HUE_RANGE[1], 255, 255), dtype=np.uint8),
        )
        patch = white[y:y + box_height, x:x + box_width]
        if cv2.countNonZero(patch) / float(max(1, patch.size)) < 0.30:
            return None
        edge_band = max(2, int(round(min(box_width, box_height) * 0.12)))
        edge_ratios = (
            cv2.countNonZero(white[y:y + edge_band, x:x + box_width]) / float(max(1, edge_band * box_width)),
            cv2.countNonZero(white[y + box_height - edge_band:y + box_height, x:x + box_width]) / float(max(1, edge_band * box_width)),
            cv2.countNonZero(white[y:y + box_height, x:x + edge_band]) / float(max(1, box_height * edge_band)),
            cv2.countNonZero(white[y:y + box_height, x + box_width - edge_band:x + box_width]) / float(max(1, box_height * edge_band)),
        )
        if min(edge_ratios) < 0.25:
            return None
        margin_x = max(4, int(round(box_width * 0.16)))
        margin_y = max(4, int(round(box_height * 0.16)))
        inner = white[y + margin_y:y + box_height - margin_y, x + margin_x:x + box_width - margin_x]
        inner_ratio = cv2.countNonZero(inner) / float(max(1, inner.size))
        if not 0.30 <= inner_ratio <= 0.98:
            return None
        mask = np.zeros_like(white)
        cv2.rectangle(mask, (x, y), (x + box_width, y + box_height), 255, -1)
        kernel = max(7, int(round(max(box_width, box_height) * 0.16)))
        if kernel % 2 == 0:
            kernel += 1
        expanded = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel, kernel)))
        ring = cv2.bitwise_and(expanded, cv2.bitwise_not(mask))
        ring_pixels = ring > 0
        ring_support = cv2.countNonZero(green[ring_pixels]) / float(max(1, int(np.count_nonzero(ring_pixels))))
        if ring_support < 0.30:
            return None
        box = np.asarray(letter.get("box", ()), dtype=np.int32)
        if box.size != 8:
            box = np.asarray(((x, y), (x + box_width, y), (x + box_width, y + box_height), (x, y + box_height)), dtype=np.int32)
        return {
            "kind": "column_block",
            "color": "white_edge",
            "source": "column_letter_box_inferred_geometry",
            "center": tuple(map(int, letter.get("center", (x + box_width // 2, y + box_height // 2)))),
            "box": box.reshape(4, 2),
            "bbox": (x, y, box_width, box_height),
            "projected_area": float(box_width * box_height),
            "inner_white_ratio": round(float(inner_ratio), 4),
            "green_ring_support": round(float(ring_support), 4),
            "fully_visible": True,
            "angle": round(angle, 1),
            "parallel": True,
            "orientation_agnostic": True,
        }

    def detect(
        self,
        frame,
        mode=DETECTION_MODE_IDLE,
        field_name="red",
        target_letters=(),
        task3_roi=None,
    ):
        if frame is None or not isinstance(frame, np.ndarray) or frame.ndim != 3:
            return []
        if mode == DETECTION_MODE_WHITE_LINE:
            return []

        field_name = str(field_name or "red").strip().lower()
        if field_name not in {"red", "blue"}:
            field_name = "red"
        target_letters = tuple(str(letter).upper() for letter in target_letters)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        rings = []
        colored_balls = []
        letters = []
        if mode == DETECTION_MODE_DISC_BALLS:
            colored_balls = self._detect_balls(
                hsv,
                colors=(field_name, "yellow"),
            )
        elif mode == DETECTION_MODE_PLATFORM_TARGETS:
            # Use standalone task-two ring depth without the generic ratio overwrite.
            selected_letters = {
                str(letter).upper()
                for letter in target_letters
                if str(letter).upper() in LETTERS
            }
            letters = [
                det for det in self._detect_letters(frame)
                if det.get("letter") in selected_letters
            ]
            return [*letters, *detect_platform_rings(frame)]
        elif mode == DETECTION_MODE_COLUMN_LETTERS:
            letters = self._detect_letters(frame)
            blocks = self._detect_column_blocks(frame)
            inferred = [
                block for item in letters
                for block in [self._infer_column_block_from_letter(frame, item)]
                if block is not None
            ]
            return [*blocks, *inferred, *letters]
        elif mode == DETECTION_MODE_COLUMN_BLOCKS:
            return self._detect_column_blocks(frame)
        elif mode == DETECTION_MODE_COLUMN_ROTATED_LETTERS:
            letters = self._detect_task3_rotated_letters(frame, task3_roi)
            blocks = self._detect_column_blocks(frame)
            inferred = [
                block for item in letters
                for block in [self._infer_column_block_from_letter(frame, item)]
                if block is not None
            ]
            return [*blocks, *inferred, *letters]
        else:
            rings = self._detect_rings(hsv)
            colored_balls = self._detect_balls(hsv, rings)
            letters = self._detect_letters(frame)
        detections = [*colored_balls, *rings, *letters]
        self._add_frame_ratios(detections, frame.shape)
        return detections

    @staticmethod
    def _add_frame_ratios(detections, frame_shape):
        height, width = frame_shape[:2]
        frame_area = max(1, width * height)
        short_side = max(1, min(width, height))

        for det in detections:
            if det.get("kind") == "ball":
                radius = float(det.get("radius", 0))
                circle_area = np.pi * radius * radius
                det["area_ratio"] = circle_area / frame_area
                det["area_percent"] = det["area_ratio"] * 100.0
                det["diameter_ratio"] = (2.0 * radius) / short_side
                det["distance_cm"] = TargetDetector._estimate_distance_cm(
                    det["area_percent"],
                    RING_DISTANCE_OFFSET_CM,
                    RING_DISTANCE_SCALE_CM,
                )
            elif det.get("kind") == "ring":
                radius = float(det.get("outer_radius", 0))
                circle_area = np.pi * radius * radius
                det["area_ratio"] = circle_area / frame_area
                det["area_percent"] = det["area_ratio"] * 100.0
                det["diameter_ratio"] = (2.0 * radius) / short_side
                det["distance_cm"] = TargetDetector._estimate_distance_cm(
                    det["area_percent"],
                    BALL_DISTANCE_OFFSET_CM,
                    BALL_DISTANCE_SCALE_CM,
                )

    @staticmethod
    def _estimate_distance_cm(area_percent, offset_cm, scale_cm):
        if area_percent <= 0.0:
            return None
        return offset_cm + scale_cm / np.sqrt(area_percent)

    def draw(self, frame, detections):
        out = frame.copy()
        height, width = out.shape[:2]
        center_x = width // 2
        center_y = height // 2
        cv2.line(out, (center_x, 0), (center_x, height), (255, 255, 255), 1)
        cv2.line(out, (0, center_y), (width, center_y), (255, 255, 255), 1)
        cv2.circle(out, (center_x, center_y), 10, (0, 255, 255), 2)
        for det in detections:
            color = det["color"]
            bgr = DRAW_COLORS.get(color, (0, 255, 0))
            label = f"{color} {det['kind']}"

            if det["kind"] == "ball":
                x, y, w, h = det["bbox"]
                cx, cy = det["center"]
                radius = det["radius"]
                area_percent = det.get("area_percent", 0.0)
                distance_cm = det.get("distance_cm")
                if distance_cm is None:
                    label = f"{label} fill {area_percent:.2f}%"
                else:
                    label = f"{label} fill {area_percent:.2f}% d {distance_cm:.1f}cm"
                cv2.rectangle(out, (x, y), (x + w, y + h), bgr, 2)
                cv2.circle(out, (cx, cy), radius, bgr, 2)
                self._label(out, label, x, y, bgr)
            elif det["kind"] == "letter":
                cv2.drawContours(out, [det["box"]], 0, bgr, 2)
                x, y, w, h = det["bbox"]
                cv2.rectangle(out, (x, y), (x + w, y + h), bgr, 1)
                area_percent = det.get("area_percent", 0.0)
                distance_cm = det.get("distance_cm")
                confidence = det.get("confidence", 0.0)
                label = f"white letter {det.get('letter', '?')} conf {confidence:.0f}%"
                if distance_cm is None:
                    label = f"{label} fill {area_percent:.2f}%"
                else:
                    label = f"{label} fill {area_percent:.2f}% d {distance_cm:.1f}cm"
                if not det.get("fully_visible", True):
                    label = f"{label} EDGE"
                self._label(out, label, x, y, bgr)
            elif det["kind"] == "ring":
                cx, cy = det["center"]
                cv2.circle(out, (cx, cy), det["outer_radius"], bgr, 2)
                cv2.circle(out, (cx, cy), det["inner_radius"], bgr, 2)
                x, y, w, h = det["bbox"]
                cv2.rectangle(out, (x, y), (x + w, y + h), bgr, 1)
                area_percent = det.get("area_percent", 0.0)
                distance_cm = det.get("distance_cm")
                if distance_cm is None:
                    label = f"{label} fill {area_percent:.2f}%"
                else:
                    label = (
                        f"{label} fill {area_percent:.2f}% "
                        f"depth {distance_cm:.1f}cm"
                    )
                self._label(out, label, x, y, bgr)

        info = self.summary(detections)
        cv2.putText(out, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)
        return out, info

    @staticmethod
    def _label(img, text, x, y, bgr):
        y = max(22, y)
        cv2.putText(img, text, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2)

    @staticmethod
    def summary(detections):
        order = [
            ("yellow", "ball"),
            ("red", "ball"),
            ("blue", "ball"),
            ("red", "ring"),
            ("blue", "ring"),
        ]
        parts = []
        for color, kind in order:
            matching = [
                det
                for det in detections
                if det.get("color") == color and det.get("kind") == kind
            ]
            count = len(matching)
            if count:
                if kind == "ball":
                    measurements = ",".join(
                        TargetDetector._format_area_measurement(det)
                        for det in matching
                    )
                    parts.append(f"{color}-{kind}:{count} {measurements}")
                else:
                    parts.append(f"{color}-{kind}:{count}")
        for letter in LETTERS:
            matching = [
                det for det in detections
                if det.get("kind") == "letter" and det.get("letter") == letter
            ]
            if matching:
                measurements = ",".join(
                    TargetDetector._format_area_measurement(det)
                    for det in matching
                )
                parts.append(f"white-letter:{letter}:{len(matching)} {measurements}")
        return " | ".join(parts) if parts else "searching selected targets..."

    @staticmethod
    def _format_area_measurement(det):
        area_percent = det.get("area_percent", 0.0)
        distance_cm = det.get("distance_cm")
        if distance_cm is None:
            return f"fill={area_percent:.2f}%"
        return (
            f"fill={area_percent:.2f}% "
            f"dist={TargetDetector._display_distance_cm(det):.1f}cm"
        )

    @staticmethod
    def _display_distance_cm(det):
        distance_cm = det.get("distance_cm")
        if distance_cm is None:
            return None
        return float(distance_cm)

    def _mask(self, hsv, color):
        return MASK_BUILDERS[color](hsv)

    def _detect_balls(self, hsv, rings=None, colors=None):
        results = []
        rings = rings or []
        if colors is None:
            detectors = BALL_DETECTORS
        else:
            wanted = {str(color).lower() for color in colors}
            detectors = tuple(
                detector
                for detector in BALL_DETECTORS
                if str(detector.COLOR_NAME).lower() in wanted
            )
        for detector in detectors:
            results.extend(detector.detect(hsv, rings))
        return results

    def _detect_rings(self, hsv, colors=None):
        results = []
        ring_colors = SHAPE_COLORS if colors is None else tuple(colors)
        for color in ring_colors:
            mask = self._mask(hsv, color)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            if hierarchy is None:
                continue
            hierarchy = hierarchy[0]
            for idx, contour in enumerate(contours):
                child_idx = hierarchy[idx][2]
                if child_idx < 0:
                    continue
                area = cv2.contourArea(contour)
                outer_circularity = self._circularity(contour)
                if area < 300 or outer_circularity < 0.60:
                    continue
                (cx, cy), outer_radius = cv2.minEnclosingCircle(contour)
                child = contours[child_idx]
                child_area = cv2.contourArea(child)
                (inner_center, inner_radius) = cv2.minEnclosingCircle(child)
                inner_circularity = self._circularity(child)
                if outer_radius < 14 or inner_radius < 5 or child_area < 40:
                    continue
                hole_ratio = inner_radius / outer_radius if outer_radius > 0 else 0
                center_offset = np.hypot(inner_center[0] - cx, inner_center[1] - cy)
                if not 0.24 <= hole_ratio <= 0.72:
                    continue
                if center_offset > outer_radius * 0.35 or inner_circularity < 0.42:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                results.append(
                    {
                        "kind": "ring",
                        "color": color,
                        "center": (int(cx), int(cy)),
                        "outer_radius": int(outer_radius),
                        "inner_radius": int(inner_radius),
                        "bbox": (x, y, w, h),
                        "score": round(outer_circularity, 2),
                    }
                )
        return results

    @staticmethod
    def _center_fill(mask, cx, cy, radius):
        radius = max(3, int(radius * 0.28))
        center_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.circle(center_mask, (int(cx), int(cy)), radius, 255, -1)
        center_area = np.count_nonzero(center_mask)
        if center_area == 0:
            return 0.0
        colored_area = np.count_nonzero(cv2.bitwise_and(mask, center_mask))
        return colored_area / center_area

    @staticmethod
    def _mask_ratio_in_circle(mask, cx, cy, radius, scale=0.82):
        if mask is None:
            return 0.0
        radius = max(3, int(radius * scale))
        circle_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.circle(circle_mask, (int(cx), int(cy)), radius, 255, -1)
        circle_area = np.count_nonzero(circle_mask)
        if circle_area == 0:
            return 0.0
        masked_area = np.count_nonzero(cv2.bitwise_and(mask, circle_mask))
        return masked_area / circle_area

    @staticmethod
    def _circle_hsv_mean(hsv, cx, cy, radius, scale=0.55):
        radius = max(3, int(radius * scale))
        circle_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.circle(circle_mask, (int(cx), int(cy)), radius, 255, -1)
        sat_mean = cv2.mean(hsv[:, :, 1], mask=circle_mask)[0]
        val_mean = cv2.mean(hsv[:, :, 2], mask=circle_mask)[0]
        return sat_mean, val_mean

    @staticmethod
    def _overlaps_existing_ring(bbox, color, rings):
        x, y, w, h = bbox
        area = max(1, w * h)
        for ring in rings:
            if ring.get("color") != color:
                continue
            rx, ry, rw, rh = ring["bbox"]
            ix0 = max(x, rx)
            iy0 = max(y, ry)
            ix1 = min(x + w, rx + rw)
            iy1 = min(y + h, ry + rh)
            if ix1 <= ix0 or iy1 <= iy0:
                continue
            overlap = (ix1 - ix0) * (iy1 - iy0) / area
            if overlap > 0.45:
                return True
        return False

    @staticmethod
    def _overlaps_existing_detection(bbox, detections, min_overlap=0.38):
        x, y, w, h = bbox
        area = max(1, w * h)
        for det in detections:
            dx, dy, dw, dh = det.get("bbox", (0, 0, 0, 0))
            ix0 = max(x, dx)
            iy0 = max(y, dy)
            ix1 = min(x + w, dx + dw)
            iy1 = min(y + h, dy + dh)
            if ix1 <= ix0 or iy1 <= iy0:
                continue
            overlap = (ix1 - ix0) * (iy1 - iy0) / area
            if overlap > min_overlap:
                return True
        return False

    @staticmethod
    def _external_contours(mask):
        return external_contours(mask)

    @staticmethod
    def _circularity(contour):
        return contour_circularity(contour)


def parse_target_letters(value, default=LETTERS):
    if value is None:
        return tuple(default)
    if isinstance(value, (list, tuple, set, frozenset)):
        letters = [str(item).strip().upper() for item in value]
    else:
        letters = [item.strip().upper() for item in str(value).split(",")]
    allowed = [item for item in letters if item in LETTERS]
    return tuple(dict.fromkeys(allowed)) or tuple(default)

def parse_device(value):
    return int(value) if str(value).isdigit() else value


def camera_identity(device):
    value = str(device or "").strip()
    if not value or value == "auto":
        return value
    try:
        return str(Path(value).resolve())
    except OSError:
        return value


def camera_candidates(device):
    if device != "auto":
        return [device]

    candidates = []
    by_id = Path("/dev/v4l/by-id")
    if by_id.exists():
        for item in sorted(by_id.iterdir()):
            if "video-index0" in item.name or "camera" in item.name.lower():
                try:
                    candidates.append(str(item.resolve()))
                except OSError:
                    pass

    for item in sorted(Path("/dev").glob("video*"), key=lambda p: p.name):
        if item.name[5:].isdigit():
            candidates.append(str(item))

    deduped = []
    seen = set()
    for item in candidates:
        if item not in seen:
            seen.add(item)
            deduped.append(item)
    return deduped


def open_camera(device, width, height, fps, role="CAMERA"):
    errors = []
    for candidate in camera_candidates(device):
        # Set the V4L2 format before OpenCV opens the node. Without this,
        # reconnects can leave the camera at its previous 640x480 mode even
        # though the route protocol requires 800x600.
        subprocess.run(
            [
                "v4l2-ctl",
                "-d",
                str(candidate),
                f"--set-fmt-video=width={int(width)},height={int(height)},pixelformat=MJPG",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        cap = cv2.VideoCapture(parse_device(candidate), cv2.CAP_V4L2)
        read_timeout_prop = getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None)
        open_timeout_prop = getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", None)
        if open_timeout_prop is not None:
            cap.set(open_timeout_prop, 1200)
        if read_timeout_prop is not None:
            cap.set(read_timeout_prop, 1200)
        # Keep the V4L2 queue shallow. A deep queue makes the preview show
        # frames from seconds ago when detection takes longer than the FPS.
        buffer_prop = getattr(cv2, "CAP_PROP_BUFFERSIZE", None)
        if buffer_prop is not None:
            cap.set(buffer_prop, 1)
        subprocess.run(
            [
                "v4l2-ctl",
                "-d",
                str(candidate),
                "--set-ctrl=exposure_dynamic_framerate=0",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        if not cap.isOpened():
            errors.append(f"{candidate}: open failed")
            cap.release()
            continue
        # Require one real frame before publishing the camera. The latest
        # frame reader drains the V4L2 queue after this point, so extra startup
        # reads only delay the H7 white-line handshake.
        for _ in range(1):
            ok, _ = cap.read()
            if ok:
                actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = cap.get(cv2.CAP_PROP_FPS)
                if actual_width != int(width) or actual_height != int(height):
                    errors.append(
                        f"{candidate}: negotiated {actual_width}x{actual_height}, "
                        f"expected {int(width)}x{int(height)}"
                    )
                    cap.release()
                    continue
                print(
                    f"{role} CAMERA: {candidate} {actual_width}x{actual_height} "
                    f"reported_fps={actual_fps:.1f}"
                )
                return cap
            time.sleep(0.03)
        errors.append(f"{candidate}: no frames")
        cap.release()
    raise RuntimeError(f"Cannot open {role.lower()} camera. Tried: " + "; ".join(errors))


class LatestFrameReader:
    """Continuously drain a camera and expose only its newest frame."""

    def __init__(self, cap):
        self.cap = cap
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="camera-latest-frame",
            daemon=True,
        )
        self._frame = None
        self._last_read_s = 0.0
        self._last_frame_at = 0.0
        self._error = None
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                started = time.perf_counter()
                ok, frame = self.cap.read()
                elapsed = time.perf_counter() - started
            except Exception as exc:
                with self._lock:
                    self._error = (
                        f"camera read exception: {type(exc).__name__}: {exc}"
                    )
                return
            if not ok:
                with self._lock:
                    self._error = "camera read failed"
                return
            with self._lock:
                self._frame = frame
                self._last_frame_at = time.monotonic()
                self._last_read_s = elapsed

    def latest(self):
        with self._lock:
            if self._frame is None:
                return False, None, self._last_read_s, self._error
            if time.monotonic() - self._last_frame_at > 0.5:
                return False, None, self._last_read_s, "camera frame stale"
            return True, self._frame, self._last_read_s, self._error

    @property
    def error(self):
        with self._lock:
            return self._error

    def close(self):
        self._stop.set()
        self.cap.release()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=0.5)


def set_pipewire(active):
    command = "start" if active else "stop"
    services = ["pipewire.socket", "pipewire.service", "pipewire-media-session.service"]
    subprocess.run(
        ["systemctl", "--user", command, *services],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def stop_old_camera_viewers():
    user = getpass.getuser()
    for name in ("cheese", "guvcview", "gst-launch-1.0"):
        subprocess.run(
            ["pkill", "-u", user, "-x", name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    subprocess.run(
        ["pkill", "-u", user, "-f", "/home/cat/bin/camera-preview"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def ensure_display_env():
    uid = os.getuid()
    os.environ.setdefault("XDG_RUNTIME_DIR", f"/run/user/{uid}")
    os.environ.setdefault("DBUS_SESSION_BUS_ADDRESS", f"unix:path=/run/user/{uid}/bus")
    os.environ.setdefault("WAYLAND_DISPLAY", "wayland-0")
    os.environ.setdefault("DISPLAY", ":0")


def local_ip_hint():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except OSError:
        return "127.0.0.1"


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=os.environ.get("CAMERA_DEVICE", "auto"))
    parser.add_argument(
        "--secondary-device",
        default=os.environ.get("SECONDARY_CAMERA_DEVICE", ""),
        help="secondary camera used only for task-two initial letter preselection",
    )
    parser.add_argument("--width", type=int, default=ROUTE_CAMERA_WIDTH)
    parser.add_argument("--height", type=int, default=ROUTE_CAMERA_HEIGHT)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--detect-every-n-frames", type=int, default=1)
    parser.add_argument("--detection-scale", type=float, default=1.0)
    parser.add_argument("--detection-smoothing-alpha", type=float, default=0.32)
    parser.add_argument("--detection-smoothing-match-px", type=float, default=90.0)
    parser.add_argument(
        "--fresh-detection-on-lock",
        action="store_true",
        help="only let new detector results advance visual lock and centering",
    )
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--no-web", action="store_true")
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--keep-pipewire", action="store_true")
    parser.add_argument("--keep-camera-users", action="store_true")
    parser.add_argument(
        "--htd85-uart",
        default=os.environ.get(
            "HTD85_UART",
            "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C82109853-if00",
        ),
        help="the single Hiwonder HTD85 bus for physical IDs 1/2/3/6/14/15/17",
    )
    parser.add_argument("--htd85-baud", type=int, default=int(os.environ.get("HTD85_BAUD", "115200")))
    parser.add_argument("--htd85-arm-time-ms", type=int, default=int(os.environ.get("HTD85_ARM_TIME_MS", "500")))
    parser.add_argument("--htd85-gripper-time-ms", type=int, default=int(os.environ.get("HTD85_GRIPPER_TIME_MS", "200")))
    parser.add_argument("--htd85-aux-time-ms", type=int, default=int(os.environ.get("HTD85_AUX_TIME_MS", "200")))
    parser.add_argument("--htd85-repeat", type=int, default=int(os.environ.get("HTD85_REPEAT", "1")))
    parser.add_argument("--trigger-color", default="red", choices=BALL_COLORS)
    parser.add_argument("--trigger-kind", default="letter", choices=("ball", "letter", "ring", "any"))
    parser.add_argument("--target-letters", default="A,B,C,D")
    parser.add_argument("--enable-arm-preview", action="store_true")
    parser.add_argument("--preview-id1", type=int, default=READY_ID1_TICK)
    parser.add_argument("--preview-id2", type=int, default=READY_ID2_TICK)
    parser.add_argument("--preview-id4", type=int, default=GRIPPER_CLOSED_TICK)
    parser.add_argument("--preview-id6", type=int, default=BASE_YAW_CENTER_TICK)
    parser.add_argument("--enable-letter-grasp", action="store_true")
    parser.add_argument(
        "--execute-letter-grasp",
        action="store_true",
        default=os.environ.get("ABCD_EXECUTE", os.environ.get("RED_SQUARE_EXECUTE", "")).lower() in {"1", "true", "yes"},
    )
    parser.add_argument("--skip-grasp-startup-sequence", action="store_true")
    parser.add_argument("--grasp-id1-ready", type=int, default=READY_ID1_TICK)
    parser.add_argument("--grasp-id2-ready", type=int, default=READY_ID2_TICK)
    parser.add_argument("--grasp-id4-closed", type=int, default=GRIPPER_CLOSED_TICK)
    parser.add_argument("--grasp-id4-open", type=int, default=GRIPPER_OPEN_TICK)
    parser.add_argument("--grasp-center-deadband-px", type=float, default=20.0)
    parser.add_argument("--grasp-stable-frames", type=int, default=5)
    parser.add_argument("--grasp-command-interval", type=float, default=1.40)
    parser.add_argument("--grasp-retrigger-cooldown", type=float, default=0.0)
    parser.add_argument("--grasp-id2-pixel-gain", type=float, default=0.12)
    parser.add_argument("--grasp-id6-pixel-gain", type=float, default=0.18)
    parser.add_argument("--grasp-id6-max-step-ticks", type=int, default=60)
    parser.add_argument("--grasp-id1-pixel-gain-y", type=float, default=0.0)
    parser.add_argument("--grasp-id2-distance-gain", type=float, default=0.25)
    parser.add_argument("--camera-gripper-offset-mm", type=float, default=50.0)
    parser.add_argument("--target-gripper-distance-mm", type=float, default=20.0)
    parser.add_argument("--grasp-distance-deadband-mm", type=float, default=12.0)
    parser.add_argument("--grasp-max-step-ticks", type=int, default=18)
    parser.add_argument("--servo-angle-gap-deg", type=float, default=MIN_ANGLE_GAP_DEG)
    parser.add_argument("--grasp-id1-min", type=int, default=ID1_SAFE_LIMITS[0])
    parser.add_argument("--grasp-id1-max", type=int, default=ID1_SAFE_LIMITS[1])
    parser.add_argument("--grasp-id2-min", type=int, default=ID2_SAFE_LIMITS[0])
    parser.add_argument("--grasp-id2-max", type=int, default=ID2_SAFE_LIMITS[1])
    parser.add_argument("--grasp-id2-center-min", type=int, default=None)
    parser.add_argument("--grasp-id2-center-max", type=int, default=None)
    parser.add_argument("--grasp-one-shot", action="store_true")
    parser.add_argument("--camera-gripper-vertical-offset-mm", type=float, default=0.0)
    parser.add_argument("--max-lateral-offset-mm", type=float, default=45.0)
    parser.add_argument("--max-one-shot-ik-error-mm", type=float, default=15.0)
    parser.add_argument("--post-center-retreat-mm", type=float, default=50.0)
    parser.add_argument("--post-center-down-mm", type=float, default=30.0)
    parser.add_argument("--post-center-ik-error-mm", type=float, default=18.0)
    parser.add_argument(
        "--chassis-link",
        action="store_true",
        default=os.environ.get("CHASSIS_LINK", "").lower() in {"1", "true", "yes"},
        help="listen for H7 USB CDC ARM,<station>,START and reply ACK/DONE",
    )
    parser.add_argument(
        "--chassis-home-on-start",
        action="store_true",
        default=os.environ.get("CHASSIS_HOME_ON_START", "").lower() in {"1", "true", "yes"},
        help="start in arm home pose for chassis linkage; stations expand the arm on demand",
    )
    parser.add_argument(
        "--chassis-uart",
        default=os.environ.get("CHASSIS_UART", "auto"),
        help="H7 USB CDC tty, or auto for /dev/ttyACM* and /dev/ttyUSB*",
    )
    parser.add_argument(
        "--chassis-baud",
        type=int,
        default=int(os.environ.get("CHASSIS_BAUD", "115200")),
    )
    parser.add_argument(
        "--station-no-target-timeout",
        type=float,
        default=float(os.environ.get("STATION_NO_TARGET_TIMEOUT", "5.0")),
        help=(
            "finish a formal PLATFORM_PICK station after this many seconds "
            "before any valid target is observed"
        ),
    )
    parser.add_argument(
        "--post-center-direct-descend",
        action="store_true",
        help="after opening ID7, skip overhead and descend using corrected post-center IK",
    )
    parser.add_argument(
        "--simple-vertical-grasp",
        action="store_true",
        default=os.environ.get("SIMPLE_VERTICAL_GRASP", "").lower()
        in {"1", "true", "yes"},
        help="after image centering, open ID7 and descend with ID1 only; keep old IK path disabled",
    )
    parser.add_argument(
        "--vertical-grasp-id1",
        type=int,
        default=int(os.environ.get("VERTICAL_GRASP_ID1", str(HOME_ID1_TICK))),
        help="ID1 tick used for the simple vertical descend",
    )
    parser.add_argument(
        "--vertical-grasp-id2",
        type=int,
        default=int(os.environ.get("VERTICAL_GRASP_ID2", str(READY_ID2_TICK))),
        help="ID2 tick held during the simple vertical descend",
    )
    parser.add_argument("--window-x", type=int, default=None)
    parser.add_argument("--window-y", type=int, default=None)
    parser.add_argument("--window-width", type=int, default=None)
    parser.add_argument("--window-height", type=int, default=None)
    return parser


def main(argv=None):
    args, _ = build_arg_parser().parse_known_args(argv)
    ensure_display_env()
    if (
        args.secondary_device
        and camera_identity(args.device) == camera_identity(args.secondary_device)
    ):
        print(
            "SECONDARY CAMERA disabled: main and secondary resolve to the same device "
            f"({args.device})",
            flush=True,
        )
        args.secondary_device = ""

    state = FrameState()
    detector = TargetDetector()
    detection_smoother = DetectionSmoother(
        alpha=args.detection_smoothing_alpha,
        max_match_px=args.detection_smoothing_match_px,
    )
    target_letters = parse_target_letters(args.target_letters)
    auto_grasp_enabled = args.enable_letter_grasp
    execute_auto_grasp = auto_grasp_enabled and args.execute_letter_grasp
    preview_id4 = (
        args.preview_id4
        if execute_auto_grasp
        else load_last_gripper_target(args.preview_id4)
    )
    arm_preview = ArmPreviewPublisher(
        args.enable_arm_preview or auto_grasp_enabled,
        args.preview_id1,
        args.preview_id2,
        preview_id4,
        args.preview_id6,
    )
    servo_bridge = HiwonderSingleBusServoBridge(
        args.htd85_uart,
        args.htd85_baud,
        enabled=auto_grasp_enabled,
        write_enabled=execute_auto_grasp,
        arm_time_ms=args.htd85_arm_time_ms,
        gripper_time_ms=args.htd85_gripper_time_ms,
        aux_time_ms=args.htd85_aux_time_ms,
        repeat=args.htd85_repeat,
    )
    grasp_controller = TargetGraspController(
        auto_grasp_enabled,
        servo_bridge,
        arm_preview,
        args.grasp_id1_ready,
        args.grasp_id2_ready,
        args.grasp_id4_closed,
        args.grasp_id4_open,
        args.grasp_center_deadband_px,
        args.grasp_stable_frames,
        args.grasp_command_interval,
        args.grasp_retrigger_cooldown,
        args.grasp_id2_pixel_gain,
        args.grasp_id6_pixel_gain,
        args.grasp_id6_max_step_ticks,
        args.grasp_id1_pixel_gain_y,
        args.grasp_id2_distance_gain,
        args.camera_gripper_offset_mm,
        args.target_gripper_distance_mm,
        args.grasp_distance_deadband_mm,
        args.grasp_max_step_ticks,
        (args.grasp_id1_min, args.grasp_id1_max),
        (args.grasp_id2_min, args.grasp_id2_max),
        args.servo_angle_gap_deg,
        startup_sequence=not args.skip_grasp_startup_sequence,
        one_shot=args.grasp_one_shot,
        camera_gripper_vertical_offset_mm=args.camera_gripper_vertical_offset_mm,
        max_lateral_offset_mm=args.max_lateral_offset_mm,
        max_one_shot_ik_error_mm=args.max_one_shot_ik_error_mm,
        post_center_retreat_mm=args.post_center_retreat_mm,
        post_center_down_mm=args.post_center_down_mm,
        post_center_ik_error_mm=args.post_center_ik_error_mm,
        post_center_direct_descend=args.post_center_direct_descend,
        simple_vertical_grasp=args.simple_vertical_grasp,
        vertical_grasp_id1=args.vertical_grasp_id1,
        vertical_grasp_id2=args.vertical_grasp_id2,
        initial_id4=preview_id4,
        id2_center_min=args.grasp_id2_center_min,
        id2_center_max=args.grasp_id2_center_max,
        field_mode=parse_field(args.trigger_color),
        target_letters=target_letters,
    )
    chassis_link = ChassisArmLink(
        args.chassis_link,
        args.chassis_uart,
        args.chassis_baud,
        args.station_no_target_timeout,
    )
    white_line_detector = WhiteLineAlignmentDetector()
    # Formal task two has a separate lower-strip detector.  Task one keeps
    # the generic detector and its fallback unchanged.
    task2_white_line_detector = WhiteLineAlignmentDetector()
    # Formal blue task three uses the same lower-strip geometry after the
    # orbit, but keeps independent temporal history from task two.
    task3_blue_white_line_detector = WhiteLineAlignmentDetector()
    white_line_last_query_sequence = None
    white_line_last_phase = None
    if args.chassis_home_on_start and execute_auto_grasp:
        startup_home_status = grasp_controller.shutdown_contract()
        print(f"CHASSIS LINK startup home requested: {startup_home_status}", flush=True)
        if not servo_bridge.last_command_ok:
            raise RuntimeError(
                "chassis startup home failed; refusing to announce RK,ARM,READY: "
                f"{startup_home_status}"
            )
    if chassis_link.enabled:
        # H7 owns every arm transition. Keep the arm at HOME after process
        # startup and wait for DISC_CATCH PREP_HIGH; never run the legacy
        # standalone HOME -> READY is not used by the formal route.
        grasp_controller.startup_stage = "complete"
        grasp_controller.state = "station_home"
        grasp_controller.status = "H7 link ready; arm held at home"
    server = None
    if not args.no_web:
        server = ThreadedHTTPServer(("0.0.0.0", args.web_port), StreamHandler, state)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"Web view: http://{local_ip_hint()}:{args.web_port}")

    if not args.keep_camera_users:
        stop_old_camera_viewers()
    if not args.keep_pipewire:
        set_pipewire(False)
    # Camera availability is independent from the H7 link.  Start the main
    # service loop even when the camera is absent so RESET/STATUS remain usable
    # and the station state machine can retract on its own deadline.
    cap = None
    camera_reader = None
    secondary_cap = None
    secondary_reader = None
    camera_retry_at = 0.0
    secondary_retry_at = 0.0
    camera_last_error_log = 0.0
    secondary_last_error_log = 0.0
    camera_last_no_frame_log = 0.0
    window_enabled = not args.no_window
    window_close_watcher = None
    if window_enabled:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(
            WINDOW_NAME,
            args.window_width or min(args.width, 1120),
            args.window_height or min(args.height, 630),
        )
        if args.window_x is not None and args.window_y is not None:
            cv2.moveWindow(WINDOW_NAME, args.window_x, args.window_y)
        window_close_watcher = WindowCloseWatcher(WINDOW_NAME)
        window_close_watcher.start()

    shutdown_signal = {"received": None}

    def request_shutdown(signum, _frame):
        shutdown_signal["received"] = signum
        raise KeyboardInterrupt

    for handled_signal in (signal.SIGINT, signal.SIGTERM):
        signal.signal(handled_signal, request_shutdown)

    arm_tune_bridge = ArmTuneFileBridge(
        ARM_TUNE_COMMAND_PATH,
        ARM_TUNE_RESULT_PATH,
        ARM_TUNE_POLL_INTERVAL_S,
    )
    print(
        "Arm tune file: "
        f"write commands to {ARM_TUNE_COMMAND_PATH}; "
        f"read result from {ARM_TUNE_RESULT_PATH}",
        flush=True,
    )
    print("Press q or Esc in the preview window to quit.")
    fps_started = time.monotonic()
    fps_last_log = fps_started
    fps_frames = 0
    display_fps = 0.0
    detection_frame_index = 0
    detections = []
    detection_interval = max(1, args.detect_every_n_frames)
    detection_scale = max(0.4, min(1.0, float(args.detection_scale)))
    detection_executor = concurrent.futures.ProcessPoolExecutor(
        max_workers=1,
        mp_context=multiprocessing.get_context("spawn"),
    )
    secondary_detection_executor = concurrent.futures.ProcessPoolExecutor(
        max_workers=1,
        mp_context=multiprocessing.get_context("spawn"),
    )
    secondary_detection_warmup_future = None
    if args.secondary_device:
        secondary_detection_warmup_future = (
            secondary_detection_executor.submit(secondary_detector_warmup_worker)
        )
    camera_executor = concurrent.futures.ThreadPoolExecutor(
        max_workers=1,
        thread_name_prefix="camera-open",
    )
    camera_open_future = None
    secondary_open_future = None
    detection_future = None
    detection_future_mode = None
    detection_future_field = None
    detection_pending_frame = None
    detection_source_frame = None
    detection_epoch = 0
    last_detection_mode = None
    secondary_detection_future = None
    secondary_detection_pending_frame = None

    def current_detection_mode():
        """Select the smallest detector set required by the current H7 phase."""

        station = chassis_link.active_task
        if station == "DISC_CATCH":
            return DETECTION_MODE_DISC_BALLS
        if chassis_link.white_line_active:
            return DETECTION_MODE_WHITE_LINE
        if station == "COLUMN_CATCH":
            if grasp_controller.chassis_station_stage in {
                "column_classify", "column_centering"
            }:
                return DETECTION_MODE_COLUMN_ROTATED_LETTERS
            return DETECTION_MODE_COLUMN_BLOCKS
        if station == "PLATFORM_PICK":
            if grasp_controller.chassis_station_stage == "platform_preselect":
                return DETECTION_MODE_IDLE
            return DETECTION_MODE_PLATFORM_TARGETS
        return DETECTION_MODE_IDLE

    def close_secondary_camera(reason=None):
        nonlocal secondary_cap, secondary_reader
        was_open = secondary_reader is not None or secondary_cap is not None
        if secondary_reader is not None:
            secondary_reader.close()
            secondary_reader = None
        elif secondary_cap is not None:
            secondary_cap.release()
        secondary_cap = None
        if was_open and reason:
            print(f"SECONDARY CAMERA LINK released reason={reason}", flush=True)

    last_detection_frame = None
    secondary_last_frame = None

    def service_secondary_preselect():
        nonlocal secondary_last_frame
        nonlocal secondary_retry_at, secondary_detection_future
        nonlocal secondary_detection_pending_frame
        nonlocal secondary_detection_warmup_future
        if (
            secondary_detection_warmup_future is not None
            and secondary_detection_warmup_future.done()
        ):
            try:
                secondary_detection_warmup_future.result()
            except Exception as exc:
                print(f"secondary detector warmup failed: {exc}", flush=True)
            secondary_detection_warmup_future = None
        if (
            grasp_controller.chassis_station_stage != "platform_preselect"
            or secondary_reader is None
        ):
            if secondary_detection_future is not None and secondary_detection_future.done():
                secondary_detection_future = None
                secondary_detection_pending_frame = None
            return

        if (
            secondary_detection_future is not None
            and secondary_detection_future.done()
        ):
            try:
                secondary_detections, _ = secondary_detection_future.result()
                preselect_info = grasp_controller.update_platform_preselect(
                    secondary_detections,
                    detection_fresh=True,
                )
                if preselect_info:
                    resolve_station_outcome(preselect_info)
            except Exception as exc:
                print(f"secondary detection worker failed: {exc}", flush=True)
            secondary_detection_future = None
            secondary_detection_pending_frame = None

        if grasp_controller.chassis_station_stage != "platform_preselect" or secondary_reader is None:
            return
        secondary_ok, secondary_frame, _, secondary_error = secondary_reader.latest()
        if secondary_error is not None:
            print(
                "SECONDARY CAMERA LINK lost; releasing device and retrying "
                f"(reader_error={secondary_error!r})",
                flush=True,
            )
            close_secondary_camera("reader_error")
            secondary_retry_at = time.monotonic() + 1.0
            return
        if (not secondary_ok or secondary_frame is None
                or secondary_frame is secondary_last_frame
                or secondary_detection_future is not None):
            return
        secondary_last_frame = secondary_frame
        if secondary_frame.shape[:2] != (args.height, args.width):
            secondary_frame = cv2.resize(
                secondary_frame,
                (args.width, args.height),
                interpolation=cv2.INTER_AREA,
            )
        if secondary_detection_future is None:
            secondary_detection_pending_frame = secondary_frame.copy()
            secondary_detection_future = secondary_detection_executor.submit(
                secondary_detection_process_worker,
                secondary_detection_pending_frame,
            )

    def resolve_station_outcome(info):
        """Finish the H7 transaction after every station state-machine tick."""

        if not chassis_link.enabled:
            return info
        error_reason = grasp_controller.consume_chassis_station_error()
        done_reason = grasp_controller.consume_chassis_station_done()
        if (
            error_reason is None
            and done_reason is None
            and chassis_link.active_task is not None
            and grasp_controller.algorithm_stage == "fault"
        ):
            abort_status = grasp_controller.abort_chassis_station(
                "SERVO_OR_CONTROL_FAULT"
            )
            error_reason = (
                grasp_controller.consume_chassis_station_error()
                or "SERVO_OR_CONTROL_FAULT"
            )
            info = f"{info}; arm abort={abort_status}"
        if error_reason:
            station = chassis_link.active_task
            if (
                station == "DISC_CATCH"
                and "HOME_FAILED" not in str(error_reason).upper()
            ):
                recovered = chassis_link.recover_active(error_reason)
                if recovered:
                    return (
                        f"{info}; station RECOVERED={error_reason}; "
                        "ARM_HOME=OK; continue route"
                    )
            if station is not None:
                chassis_link.fail_active(error_reason)
            return f"{info}; station ERR={error_reason}"
        if done_reason:
            if done_reason.startswith("PRESELECT_DONE:"):
                letters = done_reason.split(":")[1:]
                if chassis_link.active_task is not None:
                    chassis_link.finish_preselect(letters)
                close_secondary_camera("preselect_done")
            elif chassis_link.active_task is not None:
                chassis_link.finish_active(done_reason)
            return f"{info}; station DONE={done_reason}"
        return info

    def complete_task3_supplement_hold_if_ready(info):
        """Release H7 only after the optional fixed BLUE recovery grab ends."""
        if (
            chassis_link.active_task != "COLUMN_CATCH"
            or not grasp_controller.consume_task3_supplement_done()
        ):
            return info
        chassis_link.complete_column_catch_hold(
            "COLUMN_CATCH",
            reason="ARM_STOP_DONE_AFTER_SUPPLEMENT",
            success=True,
        )
        return f"{info}; H7 HOLD_DONE sent after supplement"

    def _process_white_line_queries(frame, reason):
        nonlocal white_line_last_query_sequence, white_line_last_phase
        pending_white_line_queries = chassis_link.consume_white_line_queries()
        if not pending_white_line_queries:
            return
        query_sequence = pending_white_line_queries[-1]
        phase = getattr(chassis_link, "white_line_phase", None)
        task2_phase = phase == "TASK2_AFTER_SECONDARY_SHIFT"
        task3_blue_phase = phase == "TASK3_BLUE_WHITE_LINE_ALIGN"
        task3_red_phase = phase == "TASK3_RED_WHITE_LINE_ALIGN"
        task3_phase = task3_blue_phase or task3_red_phase
        if (
            query_sequence != white_line_last_query_sequence
            or phase != white_line_last_phase
        ):
            # A new H7 white-line phase must start from the current camera
            # frame, never from a held result of a previous route stage.
            white_line_detector.reset_tracking()
            task2_white_line_detector.reset_tracking()
            task3_blue_white_line_detector.reset_tracking()
            white_line_last_query_sequence = query_sequence
            white_line_last_phase = phase
        if frame is None or getattr(frame, "size", 0) == 0:
            for sequence in pending_white_line_queries:
                chassis_link.send_white_line_result(sequence, None)
                print(
                    f"WHITE LINE seq={sequence} not found ({reason})",
                    flush=True,
                )
            return
        # H7 accepts only the calibrated 800x600 white-line geometry. Some
        # V4L2 reconnects still deliver 640x480 after negotiation; normalize
        # only this route-specific vision path so coordinates and W/H stay
        # consistent. Target/letter/ball detection keeps the native frame.
        white_line_frame = frame
        if white_line_frame.shape[1] != ROUTE_CAMERA_WIDTH or white_line_frame.shape[0] != ROUTE_CAMERA_HEIGHT:
            white_line_frame = cv2.resize(
                white_line_frame,
                (ROUTE_CAMERA_WIDTH, ROUTE_CAMERA_HEIGHT),
                interpolation=cv2.INTER_LINEAR,
            )
        if task2_phase:
            line_measurement = task2_white_line_detector.detect_task2(white_line_frame)
        elif task3_phase:
            line_measurement = task3_blue_white_line_detector.detect_task3(white_line_frame)
        else:
            line_measurement = white_line_detector.detect(white_line_frame)
        if line_measurement is None and pending_white_line_queries:
            # TEMP DEBUG: save the first unseen frame per query sequence so a
            # missed detection can be inspected offline. Remove after tuning.
            for _dbg_seq in pending_white_line_queries[:1]:
                _dbg_path = f"/tmp/wl_dbg_{_dbg_seq}.jpg"
                if not os.path.exists(_dbg_path):
                    cv2.imwrite(_dbg_path, frame)
                    print(f"WL DEBUG frame saved: {_dbg_path}", flush=True)
        for sequence in pending_white_line_queries:
            chassis_link.send_white_line_result(sequence, line_measurement)
            if line_measurement is None:
                print(
                    f"WHITE LINE seq={sequence} not found ({reason})",
                    flush=True,
                )
            else:
                print(
                    "WHITE LINE "
                    f"phase={phase or 'UNKNOWN'} seq={sequence} "
                    f"y_center={line_measurement['y_at_center']:.1f} "
                    f"angle={line_measurement['angle_deg']:+.2f}",
                    flush=True,
                )

    def service_station_without_frame():
        """Advance timers and safety exits when no camera frame is available."""

        nonlocal camera_last_no_frame_log
        info = "camera unavailable; station timers still running"
        if auto_grasp_enabled and chassis_link.active_task is not None:
            station = chassis_link.active_task
            station_info = grasp_controller.update_chassis_station(
                station,
                [],
                (args.height, args.width, 3),
                detection_fresh=False,
                chassis_link=chassis_link,
            )
            if station_info:
                info = station_info
            elif station == "PLATFORM_PICK":
                # A locked PLATFORM_PICK must finish its already-issued IK
                # stages even if the camera disappears.  With no fresh frame,
                # update() cannot acquire or recenter a new target.
                info = grasp_controller.update(
                    None,
                    (args.height, args.width, 3),
                    detection_fresh=False,
                )
            info = complete_task3_supplement_hold_if_ready(info)
            info = resolve_station_outcome(info)
            now = time.monotonic()
            if (
                chassis_link.active_task is not None
                and chassis_link.no_target_timed_out(
                    now,
                    grasp_controller.searching_for_chassis_target(),
                )
            ):
                timeout_station = chassis_link.active_task
                if timeout_station == "PLATFORM_PICK":
                    timeout_status = grasp_controller.skip_platform_slot(
                        "NO_TARGET_TIMEOUT"
                    )
                    resolve_station_outcome("platform skip returning high")
                else:
                    timeout_status = grasp_controller.retract_for_chassis_timeout(
                        timeout_station
                    )
                    timeout_error = grasp_controller.consume_chassis_station_error()
                    if timeout_error:
                        chassis_link.fail_active(timeout_error)
                    else:
                        chassis_link.finish_active("NO_TARGET_TIMEOUT")
                info = (
                    f"{info}; station {timeout_station} timeout; "
                    f"{timeout_status}"
                )
        _process_white_line_queries(None, "camera unavailable")
        grasp_controller.arm_preview.publish(info, None, grasp_controller.state)
        now = time.monotonic()
        if now - camera_last_no_frame_log >= 5.0:
            camera_last_no_frame_log = now
            print(info, flush=True)
        return info

    perf_started = fps_started
    perf_frames = 0
    perf_detect_frames = 0
    perf_read_s = 0.0
    perf_detect_s = 0.0
    perf_post_s = 0.0
    perf_display_s = 0.0
    try:
        while True:
            detection_fresh = False
            chassis_link.set_ready(
                # Arm/H7 synchronization must not depend on the camera. The
                # camera is required later for white-line and ball vision, but
                # PREP_HIGH must be accepted and sent to the 85KG bus during the arc
                # even while the camera is reconnecting.
                grasp_controller.ready_for_chassis_link(execute_auto_grasp)
            )
            pending_stations = chassis_link.update()
            arm_tune_bridge.poll(
                grasp_controller,
                chassis_link,
                (args.height, args.width, 3),
            )
            if chassis_link.consume_reset_request():
                reset_success, reset_status = grasp_controller.reset_from_chassis()
                chassis_link.complete_reset(reset_success, reset_status)
                pending_stations = []
                print(
                    f"CHASSIS RESET {'DONE' if reset_success else 'FAILED'} | "
                    f"{reset_status}",
                    flush=True,
                )
            if not grasp_controller.set_field_mode(chassis_link.field_mode):
                print(
                    "CHASSIS FIELD update rejected while station is active",
                    flush=True,
                )
            for preselect in chassis_link.consume_preselects():
                preselect_status = grasp_controller.begin_platform_preselect(
                    preselect.get("count", 2)
                )
                print(
                    f"CHASSIS PRESELECT STARTED | {preselect_status}",
                    flush=True,
                )
            platform_slot_requests = chassis_link.consume_platform_slots()
            for aux in chassis_link.consume_aux_requests():
                aux_kwargs = {
                    "pulse": aux.get("pulse"),
                    "time_ms": aux.get("time_ms"),
                }
                if aux.get("channel") is not None:
                    aux_kwargs["channel"] = aux.get("channel")
                    target_name = f"S{aux.get('channel')}"
                else:
                    aux_kwargs["servo_id"] = aux.get("servo_id")
                    target_name = f"ID{aux.get('servo_id')}"
                aux_status = grasp_controller.servo_bridge.send_aux_request(**aux_kwargs)
                aux_success = (
                    grasp_controller.servo_bridge.write_enabled
                    and grasp_controller.servo_bridge.last_command_ok
                )
                chassis_link.complete_aux_request(aux, aux_success, aux_status)
                print(
                    f"CHASSIS AUX_SERVO {target_name}={aux.get('pulse')} "
                    f"T={aux.get('time_ms')}ms "
                    f"done={'yes' if aux_success else 'no'} | {aux_status}",
                    flush=True,
                )
            for prep in chassis_link.consume_preps():
                prep_status = grasp_controller.prepare_chassis_station_high(prep)
                prep_success = (
                    grasp_controller.algorithm_stage != "fault"
                    and (
                        not grasp_controller.servo_bridge.write_enabled
                        or grasp_controller.servo_bridge.last_command_ok
                    )
                )
                # ACK only after the PREP_HIGH command has actually been
                # written to the 85KG bus. H7 can then keep the arc moving
                # without waiting for servo feedback.
                chassis_link.complete_prep(prep, prep_success, prep_status)
                print(
                    f"CHASSIS PREP {prep.get('task', 'UNKNOWN')} | "
                    f"{prep_status} ack={'yes' if prep_success else 'no'}",
                    flush=True,
                )
            for station in pending_stations:
                station_status = grasp_controller.begin_chassis_station(station)
                print(
                    f"CHASSIS STATION {station} STARTED | {station_status}",
                    flush=True,
                )
                if grasp_controller.algorithm_stage == "fault":
                    abort_status = grasp_controller.abort_chassis_station(
                        "START_COMMAND_FAILED"
                    )
                    station_status = resolve_station_outcome(
                        f"{station_status}; start abort={abort_status}"
                    )
                else:
                    # Start the station timer immediately.  The station
                    # controller itself keeps its required pose-settle stage.
                    chassis_link.restart_target_watch()
            for hold in chassis_link.consume_holds():
                hold_status = grasp_controller.hold_formal_task3_arm()
                hold_success = (
                    grasp_controller.chassis_station_error_reason is None
                    and (
                        not grasp_controller.servo_bridge.write_enabled
                        or grasp_controller.servo_bridge.last_command_ok
                    )
                )
                if hold_success and not grasp_controller.task3_supplement_pending():
                    chassis_link.complete_column_catch_hold(
                        hold,
                        reason=hold_status,
                        success=True,
                    )
                elif not hold_success:
                    chassis_link.complete_column_catch_hold(
                        hold,
                        reason=hold_status,
                        success=False,
                    )
                else:
                    print(
                        "CHASSIS TASK3 HOLD deferred until fixed supplement "
                        "grab/place completes",
                        flush=True,
                    )
                print(
                    f"CHASSIS TASK3 HOLD success={'yes' if hold_success else 'no'} | "
                    f"{hold_status}",
                    flush=True,
                )
            for hold_failure in chassis_link.consume_hold_failures():
                failure_status = grasp_controller.freeze_formal_task3_arm()
                print(
                    "CHASSIS TASK3 HOLD FAILURE frozen high | "
                    f"{failure_status}",
                    flush=True,
                )
            for slot_request in platform_slot_requests:
                if slot_request.get("task") != "PLATFORM_PICK":
                    continue
                slot = slot_request.get("slot")
                print(
                    "CHASSIS PLATFORM SLOT REQUEST "
                    f"seq={slot_request.get('sequence')} slot={slot}",
                    flush=True,
                )
            for station in chassis_link.consume_stops():
                station_status = grasp_controller.stop_chassis_station(station)
                if (
                    station == "COLUMN_CATCH"
                    and chassis_link.active_task == "COLUMN_CATCH"
                    and grasp_controller.chassis_station_error_reason is None
                    and not grasp_controller.task3_supplement_pending()
                ):
                    chassis_link.complete_column_catch_hold(
                        station, reason="ARM_STOP_DONE"
                    )
                else:
                    station_status = resolve_station_outcome(station_status)
                print(
                    f"CHASSIS STATION {station} STOPPED | {station_status}",
                    flush=True,
                )
            for retract in chassis_link.consume_retracts():
                retract_success, retract_status = (
                    grasp_controller.retract_formal_task3_arm()
                )
                chassis_link.complete_retract(retract, retract_success, retract_status)
                print(
                    f"CHASSIS TASK3 RETRACT seq={retract.get('sequence')} "
                    f"success={'yes' if retract_success else 'no'} | {retract_status}",
                    flush=True,
                )
            now = time.monotonic()
            if camera_open_future is not None and camera_open_future.done():
                try:
                    cap = camera_open_future.result()
                    camera_reader = LatestFrameReader(cap)
                    detections = []
                    detection_source_frame = None
                    detection_pending_frame = None
                    detection_smoother.tracks = []
                    print("CAMERA LINK restored", flush=True)
                except Exception as exc:
                    cap = None
                    camera_retry_at = now + 1.0
                    if now - camera_last_error_log >= 5.0:
                        camera_last_error_log = now
                        print(f"CAMERA LINK waiting: {exc}", flush=True)
                finally:
                    camera_open_future = None
            if (
                cap is None
                and camera_open_future is None
                and now >= camera_retry_at
            ):
                camera_open_future = camera_executor.submit(
                    open_camera,
                    args.device,
                    args.width,
                    args.height,
                    args.fps,
                    "MAIN",
                )
            preselect_active = (
                grasp_controller.chassis_station_stage == "platform_preselect"
            )
            if not preselect_active and (
                secondary_reader is not None or secondary_cap is not None
            ):
                close_secondary_camera("preselect_inactive")
            if (
                args.secondary_device
                and secondary_cap is None
                and secondary_open_future is None
                and preselect_active
                and now >= secondary_retry_at
            ):
                secondary_open_future = camera_executor.submit(
                    open_camera,
                    args.secondary_device,
                    args.width,
                    args.height,
                    args.fps,
                    "SECONDARY",
                )
            if secondary_open_future is not None and secondary_open_future.done():
                try:
                    opened_secondary = secondary_open_future.result()
                    if grasp_controller.chassis_station_stage == "platform_preselect":
                        secondary_cap = opened_secondary
                        secondary_reader = LatestFrameReader(secondary_cap)
                        print(
                            "SECONDARY CAMERA LINK restored "
                            f"device={args.secondary_device}",
                            flush=True,
                        )
                    else:
                        opened_secondary.release()
                        print(
                            "SECONDARY CAMERA LINK discarded; preselect inactive",
                            flush=True,
                        )
                except Exception as exc:
                    secondary_cap = None
                    secondary_retry_at = now + 1.0
                    if now - secondary_last_error_log >= 5.0:
                        secondary_last_error_log = now
                        print(f"SECONDARY CAMERA LINK waiting: {exc}", flush=True)
                finally:
                    secondary_open_future = None
            if cap is None:
                service_secondary_preselect()
                service_station_without_frame()
                time.sleep(0.02)
                continue
            read_started = time.perf_counter()
            ok, frame, reader_elapsed, reader_error = camera_reader.latest()
            read_elapsed = time.perf_counter() - read_started
            if not ok and reader_error is None:
                # The reader thread may still be acquiring its first frame.
                # Keep secondary recognition and station deadlines alive too.
                service_secondary_preselect()
                service_station_without_frame()
                time.sleep(0.002)
                continue
            if reader_error is not None:
                ok = False
            if not ok:
                print(
                    "CAMERA LINK lost; releasing device and retrying"
                    f" (reader_error={reader_error!r})",
                    flush=True,
                )
                if camera_reader is not None:
                    camera_reader.close()
                    camera_reader = None
                elif cap is not None:
                    cap.release()
                cap = None
                camera_retry_at = time.monotonic() + 0.2
                if detection_future is not None:
                    detection_future.cancel()
                detection_future = None
                detection_future_mode = None
                detection_pending_frame = None
                detection_source_frame = None
                detections = []
                detection_smoother.tracks = []
                # Camera availability must not revoke RK/H7 readiness.  The
                # arm PREP_HIGH transaction is independent of vision and must
                # remain serviceable while the camera reconnects.
                service_station_without_frame()
                time.sleep(0.02)
                continue
            _process_white_line_queries(frame, "frame available")
            service_secondary_preselect()
            requested_detection_mode = current_detection_mode()
            if requested_detection_mode != last_detection_mode:
                print(
                    "VISION MODE "
                    f"{last_detection_mode or 'none'} -> {requested_detection_mode}",
                    flush=True,
                )
                detection_smoother.tracks = []
                detections = []
                detection_source_frame = None
                if detection_future is not None and not detection_future.done():
                    detection_future.cancel()
                last_detection_mode = requested_detection_mode
            detect_elapsed = 0.0
            if detection_future is not None and detection_future.done():
                try:
                    completed_mode = detection_future_mode
                    completed_field = detection_future_field
                    result, detect_elapsed = detection_future.result()
                    current_field = grasp_controller.field_mode.value
                    if (completed_mode == requested_detection_mode
                            and completed_field == current_field):
                        detections = detection_smoother.update(result)
                        perf_detect_frames += 1
                        detection_source_frame = detection_pending_frame
                        detection_epoch += 1
                        for detection in detections:
                            detection["detection_epoch"] = detection_epoch
                        detection_fresh = True
                    else:
                        print(
                            "VISION stale result discarded "
                            f"mode={completed_mode} field={completed_field} "
                            f"current={requested_detection_mode}/{current_field}",
                            flush=True,
                        )
                except Exception as exc:
                    print(f"detection worker failed: {exc}", flush=True)
                detection_future = None
                detection_future_mode = None
                detection_future_field = None
                detection_pending_frame = None
            if (
                detection_frame_index % detection_interval == 0
                and detection_future is None
                and frame is not last_detection_frame
                and requested_detection_mode
                not in {DETECTION_MODE_IDLE, DETECTION_MODE_WHITE_LINE}
            ):
                last_detection_frame = frame
                detection_pending_frame = frame.copy()
                detection_future_mode = requested_detection_mode
                detection_future_field = grasp_controller.field_mode.value
                task3_roi = None
                if (
                    requested_detection_mode == DETECTION_MODE_COLUMN_ROTATED_LETTERS
                    and grasp_controller.column_locked_block is not None
                ):
                    bx, by, bw, bh = grasp_controller.column_locked_block.get(
                        "bbox", (0, 0, 0, 0)
                    )
                    pad_x = max(12, int(round(float(bw) * 0.45)))
                    pad_y = max(12, int(round(float(bh) * 0.45)))
                    task3_roi = (
                        int(bx) - pad_x,
                        int(by) - pad_y,
                        int(bw) + pad_x * 2,
                        int(bh) + pad_y * 2,
                    )
                detection_future = detection_executor.submit(
                    detection_process_worker,
                    detection_pending_frame,
                    detection_scale,
                    requested_detection_mode,
                    grasp_controller.field_mode.value,
                    tuple(
                        grasp_controller.platform_selected_letters
                        if requested_detection_mode == DETECTION_MODE_PLATFORM_TARGETS
                        else target_letters
                    ),
                    task3_roi=task3_roi,
                )
            detection_frame_index += 1
            post_started = time.perf_counter()
            active_target_letters = (
                grasp_controller.platform_selected_letters
                if grasp_controller.active_chassis_station == "PLATFORM_PICK"
                else target_letters
            )
            active_target_kind = (
                "any"
                if grasp_controller.active_chassis_station == "PLATFORM_PICK"
                else args.trigger_kind
            )
            selected_targets = [
                det
                for det in detections
                if det.get("observed", True)
                and grasp_controller.target_policy.matches_platform_target(
                    det, active_target_kind, active_target_letters
                )
            ]
            preview_target = max(
                selected_targets,
                key=lambda det: det.get("bbox", (0, 0, 0, 0))[2]
                * det.get("bbox", (0, 0, 0, 0))[3],
                default=None,
            )
            display_detections = [
                det
                for det in detections
                if det.get("observed", True)
                and not grasp_controller.target_policy.matches_platform_target(
                    det, active_target_kind, active_target_letters
                )
            ]
            if preview_target is not None:
                display_detections.append(preview_target)
            render_frame = (
                detection_source_frame
                if detection_source_frame is not None
                else frame
            )
            output, info = detector.draw(render_frame, display_detections)
            aux_info = ""
            if auto_grasp_enabled:
                chassis_active = (
                    not chassis_link.enabled
                    or chassis_link.active_task is not None
                )
                if chassis_link.enabled:
                    if detection_fresh:
                        # PLATFORM_PICK has its own filtered target gate and
                        # target_seen latch.  Do not let an arbitrary visible
                        # object reset the station's no-target observation
                        # window; only a selected letter or own-field ring
                        # can release that watchdog.
                        if chassis_link.active_task != "PLATFORM_PICK":
                            chassis_link.note_target(preview_target)
                if chassis_active:
                    station_info = None
                    if (
                        detection_fresh
                        and chassis_link.active_task == "PLATFORM_PICK"
                        and grasp_controller.chassis_station_stage is None
                        and preview_target is None
                    ):
                        visible_platform_objects = [
                            det for det in detections
                            if det.get("observed", True)
                            and det.get("kind") in {"letter", "ring"}
                        ]
                        if visible_platform_objects:
                            best_rejected = max(
                                visible_platform_objects,
                                key=lambda det: (
                                    float(det.get("confidence") or 0.0),
                                    float(det.get("projected_area") or 0.0),
                                ),
                            )
                            print(
                                "PLATFORM_PICK observed non-selected target; "
                                "continue target observation window "
                                f"kind={best_rejected.get('kind')} "
                                f"letter={best_rejected.get('letter', '-')} "
                                f"color={best_rejected.get('color', '-')}",
                                flush=True,
                            )
                    if station_info is None:
                        station_info = grasp_controller.update_chassis_station(
                            chassis_link.active_task,
                            detections,
                            frame.shape,
                            detection_fresh=detection_fresh,
                            chassis_link=chassis_link,
                        )
                    station_info = complete_task3_supplement_hold_if_ready(
                        station_info
                    )
                    if station_info is not None:
                        grasp_info = station_info
                    else:
                        if (
                            detection_fresh
                            and preview_target is None
                            and grasp_controller.locked_target is None
                            and grasp_controller.active_chassis_station is None
                        ):
                            aux_info = grasp_controller.update_auxiliary(detections)
                        grasp_info = grasp_controller.update(
                            preview_target,
                            frame.shape,
                            detection_fresh=(
                                detection_fresh or not args.fresh_detection_on_lock
                            ),
                        )
                elif not grasp_controller.startup_complete():
                    grasp_info = grasp_controller.update(None, frame.shape)
                else:
                    grasp_info = (
                        "waiting chassis ARM START"
                        if chassis_link.fd is not None
                        else chassis_link.status
                    )
                    arm_preview.publish(grasp_info, preview_target, "WAIT_CHASSIS")
                grasp_info = resolve_station_outcome(grasp_info)
                now = time.monotonic()
                if (
                    chassis_link.enabled
                    and chassis_link.no_target_timed_out(
                        now,
                        grasp_controller.searching_for_chassis_target(),
                    )
                ):
                    station = chassis_link.active_task
                    if station == "PLATFORM_PICK":
                        timeout_status = grasp_controller.skip_platform_slot(
                            "NO_TARGET_TIMEOUT"
                        )
                        resolve_station_outcome("platform skip returning high")
                    else:
                        timeout_status = grasp_controller.retract_for_chassis_timeout(
                            station
                        )
                        error_reason = grasp_controller.consume_chassis_station_error()
                        if error_reason:
                            chassis_link.fail_active(error_reason)
                        else:
                            chassis_link.finish_active("NO_TARGET_TIMEOUT")
                    grasp_info = (
                        f"chassis station {station} timeout; "
                        f"{timeout_status}"
                    )
                arm_preview.publish(
                    grasp_info,
                    preview_target,
                    grasp_controller.state,
                )
                info = f"{info} | {grasp_info}"
            elif preview_target is not None:
                arm_preview.publish(
                    f"{args.trigger_kind} detected",
                    preview_target,
                )
            else:
                arm_preview.publish(
                    f"searching {args.trigger_kind}"
                )
            if aux_info:
                info = f"{info} | {aux_info}"
            fps_frames += 1
            fps_elapsed = time.monotonic() - fps_started
            if fps_elapsed >= 1.0:
                display_fps = fps_frames / fps_elapsed
                fps_started = time.monotonic()
                fps_frames = 0
            if time.monotonic() - fps_last_log >= 5.0:
                print(f"VISION FPS={display_fps:.1f}", flush=True)
                fps_last_log = time.monotonic()
            info = f"{info} | FPS {display_fps:.1f}"
            state.update(output, info)
            post_elapsed = time.perf_counter() - post_started
            display_started = time.perf_counter()
            if window_enabled:
                cv2.imshow(WINDOW_NAME, output)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                try:
                    if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) == 0:
                        break
                except cv2.error:
                    pass
                if window_close_watcher.closed.is_set():
                    break
            display_elapsed = time.perf_counter() - display_started
            perf_frames += 1
            perf_read_s += read_elapsed
            perf_detect_s += detect_elapsed
            perf_post_s += post_elapsed
            perf_display_s += display_elapsed
            perf_elapsed = time.monotonic() - perf_started
            if perf_elapsed >= 5.0 and perf_frames:
                detect_divisor = max(1, perf_detect_frames)
                print(
                    "VISION PERF "
                    f"read={perf_read_s * 1000.0 / perf_frames:.1f}ms "
                    f"detect={perf_detect_s * 1000.0 / detect_divisor:.1f}ms/detect "
                    f"post={perf_post_s * 1000.0 / perf_frames:.1f}ms "
                    f"display={perf_display_s * 1000.0 / perf_frames:.1f}ms",
                    flush=True,
                )
                perf_started = time.monotonic()
                perf_frames = 0
                perf_detect_frames = 0
                perf_read_s = 0.0
                perf_detect_s = 0.0
                perf_post_s = 0.0
                perf_display_s = 0.0
    except KeyboardInterrupt:
        detail = shutdown_signal["received"]
        if detail is None:
            print("Shutdown requested from terminal.", flush=True)
        else:
            print(f"Shutdown signal {detail} received.", flush=True)
    finally:
        state.running = False
        detection_executor.shutdown(wait=False, cancel_futures=True)
        if (
            secondary_detection_warmup_future is not None
            and not secondary_detection_warmup_future.done()
        ):
            secondary_detection_warmup_future.cancel()
        secondary_detection_executor.shutdown(wait=False, cancel_futures=True)
        if camera_open_future is not None:
            camera_open_future.cancel()
        camera_executor.shutdown(wait=False, cancel_futures=True)
        if camera_reader is not None:
            camera_reader.close()
        elif cap is not None:
            cap.release()
        close_secondary_camera()
        if auto_grasp_enabled and execute_auto_grasp:
            try:
                grasp_controller.shutdown_contract()
            except Exception as exc:
                print(f"shutdown contract error: {exc}", flush=True)
        servo_bridge.close()
        chassis_link.close()
        arm_preview.close()
        if server is not None:
            server.shutdown()
        if window_enabled:
            cv2.destroyAllWindows()
        if window_close_watcher is not None:
            window_close_watcher.stop()
        if not args.keep_pipewire:
            set_pipewire(True)


if __name__ == "__main__":
    main()
