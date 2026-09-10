#!/usr/bin/env python3
"""Detect RoboCup balls, rings, and selected A/B/C/D letter blocks."""

from __future__ import annotations

import argparse
import concurrent.futures
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
SINGLE_ID2_DEADBAND_PX = 45.0
SINGLE_ID2_MIN_STEP_TICKS = 8
SINGLE_ID2_MAX_STEP_TICKS = 40
SINGLE_ID6_MIN_STEP_TICKS = 1
ID6_SAFE_LIMITS = (300, 700)
CENTERING_SLOW_COMMAND_INTERVAL_S = 0.48
CENTERING_SLOW_ID2_GAIN = 0.06
CENTERING_SLOW_ID6_GAIN = 0.045
CENTERING_SLOW_ID2_MAX_STEP_TICKS = 12
CENTERING_SLOW_ID6_MAX_STEP_TICKS = 6
CENTERING_VECTOR_COMPONENT_DEADBAND_PX = 5.0
SPLITTER_YELLOW_TICK = 1300
SPLITTER_OTHER_BALL_TICK = 1600
CATCHER_HOME_TICK = 900
CATCHER_RELEASE_READY_TICK = 1100
POST_GRAB_ID2_RETREAT_TICK = 100
DISC_CATCH_PREP_ID1_TICK = 650
DISC_CATCH_PREP_ID2_TICK = 600
DISC_CATCH_READY_ID1_TICK = 550
DISC_CATCH_READY_ID2_TICK = 550
DISC_CATCH_ID6_TICK = 350
DISC_CATCH_CATCHER_READY_TICK = 1110
DISC_CATCH_SPLITTER_READY_TICK = 1300
DISC_CATCH_POST_HIGH_ID1_TICK = 650
DISC_CATCH_POST_HIGH_ID2_TICK = 600
DISC_CATCH_POST_HIGH_ID6_TICK = 350
# Formal task-three keeps its own fixed descent calibration. This is not a
# task-one ball pose and must not be reused by DISC_CATCH.
COLUMN_CATCH_DESCEND_ID1_TICK = 520
DISC_CATCH_TARGET_TIMEOUT_S = 5.0
DISC_CATCH_SPLITTER_FIELD_TICK = 1100
DISC_CATCH_SPLITTER_YELLOW_TICK = 1600
DISC_CATCH_SPLITTER_RESET_TICK = 1300
DISC_CATCH_YELLOW_COOLDOWN_S = 0.5
DISC_CATCH_OPEN_HOLD_MARGIN_S = 0.1
DISC_CATCH_CLOSE_CONFIRM_DELAY_S = 0.05
ARM_JOINT_SEQUENCE_DELAY_S = 0.2
ARM_TUNE_COMMAND_PATH = Path("/home/cat/ros2_ws/arm_tune_command.txt")
ARM_TUNE_RESULT_PATH = Path("/home/cat/ros2_ws/arm_tune_result.txt")
ARM_TUNE_POLL_INTERVAL_S = 0.10
ARM_TUNE_85KG_LIMITS = {
    1: ID1_SAFE_LIMITS,
    2: ID2_SAFE_LIMITS,
    6: ID6_SAFE_LIMITS,
}
ARM_TUNE_ZP_LIMITS = {
    4: (500, 2500),
    5: (500, 2500),
    7: (500, 2500),
}
COLUMN_CATCH_READY_ID1_TICK = 650
COLUMN_CATCH_READY_ID2_TICK = 500
COLUMN_CATCH_READY_ID6_TICK = 350
COLUMN_CATCH_SPLITTER_TICK = 1300
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


def direct_zp_move_packet(servo_id, position, time_ms):
    position = max(500, min(2500, int(position)))
    time_ms = max(0, min(9999, int(time_ms)))
    return f"#{servo_id:03d}P{position:04d}T{time_ms:04d}!".encode("ascii")


ZP_GROUP_NAME = "G0000"

# Fixed SG90 outputs on the ZL 24-channel board.  These channels are shared
# with the formal route but are not task-arm joints, so every route keeps them
# at the configured pulses instead of allowing an auxiliary request to drift.
AUX_ZP_HOLD_TARGETS = ((12, 600, 800), (23, 1000, 800))


def direct_zp_group_packet(targets):
    """Build the multi-servo frame used by the standalone ball app."""

    commands = "".join(
        direct_zp_move_packet(servo_id, position, time_ms).decode("ascii")
        for servo_id, position, time_ms in targets
    )
    return f"{{{ZP_GROUP_NAME}{commands}}}".encode("ascii")


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


def load_last_id4_target(default):
    try:
        state = json.loads((Path.home() / ".servo_zp_state.json").read_text())
        return max(500, min(2500, int(state.get("zp4_target", default))))
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
                points = np.asarray(detection[key], dtype=np.float32).copy()
                points[:, 0] *= scale_x
                points[:, 1] *= scale_y
                detection[key] = points.astype(np.int32)
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


def detection_process_worker(source_frame, detection_scale):
    global _PROCESS_DETECTOR
    if _PROCESS_DETECTOR is None:
        _PROCESS_DETECTOR = TargetDetector()
    detect_started = time.perf_counter()
    if detection_scale < 0.999:
        source_height, source_width = source_frame.shape[:2]
        detect_width = max(1, int(round(source_width * detection_scale)))
        detect_height = max(1, int(round(source_height * detection_scale)))
        detect_frame = cv2.resize(
            source_frame,
            (detect_width, detect_height),
            interpolation=cv2.INTER_AREA,
        )
        result = _PROCESS_DETECTOR.detect(detect_frame)
        result = scale_detections(
            result,
            source_width / float(detect_width),
            source_height / float(detect_height),
        )
    else:
        result = _PROCESS_DETECTOR.detect(source_frame)
    return result, time.perf_counter() - detect_started

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


class SerialTrigger:
    def __init__(
        self,
        device,
        baudrate,
        command,
        color,
        kind,
        stable_frames,
        reset_frames,
        cooldown_s,
        enabled=True,
    ):
        self.device = device
        self.baudrate = baudrate
        self.command = (command.strip() + "\n").encode("ascii", "ignore")
        self.color = color
        self.kind = kind
        self.stable_frames = max(1, stable_frames)
        self.reset_frames = max(1, reset_frames)
        self.cooldown_s = max(0.0, cooldown_s)
        self.enabled = enabled
        self.fd = None
        self.seen_frames = 0
        self.missing_frames = 0
        self.armed = True
        self.last_trigger = 0.0
        self.status = "servo trigger disabled" if not enabled else "servo trigger ready"

        if self.enabled:
            self._open()

    def _open(self):
        try:
            subprocess.run(
                [
                    "stty",
                    "-F",
                    self.device,
                    str(self.baudrate),
                    "cs8",
                    "-cstopb",
                    "-parenb",
                    "-ixon",
                    "-ixoff",
                    "-crtscts",
                    "raw",
                    "-echo",
                ],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self.status = f"servo trigger on {self.device}"
            print(self.status)
        except OSError as exc:
            self.enabled = False
            self.status = f"servo uart open failed: {exc}"
            print(self.status)
        except subprocess.CalledProcessError as exc:
            self.enabled = False
            self.status = f"servo uart setup failed: {exc}"
            print(self.status)

    def update(self, detections):
        if not self.enabled:
            return self.status

        matched = any(
            det.get("color") == self.color and det.get("kind") == self.kind
            for det in detections
        )
        if matched:
            self.seen_frames += 1
            self.missing_frames = 0
        else:
            self.seen_frames = 0
            self.missing_frames += 1
            if self.missing_frames >= self.reset_frames:
                self.armed = True
            return self.status

        now = time.monotonic()
        if not self.armed:
            return f"{self.status}, waiting target leave"
        if self.seen_frames < self.stable_frames:
            return f"{self.status}, confirming {self.color} {self.kind}"
        if (now - self.last_trigger) < self.cooldown_s:
            return f"{self.status}, cooldown"

        try:
            os.write(self.fd, self.command)
            self.last_trigger = now
            self.armed = False
            command_text = self.command.decode("ascii", "replace").strip()
            self.status = f"sent {command_text} for {self.color} {self.kind}"
            print(self.status)
        except OSError as exc:
            self.status = f"servo uart write failed: {exc}"
            print(self.status)
        return self.status

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class AbsoluteServoBridge:
    def __init__(self, device, baudrate, enabled=True, write_enabled=True):
        self.device = device
        self.baudrate = baudrate
        self.enabled = enabled
        self.write_enabled = enabled and write_enabled
        self.fd = None
        self.last_feedback = None
        self.last_command_ok = False
        self.last_ack = {}
        self.status = "servo bridge disabled"
        if self.enabled:
            self._open()

    def _open(self):
        try:
            subprocess.run(
                [
                    "stty",
                    "-F",
                    self.device,
                    str(self.baudrate),
                    "cs8",
                    "-cstopb",
                    "-parenb",
                    "-ixon",
                    "-ixoff",
                    "-crtscts",
                    "raw",
                    "-echo",
                ],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self.status = f"servo bridge on {self.device}"
            print(self.status)
        except (OSError, subprocess.CalledProcessError) as exc:
            self.enabled = False
            self.status = f"servo bridge open failed: {exc}"
            print(self.status)

    def _read_text(self, timeout_s):
        if self.fd is None:
            return ""
        deadline = time.monotonic() + timeout_s
        chunks = []
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select([self.fd], [], [], remaining)
            if not readable:
                break
            try:
                data = os.read(self.fd, 512)
            except BlockingIOError:
                continue
            if not data:
                break
            chunks.append(data)
            if b"\n" in data:
                time.sleep(0.01)
        return b"".join(chunks).decode("ascii", "replace")

    def send_targets(
        self,
        id1=None,
        id2=None,
        id4=None,
        id6=None,
        id5=None,
        splitter_id4=None,
    ):
        if not self.enabled or self.fd is None:
            self.last_command_ok = False
            return self.status
        if not self.write_enabled:
            return "servo bridge read-only; targets not sent"

        commands = []
        if id1 is not None and id2 is not None:
            value1 = int(id1)
            value2 = int(id2)
            commands.append(
                (f"armpair={value1},{value2}", f"OK ARMPAIR {value1} {value2}", 2, 0.25, 0.25, 1)
            )
        elif id1 is not None:
            value = int(id1)
            commands.append((f"id1={value}", f"OK ID1 TARGET {value}", 2, 0.25, 0.25, 1))
        elif id2 is not None:
            value = int(id2)
            commands.append((f"id2={value}", f"OK ID2 TARGET {value}", 2, 0.25, 0.25, 1))
        if id6 is not None:
            value = int(id6)
            commands.append((f"id6={value}", f"OK ID6 TARGET {value}", 2, 0.25, 0.25, 1))
        if splitter_id4 is not None:
            value = int(splitter_id4)
            commands.append((f"zp4:{value}", f"OK ZP4 {value}", 4, 0.45, 0.45, 2))
        if id4 is not None:
            value = int(id4)
            commands.append((f"zp4:{value}", f"OK ZP4 {value}", 4, 0.45, 0.45, 2))
        if id5 is not None:
            value = int(id5)
            commands.append((f"zp5:{value}", f"OK ZP5 {value}", 4, 0.45, 0.45, 2))
        if not commands:
            return self.status

        self.last_command_ok = False
        self.last_ack = {}
        try:
            self._read_text(0.02)
            for command, expected_ack, attempts, read_timeout, retry_delay, required_acks in commands:
                response = ""
                ack_count = 0
                for attempt in range(attempts):
                    os.write(self.fd, (command + "\n").encode("ascii", "ignore"))
                    response = self._read_text(read_timeout).strip()
                    normalized = response.replace("\r", " ").replace("\n", " ")
                    print(
                        f"SERVO UART TX={command} attempt={attempt + 1} "
                        f"RX={normalized or 'timeout'}",
                        flush=True,
                    )
                    if expected_ack in normalized and "ERR" not in normalized:
                        ack_count += 1
                        self.last_ack[command] = normalized
                        if ack_count >= required_acks:
                            break
                    if attempt < attempts - 1:
                        time.sleep(retry_delay)
                if ack_count < required_acks:
                    detail = response.replace("\r", " ").replace("\n", " ").strip()
                    self.status = f"ACK failed {command}: {detail or 'timeout'}"
                    return self.status
            self.last_command_ok = True
            self.status = "ACK " + ",".join(command for command, *_ in commands)
        except OSError as exc:
            self.status = f"servo bridge write failed: {exc}"
            print(self.status)
        return self.status

    def ready_for_commands(self):
        return (
            self.enabled
            and self.write_enabled
            and self.fd is not None
            and self.last_command_ok
        )

    def wait_ready(self, timeout_s=0.2):
        if not self.enabled or self.fd is None:
            return False
        text = self._read_text(timeout_s)
        if "ARM UART READY" in text:
            self.status = "ARM UART READY"
            return True
        return False

    def command_arm_ready(self, timeout_s=4.0):
        if not self.enabled or self.fd is None or not self.write_enabled:
            return None
        self.last_command_ok = False
        try:
            self._read_text(0.05)
            os.write(self.fd, b"armready\n")
            response = self._read_text(timeout_s)
        except OSError as exc:
            self.status = f"ARMREADY write failed: {exc}"
            return None
        normalized = response.replace("\r", " ").replace("\n", " ").strip()
        print(f"SERVO UART TX=armready RX={normalized or 'timeout'}", flush=True)
        match = ARM_READY_REPORT_RE.search(response)
        if match is None or "ERR ARMREADY" in response:
            self.status = f"ARMREADY failed: {normalized or 'timeout'}"
            return None
        self.last_feedback = (int(match.group(1)), int(match.group(2)))
        self.last_command_ok = True
        self.status = (
            f"ARMREADY complete ID1={self.last_feedback[0]} "
            f"ID2={self.last_feedback[1]}"
        )
        return self.last_feedback

    def query_positions(self, timeout_s=0.90):
        if not self.enabled or self.fd is None:
            return None
        try:
            self._read_text(0.02)
            os.write(self.fd, b"pos\n")
            response = self._read_text(timeout_s)
        except OSError as exc:
            self.status = f"servo feedback failed: {exc}"
            return None
        match = POSITION_REPORT_RE.search(response)
        if match is None:
            self.status = f"servo feedback invalid: {response.strip() or 'timeout'}"
            return None
        id6_text = match.group(3)
        id6 = None if id6_text in (None, "ERR") else int(id6_text)
        self.last_feedback = (int(match.group(1)), int(match.group(2)), id6)
        id6_status = "" if id6 is None else f" ID6={id6}"
        self.status = (
            f"feedback ID1={self.last_feedback[0]} ID2={self.last_feedback[1]}"
            f"{id6_status}"
        )
        return self.last_feedback

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.last_command_ok = False


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
                f"ID4={controller.splitter_id4} ID5={controller.id5} "
                f"ID6={controller.id6} ID7={controller.id4}"
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
            and getattr(controller.servo_bridge, "zp_fd", None) is not None
        )
        if not servo_ready:
            return self._write_result(f"ERR SERVO_NOT_READY {controller.servo_bridge.status}")
        if (len(parts) - 1) % 2 != 0:
            return self._write_result("ERR BAD_ARGS use pairs like: SET ID6 580 ID7 1120")

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
            limits = ARM_TUNE_85KG_LIMITS.get(servo_id) or ARM_TUNE_ZP_LIMITS.get(servo_id)
            if limits is None:
                return self._write_result("ERR UNSUPPORTED_ID allowed=ID1,ID2,ID4,ID5,ID6,ID7")
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
        if 4 in targets:
            send_kwargs["splitter_id4"] = targets[4]
        if 5 in targets:
            send_kwargs["id5"] = targets[5]
        if 7 in targets:
            send_kwargs["id4"] = targets[7]

        status = controller.servo_bridge.send_targets(**send_kwargs)
        if not controller.servo_bridge.last_command_ok:
            return self._write_result(f"ERR WRITE_FAILED {status}")

        if 1 in targets:
            controller.id1 = targets[1]
        if 2 in targets:
            controller.id2 = targets[2]
        if 6 in targets:
            controller.id6 = targets[6]
        if 4 in targets:
            controller.splitter_id4 = targets[4]
        if 5 in targets:
            controller.id5 = targets[5]
        if 7 in targets:
            controller.id4 = targets[7]
        controller.last_command_time = time.monotonic()
        controller.arm_preview.set_targets(controller.id1, controller.id2, controller.id4, controller.id6)
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


class DirectBusServoBridge:
    def __init__(
        self,
        device,
        baudrate,
        enabled=True,
        write_enabled=True,
        arm_device=None,
        zp_device=None,
        arm_time_ms=600,
        zp_time_ms=300,
        gripper_time_ms=None,
        splitter_time_ms=None,
        repeat=1,
    ):
        self.device = device
        self.arm_device = arm_device or device
        self.zp_device = zp_device or device
        self.baudrate = baudrate
        self.enabled = enabled
        self.write_enabled = enabled and write_enabled
        self.arm_time_ms = int(arm_time_ms)
        self.zp_time_ms = int(zp_time_ms)
        self.gripper_time_ms = (
            int(gripper_time_ms)
            if gripper_time_ms is not None
            else self.zp_time_ms
        )
        self.splitter_time_ms = (
            int(splitter_time_ms)
            if splitter_time_ms is not None
            else 500
        )
        self.repeat = max(1, min(8, int(repeat)))
        self.arm_fd = None
        self.zp_fd = None
        self.last_feedback = None
        self.last_command_ok = False
        self.last_ack = {}
        self.assumed_feedback = True
        self._arm_io_lock = threading.RLock()
        self._zp_io_lock = threading.RLock()
        self.status = "dual servo-board bridge disabled"
        if self.enabled:
            self._open()

    def _open_device(self, device):
        subprocess.run(
            [
                "stty",
                "-F",
                device,
                str(self.baudrate),
                "cs8",
                "-cstopb",
                "-parenb",
                "-ixon",
                "-ixoff",
                "-crtscts",
                "raw",
                "-echo",
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
        try:
            self.arm_fd = self._open_device(self.arm_device)
            if self.zp_device == self.arm_device:
                self.zp_fd = self.arm_fd
            else:
                self.zp_fd = self._open_device(self.zp_device)
            self.status = (
                f"dual servo boards htd85-binary={self.arm_device} "
                f"zp20s-ascii={self.zp_device} "
                f"zp_time={self.zp_time_ms}ms splitter_time={self.splitter_time_ms}ms "
                "exclusive=yes"
            )
            print(self.status)
            if self.write_enabled:
                self._send_aux_hold_targets()
        except (OSError, subprocess.CalledProcessError) as exc:
            self.close()
            self.enabled = False
            self.write_enabled = False
            self.status = f"dual servo-board open failed: {exc}"
            print(self.status)

    def _write_payload(self, fd, payload, repeat=None):
        repeat_count = self.repeat if repeat is None else max(1, min(8, int(repeat)))
        for attempt in range(repeat_count):
            offset = 0
            deadline = time.monotonic() + 0.2
            while offset < len(payload):
                try:
                    written = os.write(fd, payload[offset:])
                    if written <= 0:
                        raise OSError("serial write returned zero bytes")
                    offset += written
                except BlockingIOError:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        raise TimeoutError("serial write timed out")
                    _, writable, _ = select.select(
                        [], [fd], [], min(0.02, remaining)
                    )
                    if not writable:
                        continue
            if attempt < repeat_count - 1:
                time.sleep(0.08)

    def _format_payload(self, payload):
        try:
            text = payload.decode("ascii")
            if text.startswith("#"):
                return text
        except UnicodeDecodeError:
            pass
        return " ".join(f"{byte:02X}" for byte in payload)

    def _send_zp_targets(self, zp_targets, repeat=None):
        """Write ZP20S targets as one group frame when more than one is present."""

        if not zp_targets:
            return True, ""
        payload = (
            direct_zp_group_packet(zp_targets)
            if len(zp_targets) > 1
            else direct_zp_move_packet(*zp_targets[0])
        )
        try:
            with self._zp_io_lock:
                self._write_payload(self.zp_fd, payload, repeat=repeat)
        except (OSError, TimeoutError) as exc:
            warning = f"ZP20S UART write failed: {exc}"
            print(warning, flush=True)
            return False, warning
        label = "group" if len(zp_targets) > 1 else "single"
        print(
            f"DIRECT SERVO TX zp20s-ascii={self.zp_device} mode={label} "
            f"{self._format_payload(payload)} "
            f"physical_ids={','.join(f'ID{sid}' for sid, _, _ in zp_targets)}",
            flush=True,
        )
        return True, (
            "ZP20S TX "
            + ",".join(f"ID{sid}={value}" for sid, value, _ in zp_targets)
            + f" mode={label}"
        )

    def _send_aux_hold_targets(self):
        """Set the fixed S12/S23 outputs after opening the ZP bus."""
        if not self.write_enabled or self.zp_fd is None:
            return False
        try:
            with self._zp_io_lock:
                for channel, pulse, time_ms in AUX_ZP_HOLD_TARGETS:
                    payload = direct_zp_move_packet(channel, pulse, time_ms)
                    self._write_payload(self.zp_fd, payload, repeat=1)
                    print(
                        f"DIRECT SERVO TX zp20s-ascii={self.zp_device} "
                        f"AUX_HOLD S{channel}={pulse} T={time_ms}ms "
                        f"{self._format_payload(payload)}",
                        flush=True,
                    )
                    time.sleep(0.003)
            return True
        except (OSError, TimeoutError) as exc:
            print(f"ZP20S AUX_HOLD write failed: {exc}", flush=True)
            return False

    def send_zp_aux(self, channel=None, servo_id=None, pulse=1500, time_ms=300, repeat=None):
        """Write a formal-route auxiliary target on the ZL ZP20S bus."""
        target = channel if channel is not None else servo_id
        if channel is not None:
            fixed = {12: 600, 23: 1000}.get(int(channel))
            if fixed is not None:
                if int(pulse) != fixed:
                    print(
                        f"ZP20S AUX_HOLD override S{int(channel)} "
                        f"requested={int(pulse)} fixed={fixed}",
                        flush=True,
                    )
                pulse = fixed
        if (
            not self.enabled
            or self.zp_fd is None
            or target is None
            or not 1 <= int(target) <= 255
            or not 500 <= int(pulse) <= 2500
            or not 0 <= int(time_ms) <= 9999
        ):
            self.last_command_ok = False
            return self.status
        if not self.write_enabled:
            self.last_command_ok = False
            self.status = "direct bus servo read-only; auxiliary target not sent"
            return self.status
        self.last_command_ok = False
        ok, status = self._send_zp_targets(
            [(int(target), int(pulse), int(time_ms))], repeat=repeat
        )
        self.last_command_ok = bool(ok)
        if ok:
            label = f"S{int(channel)}" if channel is not None else f"ID{int(servo_id)}"
            self.status = f"ZP20S AUX TX {label}={int(pulse)} T={int(time_ms)}ms"
        else:
            self.status = status
        return self.status

    def send_htd85_aux(self, servo_id=3, pulse=0, time_ms=300, repeat=None):
        """Write one auxiliary target on the Hiwonder HTD85 binary bus."""
        servo_id = int(servo_id)
        pulse = int(pulse)
        time_ms = int(time_ms)
        if (
            not self.enabled
            or self.arm_fd is None
            or servo_id != 3
            or not 0 <= pulse <= HTD85_MAX_POSITION
            or not 0 <= time_ms <= 30000
        ):
            self.last_command_ok = False
            return self.status
        if not self.write_enabled:
            self.last_command_ok = False
            self.status = "direct bus servo read-only; HTD85 auxiliary target not sent"
            return self.status
        self.last_command_ok = False
        ok, status = self._send_htd85_targets(
            [(servo_id, pulse)], repeat=repeat, time_ms=time_ms
        )
        self.last_command_ok = bool(ok)
        self.status = status if not ok else (
            f"HTD85 AUX TX ID{servo_id}={pulse} T={time_ms}ms"
        )
        return self.status

    def _send_htd85_targets(self, arm_targets, repeat=None, time_ms=None):
        """Write 85KG targets through the Hiwonder USB servo board."""
        if not arm_targets:
            return True, ""
        try:
            with self._arm_io_lock:
                sent = []
                for servo_id, value in arm_targets:
                    value = max(HTD85_MIN_POSITION, min(HTD85_MAX_POSITION, int(value)))
                    payload = htd85_move_packet(
                        servo_id,
                        value,
                        self.arm_time_ms if time_ms is None else int(time_ms),
                    )
                    self._write_payload(self.arm_fd, payload, repeat=repeat)
                    sent.append(f"ID{servo_id}={value}")
                    print(
                        f"DIRECT SERVO TX htd85-binary={self.arm_device} "
                        f"{self._format_payload(payload)} "
                        f"physical_id={servo_id} target={value} "
                        f"time={self.arm_time_ms if time_ms is None else int(time_ms)}ms",
                        flush=True,
                    )
                    # Keep adjacent joint commands separate without waiting for
                    # a servo reply or delaying the chassis/RK handoff.
                    time.sleep(0.003)
        except (OSError, TimeoutError) as exc:
            warning = f"HTD85 UART write failed: {exc}"
            print(warning, flush=True)
            return False, warning
        return True, "HTD85 TX " + ",".join(sent) + " feedback=OPTIONAL"

    def _read_htd85_position(self, servo_id, timeout_s=0.25):
        if self.arm_fd is None:
            return None
        with self._arm_io_lock:
            try:
                if termios is not None:
                    termios.tcflush(self.arm_fd, termios.TCIFLUSH)
                payload = htd85_position_read_packet(servo_id)
                print(
                    f"DIRECT SERVO TX htd85-binary={self.arm_device} "
                    f"{self._format_payload(payload)} read=ID{servo_id}",
                    flush=True,
                )
                position = htd85_read_position(
                    self.arm_fd,
                    servo_id,
                    timeout_s=timeout_s,
                )
                if position is not None:
                    print(
                        f"DIRECT SERVO RX htd85-binary={self.arm_device} "
                        f"ID{servo_id} position={position}",
                        flush=True,
                    )
                return position
            except (OSError, TimeoutError) as exc:
                print(f"HTD85 position read failed ID{servo_id}: {exc}", flush=True)
                return None

    def send_targets(
        self,
        id1=None,
        id2=None,
        id3=None,
        id4=None,
        id6=None,
        id5=None,
        splitter_id4=None,
        repeat=None,
    ):
        if not self.enabled or self.arm_fd is None or self.zp_fd is None:
            self.last_command_ok = False
            return self.status
        if not self.write_enabled:
            return "direct bus servo read-only; targets not sent"

        zp_targets = []
        targets = {}
        arm_targets = []
        if id1 is not None:
            arm_targets.append((1, int(id1)))
        if id2 is not None:
            arm_targets.append((2, int(id2)))
        if id3 is not None:
            arm_targets.append((3, int(id3)))
        if id6 is not None:
            arm_targets.append((6, int(id6)))

        for servo_id, value in arm_targets:
            targets[servo_id] = value
        if splitter_id4 is not None:
            zp_targets.append((4, int(splitter_id4), self.splitter_time_ms))
            targets[4] = int(splitter_id4)
        if id4 is not None:
            # Legacy keyword id4 is retained for callers, but means the
            # physical ZP20S gripper ID7 in this bridge.
            zp_targets.append((7, int(id4), self.gripper_time_ms))
            targets[7] = int(id4)
        if id5 is not None:
            zp_targets.append((5, int(id5), self.zp_time_ms))
            targets[5] = int(id5)
        if not zp_targets and not arm_targets:
            return self.status

        self.last_command_ok = False
        try:
            arm_ok, arm_status = self._send_htd85_targets(arm_targets, repeat=repeat)
            if not arm_ok:
                self.status = arm_status
                print(self.status, flush=True)
                return self.status
            if arm_status:
                print(f"DIRECT SERVO TX htd85={arm_status}", flush=True)
            zp_ok, zp_status = self._send_zp_targets(zp_targets, repeat=repeat)
            if not zp_ok:
                self.status = zp_status
                print(self.status, flush=True)
                return self.status
            previous = self.last_feedback or (READY_ID1_TICK, READY_ID2_TICK, None)
            self.last_feedback = (
                targets.get(1, previous[0]),
                targets.get(2, previous[1]),
                targets.get(6, previous[2] if len(previous) > 2 else None),
            )
            self.last_command_ok = True
            self.status = (
                "dual-board TX htd85="
                + ",".join(f"ID{k}={v}" for k, v in targets.items() if k in {1, 2, 3, 6})
                + " zp20s="
                + ",".join(f"ID{k}={v}" for k, v in targets.items() if k in {4, 5, 7})
            )
        except (OSError, TimeoutError) as exc:
            self.status = f"dual servo-board write failed: {exc}"
            print(self.status)
        return self.status

    def ready_for_commands(self):
        return (
            self.enabled
            and self.write_enabled
            and self.arm_fd is not None
            and self.zp_fd is not None
            and self.last_command_ok
        )

    def wait_ready(self, timeout_s=0.2):
        if not self.enabled or self.arm_fd is None or self.zp_fd is None:
            return False
        feedback = self.query_positions(timeout_s=max(0.12, float(timeout_s)))
        if feedback is None:
            self.status = f"Hiwonder HTD85 board probe timeout on {self.arm_device}"
            return False
        self.status = (
            f"Hiwonder HTD85 board ready on {self.arm_device} "
            f"ID1={feedback[0]} ID2={feedback[1]}"
        )
        return True

    def command_arm_ready(self, timeout_s=4.0):
        if not self.enabled or self.arm_fd is None or not self.write_enabled:
            return None
        feedback = self.query_positions(timeout_s=min(0.8, max(0.2, float(timeout_s))))
        if feedback is None:
            self.last_command_ok = False
            self.status = f"Hiwonder HTD85 position readback failed on {self.arm_device}"
            return None
        self.last_feedback = feedback
        self.last_command_ok = True
        self.status = (
            f"HTD85 ARMREADY readback ID1={self.last_feedback[0]} "
            f"ID2={self.last_feedback[1]}"
        )
        return self.last_feedback

    def query_positions(self, timeout_s=0.90):
        if not self.enabled or self.arm_fd is None or self.zp_fd is None:
            return None
        positions = {}
        per_servo_timeout = max(0.08, min(0.30, float(timeout_s) / 3.0))
        for servo_id in (1, 2, 6):
            position = self._read_htd85_position(servo_id, per_servo_timeout)
            if position is not None:
                positions[servo_id] = int(position)
        if 1 in positions and 2 in positions:
            self.last_feedback = (
                positions[1],
                positions[2],
                positions.get(6),
            )
            self.last_command_ok = True
            self.status = (
                f"HTD85 feedback ID1={self.last_feedback[0]} "
                f"ID2={self.last_feedback[1]}"
                + (
                    ""
                    if self.last_feedback[2] is None
                    else f" ID6={self.last_feedback[2]}"
                )
            )
            return self.last_feedback
        self.status = (
            f"HTD85 feedback incomplete on {self.arm_device}: "
            + ",".join(f"ID{sid}={positions.get(sid, 'timeout')}" for sid in (1, 2, 6))
        )
        return None

    def close(self):
        if self.zp_fd is not None and self.zp_fd != self.arm_fd:
            try:
                os.close(self.zp_fd)
            except OSError:
                pass
        self.zp_fd = None
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
        self.splitter_id4 = SPLITTER_YELLOW_TICK
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
        self.disc_pulse_done = False
        self.disc_last_pulsed_color = None
        self.disc_prep_high_active = False
        self.column_target_armed = True
        self.column_target_absent_frames = 0
        self.platform_selected_letters = frozenset()
        self.platform_preselect_letters = set()
        self.platform_preselect_count = 2
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
        return max(0.15, getattr(self.servo_bridge, "arm_time_ms", 700) / 1000.0 + 0.12)

    def _zp_settle_s(self):
        return max(0.12, getattr(self.servo_bridge, "zp_time_ms", 450) / 1000.0 + 0.08)

    def _gripper_settle_s(self):
        return max(
            0.12,
            getattr(self.servo_bridge, "gripper_time_ms", 450) / 1000.0 + 0.05,
        )

    def _gripper_motion_s(self):
        return max(
            0.10,
            getattr(self.servo_bridge, "gripper_time_ms", 450) / 1000.0,
        )

    def _disc_low_pose_settle_s(self):
        return max(
            0.15,
            max(
                getattr(self.servo_bridge, "arm_time_ms", 700),
                getattr(self.servo_bridge, "zp_time_ms", 450),
                getattr(self.servo_bridge, "gripper_time_ms", 450),
            )
            / 1000.0
            + 0.15,
        )

    def _disc_open_hold_s(self):
        return max(
            self._gripper_settle_s(),
            getattr(self.servo_bridge, "gripper_time_ms", 450) / 1000.0
            + DISC_CATCH_OPEN_HOLD_MARGIN_S,
        )

    def _splitter_settle_s(self):
        return max(
            0.12,
            getattr(self.servo_bridge, "splitter_time_ms", 900) / 1000.0 + 0.08,
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

        target_id6 = self._clamp(self.id6 + delta_id6, ID6_SAFE_LIMITS)
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

        self.id7 = target_id7
        self.id5 = target_id5
        self.splitter_id4 = target_splitter
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(f"{reason} phase 1 {first_label}")
        print(
            f"ARM STAGED reason={reason} phase=1 joint={first_label} "
            f"delay_ms={int(ARM_JOINT_SEQUENCE_DELAY_S * 1000)}",
            flush=True,
        )
        first_status = self.servo_bridge.send_targets(**first_targets)
        self.last_command_time = time.monotonic()
        if not self.servo_bridge.last_command_ok:
            self.status = f"{reason} phase 1 {first_label} failed: {first_status}"
            return self.status

        time.sleep(ARM_JOINT_SEQUENCE_DELAY_S)
        self.id1 = target_id1
        self.id2 = target_id2
        self.id6 = target_id6
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(f"{reason} phase 2 {second_label}")
        print(
            f"ARM STAGED reason={reason} phase=2 joint={second_label} "
            f"after_ms={int(ARM_JOINT_SEQUENCE_DELAY_S * 1000)}",
            flush=True,
        )
        second_status = self.servo_bridge.send_targets(**second_targets)
        self.last_command_time = time.monotonic()
        if not self.servo_bridge.last_command_ok:
            self.status = f"{reason} phase 2 {second_label} failed: {second_status}"
            return self.status

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

    # The UART protocol still names the claw channel id4, but this helper
    # makes the task-one ZP20S gripper channel explicit.
    def _send_gripper_id7(self, target, reason):
        previous_id7 = self.id7
        target = int(target)
        self.arm_preview.set_targets(self.id1, self.id2, target, self.id6)
        self.arm_preview.publish(reason)
        print(
            f"GRASP COMMAND reason={reason} ID1={self.id1} "
            f"ID2={self.id2} ID6={self.id6} ID7={previous_id7}->{target}",
            flush=True,
        )
        self.status = self.servo_bridge.send_targets(id4=target)
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

    def _send_splitter_id4(self, target, reason):
        previous = self.splitter_id4
        target = int(target)
        print(
            f"AUX COMMAND reason={reason} ID4={previous}->{target}",
            flush=True,
        )
        self.status = self.servo_bridge.send_targets(splitter_id4=target)
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

    def _send_disc_open_phase(self, splitter_target, reason):
        previous_splitter = self.splitter_id4
        previous_id7 = self.id7
        splitter_target = int(splitter_target)
        self.arm_preview.set_targets(self.id1, self.id2, self.id7_open, self.id6)
        self.arm_preview.publish(reason)
        print(
            f"DISC OPEN reason={reason} ID1={self.id1} ID2={self.id2} ID6={self.id6} "
            f"ID4={previous_splitter}->{splitter_target} ID7={previous_id7}->{self.id7_open}",
            flush=True,
        )
        self.status = self.servo_bridge.send_targets(
            splitter_id4=splitter_target,
            id4=self.id7_open,
        )
        self.last_command_time = time.monotonic()
        if self.servo_bridge.write_enabled and not self.servo_bridge.last_command_ok:
            self.splitter_id4 = previous_splitter
            self.id7 = previous_id7
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.feedback_pending = False
            self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
            self.status = f"automatic motion stopped: {self.servo_bridge.status}"
            self.arm_preview.publish(self.status)
            return self.status
        self.splitter_id4 = splitter_target
        self.id7 = self.id7_open
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        return (
            f"{reason}: splitter_id4={self.splitter_id4} id7={self.id7} | {self.status}"
        )

    def update_auxiliary(self, detections):
        if not self.enabled or not self.servo_bridge.write_enabled:
            return ""
        now = time.monotonic()
        if now - self.last_aux_command_time < self.aux_command_interval_s:
            return ""
        balls = [det for det in detections if det.get("kind") == "ball"]
        if any(det.get("color") == "yellow" for det in balls):
            target = SPLITTER_YELLOW_TICK
            reason = "yellow ball detected; splitter retract"
        elif balls:
            target = SPLITTER_OTHER_BALL_TICK
            reason = "non-yellow ball detected; splitter extend"
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
        )

    def begin_chassis_station(self, station):
        self._reset_cycle_for_search(f"chassis station {station} start")
        if self.startup_stage != "complete":
            return f"chassis station {station} queued until startup completes"
        self.active_chassis_station = station
        self.chassis_station_done_reason = None
        self.chassis_station_error_reason = None
        if station == "DISC_CATCH":
            return self._begin_disc_catch_station()
        if station == "COLUMN_CATCH":
            return self._begin_column_catch_station()
        if station == "PLATFORM_PICK":
            # The preselect-to-first-slot gap is held high while H7 performs
            # its lateral shift and white-line alignment.  A slot START is
            # the explicit boundary that permits normal target handling.
            self.chassis_station_stage = None
            self.id1 = 600
            self.id2 = 600
            self.id6 = 350
            self.id5 = CATCHER_HOME_TICK
            self.splitter_id4 = DISC_CATCH_SPLITTER_READY_TICK
        else:
            self.id1 = READY_ID1_TICK
            self.id2 = READY_ID2_TICK
            self.id6 = BASE_YAW_CENTER_TICK
            self.id5 = CATCHER_HOME_TICK
            self.splitter_id4 = SPLITTER_YELLOW_TICK
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

    def begin_platform_preselect(self, count=2):
        self._reset_cycle_for_search("platform preselect start")
        self.active_chassis_station = "PLATFORM_PICK"
        self.chassis_station_done_reason = None
        self.chassis_station_error_reason = None
        self.chassis_station_stage = "platform_preselect"
        self.platform_preselect_count = 2
        self.platform_preselect_letters = set()
        self.id1 = 600
        self.id2 = 600
        self.id6 = 350
        self.id7 = self.id7_closed
        self.id5 = CATCHER_HOME_TICK
        self.splitter_id4 = DISC_CATCH_SPLITTER_READY_TICK
        self._enforce_angle_gap()
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(
            "PLATFORM_PICK preselect; secondary camera; collecting 2 distinct letters"
        )
        if not self.servo_bridge.write_enabled:
            self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
            return (
                "preview PLATFORM_PICK preselect "
                f"ID1={self.id1} ID2={self.id2} ID6={self.id6}"
            )
        status = self._send_fixed_arm_pose_staged(
            self.id1,
            self.id2,
            self.id6,
            "PLATFORM_PICK preselect high",
            raising=True,
            id7=self.id7,
            id5=self.id5,
            splitter_id4=self.splitter_id4,
        )
        if not self.servo_bridge.last_command_ok:
            self.chassis_station_stage = None
            self.state = "fault"
            self.algorithm_stage = "fault"
            self.status = f"PLATFORM_PICK preselect arm pose failed: {status}"
            return self.status
        self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
        self.status = (
            f"PLATFORM_PICK preselect ID1={self.id1} ID2={self.id2} "
            f"ID6={self.id6}; waiting secondary letters | {status}"
        )
        print(f"CHASSIS STATION {self.status}", flush=True)
        return self.status

    def update_platform_preselect(self, detections, detection_fresh=True):
        if self.chassis_station_stage != "platform_preselect":
            return None
        now = time.monotonic()
        if now < self.chassis_station_deadline:
            return (
                "PLATFORM_PICK preselect settling "
                f"{self.chassis_station_deadline - now:.1f}s"
            )
        if detection_fresh:
            candidates = [
                det for det in detections
                if det.get("kind") == "letter"
                and det.get("letter") in LETTERS
            ]
            candidates.sort(
                key=lambda det: (
                    float(det.get("confidence") or 0.0),
                    float(det.get("projected_area") or 0.0),
                ),
                reverse=True,
            )
            for det in candidates:
                letter = str(det.get("letter")).upper()
                if letter not in self.platform_preselect_letters:
                    self.platform_preselect_letters.add(letter)
                    print(
                        "PLATFORM PRESELECT secondary letter="
                        f"{letter} count={len(self.platform_preselect_letters)}/2",
                        flush=True,
                    )
                    if len(self.platform_preselect_letters) >= self.platform_preselect_count:
                        break
        if len(self.platform_preselect_letters) >= self.platform_preselect_count:
            selected = tuple(sorted(self.platform_preselect_letters))[:2]
            self.platform_selected_letters = frozenset(selected)
            self.target_letters = self.platform_selected_letters
            # Keep the platform arm in its expanded high pose until H7 sends
            # the first slot START after its white-line entry is complete.
            # Clearing the station here lets the generic no-target path slowly
            # retract the arm during that entry window.
            self.chassis_station_stage = "platform_entry_hold"
            self.chassis_station_done_reason = (
                f"PRESELECT_DONE:{selected[0]}:{selected[1]}"
            )
            self.arm_preview.publish(
                f"PLATFORM_PICK letters locked {selected[0]},{selected[1]}"
            )
            return (
                "PLATFORM_PICK preselect done "
                f"letters={selected[0]},{selected[1]}"
            )
        return (
            "PLATFORM_PICK preselect waiting secondary letters "
            f"{','.join(sorted(self.platform_preselect_letters)) or '-'}"
        )

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
        self.id5 = CATCHER_HOME_TICK
        self.splitter_id4 = DISC_CATCH_SPLITTER_READY_TICK
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
        self.id7 = self.id7_closed
        self.id5 = CATCHER_HOME_TICK
        self.splitter_id4 = COLUMN_CATCH_SPLITTER_TICK
        self.chassis_station_stage = "column_ready"
        self.column_target_armed = True
        self.column_target_absent_frames = 0
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
        status = self._send_fixed_arm_pose_staged(
            COLUMN_CATCH_READY_ID1_TICK,
            COLUMN_CATCH_READY_ID2_TICK,
            COLUMN_CATCH_READY_ID6_TICK,
            "COLUMN_CATCH ready",
            raising=True,
            id7=self.id7,
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
        for det in detections:
            if det.get("kind") != "ball":
                continue
            color = det.get("color")
            if color in allowed_colors:
                allowed.append(det)
            elif color in {"red", "blue", "yellow"}:
                rejected.append(color)
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

    def _finish_chassis_station_after_retract(self, reason):
        # Every completed station must leave the arm fully retracted. In
        # particular, task-one completion must not leave ID5 at the catcher
        # ready position while H7 transfers to task two.
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
        self.disc_prep_high_active = False
        self.column_target_armed = True
        self.column_target_absent_frames = 0
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
        self.id1 = 600
        self.id2 = 600
        self.id6 = 350
        self.id7 = self.id7_closed
        self.chassis_station_stage = None
        self.active_chassis_station = None
        self.chassis_station_done_reason = f"SKIPPED:{reason}"
        self.algorithm_stage = "centering"
        self.state = "platform slot skipped"
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(
            f"PLATFORM_PICK skipped; keep expanded reason={reason}"
        )
        return True

    def _finish_platform_pick(self, reason):
        self.id1 = 600
        self.id2 = 600
        self.id6 = 350
        self.id7 = self.id7_closed
        self.id5 = CATCHER_HOME_TICK
        self.splitter_id4 = DISC_CATCH_SPLITTER_READY_TICK
        self._enforce_angle_gap()
        self.chassis_station_stage = None
        self.active_chassis_station = None
        self.chassis_station_done_reason = f"{reason}"
        self.algorithm_stage = "centering"
        self.state = "platform slot complete"
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish(
            f"PLATFORM_PICK {reason}; keep expanded for next slot"
        )
        return f"PLATFORM_PICK {reason}; ID1=600 ID2=600 ID6=350 ID7=1300"

    def stop_chassis_station(self, station):
        if self.active_chassis_station != station:
            return f"chassis station {station} stop ignored; active={self.active_chassis_station}"
        return self._finish_chassis_station_after_retract("STOPPED_BY_CHASSIS")

    def abort_chassis_station(self, reason):
        """Best-effort home before reporting a station control failure to H7."""

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
        self, station, detections, frame_shape, detection_fresh=True
    ):
        if station == "COLUMN_CATCH" and self.chassis_station_stage is not None:
            return self._update_column_catch_station(
                detections, detection_fresh=detection_fresh
            )
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
                self.disc_pulse_done = False
                self.disc_last_pulsed_color = None
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
            if self.disc_pulse_done and ball_color == self.disc_last_pulsed_color:
                return (
                    f"DISC_CATCH already pulsed {ball_color} ball; waiting new target"
                )
            splitter_target = (
                DISC_CATCH_SPLITTER_YELLOW_TICK
                if ball_color == "yellow"
                else DISC_CATCH_SPLITTER_FIELD_TICK
            )
            self.status = (
                f"DISC_CATCH {ball_color} ball detected at ID1={self.id1} "
                f"ID2={self.id2}; sync ID4 with ID7 open"
            )
            print(f"CHASSIS STATION {self.status}", flush=True)
            if not self.servo_bridge.write_enabled:
                self.disc_last_pulsed_color = ball_color
                self.disc_pulse_done = True
                self.splitter_id4 = splitter_target
                self.id7 = self.id7_open
                self.chassis_station_stage = "disc_open_wait"
                self.chassis_station_deadline = now + self._disc_open_hold_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return (
                    f"preview DISC_CATCH sync ID4={self.splitter_id4} "
                    "with ID7 open pulse"
                )
            trigger_status = self._send_disc_open_phase(
                splitter_target,
                "DISC_CATCH synchronized ID4/ID7 open phase",
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
                self.id7 = self.id7_open
                self.chassis_station_stage = "disc_open_wait"
                self.chassis_station_deadline = now + self._disc_open_hold_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview DISC_CATCH open ID7 pulse"
            status = self._send_gripper_id7(self.id7_open, "DISC_CATCH open ID7 pulse")
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
            status = self._send_gripper_id7(self.id7_closed, "DISC_CATCH close ID7 pulse")
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
                self.chassis_station_deadline = now + self._gripper_motion_s()
                return "preview DISC_CATCH close ID7 confirm"
            status = self._send_gripper_id7(
                self.id7_closed,
                "DISC_CATCH close ID7 confirm",
            )
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "disc_close_wait"
                self.chassis_station_deadline = (
                    time.monotonic() + self._gripper_motion_s()
                )
            return status

        if self.chassis_station_stage == "disc_close_wait":
            self.state = "DISC_CATCH close wait"
            if now < self.chassis_station_deadline:
                return f"DISC_CATCH close wait {self.chassis_station_deadline - now:.1f}s"
            if self.disc_last_pulsed_color == "yellow":
                self.chassis_station_stage = "disc_yellow_reset"
                return "DISC_CATCH yellow pulse closed; resetting splitter"
            self.disc_pulse_done = False
            self.disc_last_pulsed_color = None
            self.chassis_station_stage = "disc_detect"
            self.chassis_station_deadline = 0.0
            self.chassis_station_no_target_deadline = (
                time.monotonic() + DISC_CATCH_TARGET_TIMEOUT_S
            )
            return "DISC_CATCH pulse complete; ready for next target frame"

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
            self.chassis_station_stage = "disc_detect"
            self.chassis_station_deadline = 0.0
            self.chassis_station_no_target_deadline = (
                time.monotonic() + DISC_CATCH_TARGET_TIMEOUT_S
            )
            return "DISC_CATCH pulse complete; ready for next target frame"

        return "DISC_CATCH station idle"

    def _update_column_catch_station(self, detections, detection_fresh=True):
        now = time.monotonic()
        letter_target = (
            self._column_letter_visible(detections) if detection_fresh else None
        )

        if self.splitter_id4 != COLUMN_CATCH_SPLITTER_TICK:
            if not self.servo_bridge.write_enabled:
                self.splitter_id4 = COLUMN_CATCH_SPLITTER_TICK
            else:
                return self._send_splitter_id4(
                    COLUMN_CATCH_SPLITTER_TICK,
                    "COLUMN_CATCH hold splitter",
                )

        if self.chassis_station_stage == "column_ready":
            self.state = "COLUMN_CATCH ready"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH ready settling {self.chassis_station_deadline - now:.1f}s"
            self.id1 = COLUMN_CATCH_DESCEND_ID1_TICK
            self.id2 = COLUMN_CATCH_READY_ID2_TICK
            self.id6 = BASE_YAW_CENTER_TICK
            self._enforce_angle_gap()
            if not self.servo_bridge.write_enabled:
                self.chassis_station_stage = "column_descend_wait"
                self.chassis_station_deadline = now + self._arm_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return (
                    f"preview COLUMN_CATCH descend ID1={self.id1} "
                    f"ID2={self.id2}"
                )
            status = self.servo_bridge.send_targets(
                id1=self.id1,
                id2=self.id2,
                id6=self.id6,
            )
            self.last_command_time = now
            if not self.servo_bridge.last_command_ok:
                self.chassis_station_stage = None
                self.state = "fault"
                self.algorithm_stage = "fault"
                self.status = f"COLUMN_CATCH descend failed: {status}"
                self.arm_preview.publish(self.status)
                return self.status
            self.chassis_station_stage = "column_descend_wait"
            self.chassis_station_deadline = time.monotonic() + self._arm_settle_s()
            self.status = (
                f"COLUMN_CATCH descend ID1={self.id1} ID2={self.id2} "
                f"ID6={self.id6} | {status}"
            )
            print(f"CHASSIS STATION COLUMN_CATCH DESCEND {self.status}", flush=True)
            return self.status

        if self.chassis_station_stage == "column_descend_wait":
            self.state = "COLUMN_CATCH descend"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH descend settling {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_detect"
            return (
                "COLUMN_CATCH detecting selected A/B/C/D letter during orbit"
            )

        if self.chassis_station_stage == "column_detect":
            self.state = (
                "COLUMN_CATCH detect selected letter"
            )
            if not detection_fresh:
                return "COLUMN_CATCH waiting for fresh detection"
            if letter_target is None:
                self.column_target_absent_frames += 1
                if self.column_target_absent_frames >= 3:
                    self.column_target_armed = True
                return (
                    "COLUMN_CATCH orbit detect; no selected letter"
                )
            self.column_target_absent_frames = 0
            if not self.column_target_armed:
                return "COLUMN_CATCH waiting detected letter to leave before rearm"
            self.column_target_armed = False
            self.chassis_station_stage = "column_open"
            self.status = (
                f"COLUMN_CATCH letter {letter_target.get('letter')} detected; pulse ID7"
            )
            print(f"CHASSIS STATION {self.status}", flush=True)

        if self.chassis_station_stage == "column_open":
            self.state = "COLUMN_CATCH open claw"
            if not self.servo_bridge.write_enabled:
                self.id7 = self.id7_open
                self.chassis_station_stage = "column_open_wait"
                self.chassis_station_deadline = now + self._zp_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview COLUMN_CATCH open ID7 pulse"
            status = self._send_gripper_id7(self.id7_open, "COLUMN_CATCH open ID7 pulse")
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_open_wait"
                self.chassis_station_deadline = time.monotonic() + self._zp_settle_s()
            return status

        if self.chassis_station_stage == "column_open_wait":
            self.state = "COLUMN_CATCH open wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH open wait {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_close"

        if self.chassis_station_stage == "column_close":
            self.state = "COLUMN_CATCH close claw"
            if not self.servo_bridge.write_enabled:
                self.id7 = self.id7_closed
                self.chassis_station_stage = "column_close_wait"
                self.chassis_station_deadline = now + self._zp_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview COLUMN_CATCH close ID7 pulse"
            status = self._send_gripper_id7(self.id7_closed, "COLUMN_CATCH close ID7 pulse")
            if self.servo_bridge.last_command_ok:
                self.chassis_station_stage = "column_close_wait"
                self.chassis_station_deadline = time.monotonic() + self._zp_settle_s()
            return status

        if self.chassis_station_stage == "column_close_wait":
            self.state = "COLUMN_CATCH close wait"
            if now < self.chassis_station_deadline:
                return f"COLUMN_CATCH close wait {self.chassis_station_deadline - now:.1f}s"
            self.chassis_station_stage = "column_detect"
            return "COLUMN_CATCH pulse complete; wait target leave before rearm"

        return "COLUMN_CATCH station idle"

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
        self.id7 = self.id7_closed
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish("shutdown contract close claw")
        print(
            "SHUTDOWN CONTRACT close claw first "
            f"ID7={self.id7}",
            flush=True,
        )
        claw_status = self.servo_bridge.send_targets(id4=self.id7)
        self.last_command_time = time.monotonic()
        if not self.servo_bridge.last_command_ok:
            self.status = f"shutdown contract close claw failed: {claw_status}"
            self.arm_preview.publish(self.status)
            print(self.status, flush=True)
            return self.status
        time.sleep(max(0.10, min(1.0, self._zp_settle_s())))

        high_status = "not requested"
        if raise_before_home:
            high_status = self._send_fixed_arm_pose_staged(
                DISC_CATCH_POST_HIGH_ID1_TICK,
                DISC_CATCH_POST_HIGH_ID2_TICK,
                DISC_CATCH_POST_HIGH_ID6_TICK,
                "DISC_CATCH post-catch high",
                raising=True,
                id7=self.id7,
                id5=CATCHER_HOME_TICK,
                splitter_id4=(
                    SPLITTER_YELLOW_TICK
                    if splitter_target is None
                    else int(splitter_target)
                ),
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
        id2_id6_status = self.servo_bridge.send_targets(
            id2=self.id2,
            id6=self.id6,
        )
        self.last_command_time = time.monotonic()
        if not self.servo_bridge.last_command_ok:
            self.status = f"shutdown contract ID2/ID6 failed: {id2_id6_status}"
            self.arm_preview.publish(self.status)
            print(self.status, flush=True)
            return self.status
        time.sleep(ARM_JOINT_SEQUENCE_DELAY_S)

        self.id1 = HOME_ID1_TICK
        self.id6 = BASE_YAW_HOME_TICK
        self.id5 = int(
            CATCHER_HOME_TICK if catcher_target is None else catcher_target
        )
        self.splitter_id4 = int(
            SPLITTER_YELLOW_TICK
            if splitter_target is None
            else splitter_target
        )
        self._enforce_angle_gap()
        self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
        self.arm_preview.publish("shutdown contract arm home")
        print(
            "SHUTDOWN CONTRACT "
            f"ID1={self.id1} ID2={self.id2} ID4={self.splitter_id4} "
            f"ID5={self.id5} ID6={self.id6} ID7={self.id7}",
            flush=True,
        )
        status = self.servo_bridge.send_targets(
            id1=self.id1,
            id4=self.id7,
            id6=self.id6,
            id5=self.id5,
            splitter_id4=self.splitter_id4,
        )
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
        self.splitter_id4 = SPLITTER_YELLOW_TICK
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
            self.splitter_id4 = SPLITTER_YELLOW_TICK
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
                    self._zp_settle_s(),
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
            self.startup_deadline = now + max(0.8, self._arm_settle_s())
            self.startup_stage = "home_settle"
            return result

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
            "calibration_model": "measured_10_30cm",
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
            "calibration_model": "measured_10_30cm",
        }
        return plan, (
            f"measured model distance={distance_cm:.1f}cm phase={phase} "
            f"ID1={solved_id1} ID2={solved_id2}"
        )

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
                        self.stage_deadline = time.monotonic() + self._zp_settle_s()
                    elif self.post_center_direct_descend:
                        self.next_stage_after_open = "post_open_retreat"
                        self.next_stage_after_retreat = "descend"
                        self.algorithm_stage = "open_wait"
                        self.stage_deadline = time.monotonic() + self._zp_settle_s()
                    else:
                        self.next_stage_after_open = "post_lock_visual_confirm"
                        self.algorithm_stage = "open_wait"
                        self.stage_deadline = time.monotonic() + self._zp_settle_s()
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
                        self.stage_deadline = time.monotonic() + self._zp_settle_s()
                    elif self.post_center_direct_descend:
                        self.next_stage_after_open = "post_open_retreat"
                        self.next_stage_after_retreat = "descend"
                        self.algorithm_stage = "open_wait"
                        self.stage_deadline = time.monotonic() + self._zp_settle_s()
                    else:
                        self.next_stage_after_open = "post_lock_visual_confirm"
                        self.algorithm_stage = "open_wait"
                        self.stage_deadline = time.monotonic() + self._zp_settle_s()
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
                    self.stage_deadline = time.monotonic() + self._zp_settle_s()
                return result
            return "locked target; waiting to close claw"

        if self.algorithm_stage == "close_wait":
            self.state = "locked target waiting close claw"
            if now < self.stage_deadline:
                return f"locked target; waiting close claw {self.stage_deadline - now:.1f}s"
            if (
                self.active_chassis_station == "PLATFORM_PICK"
                and not self.abort_after_return
            ):
                self.algorithm_stage = "platform_post_grab_id6"
            elif self.post_center_direct_descend and not self.abort_after_return:
                self.algorithm_stage = "post_grab_id2_retreat"
            else:
                self.algorithm_stage = "return"
            self.return_attempts = 0

        if self.algorithm_stage == "platform_post_grab_id6":
            self.state = "PLATFORM_PICK post-grab ID6=270"
            if not self.servo_bridge.write_enabled:
                self.id6 = 270
                self.algorithm_stage = "platform_post_grab_id6_wait"
                self.stage_deadline = now + self._arm_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview PLATFORM_PICK post-grab ID6=270"
            if can_command:
                self.id6 = 270
                result = self._send(
                    "PLATFORM_PICK post-grab move ID6=270",
                    require_feedback=False,
                )
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "platform_post_grab_id6_wait"
                    self.stage_deadline = time.monotonic() + self._arm_settle_s()
                return result
            return "PLATFORM_PICK waiting post-grab ID6=270"

        if self.algorithm_stage == "platform_post_grab_id6_wait":
            if now < self.stage_deadline:
                return (
                    "PLATFORM_PICK waiting ID6=270 "
                    f"{self.stage_deadline - now:.1f}s"
                )
            self.algorithm_stage = "platform_post_grab_open"

        if self.algorithm_stage == "platform_post_grab_open":
            self.state = "PLATFORM_PICK post-grab ID7 open pulse"
            if not self.servo_bridge.write_enabled:
                self.id7 = self.id7_open
                self.algorithm_stage = "platform_post_grab_open_wait"
                self.stage_deadline = now + self._zp_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview PLATFORM_PICK post-grab ID7 open"
            if can_command:
                result = self._send_gripper_id7(
                    self.id7_open,
                    "PLATFORM_PICK post-grab ID7 open",
                )
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "platform_post_grab_open_wait"
                    self.stage_deadline = time.monotonic() + self._zp_settle_s()
                return result
            return "PLATFORM_PICK waiting post-grab ID7 open"

        if self.algorithm_stage == "platform_post_grab_open_wait":
            if now < self.stage_deadline:
                return (
                    "PLATFORM_PICK waiting ID7 open "
                    f"{self.stage_deadline - now:.1f}s"
                )
            self.algorithm_stage = "platform_post_grab_close"

        if self.algorithm_stage == "platform_post_grab_close":
            self.state = "PLATFORM_PICK post-grab ID7 close pulse"
            if not self.servo_bridge.write_enabled:
                self.id7 = self.id7_closed
                self.algorithm_stage = "platform_post_grab_close_wait"
                self.stage_deadline = now + self._zp_settle_s()
                self.arm_preview.set_targets(self.id1, self.id2, self.id7, self.id6)
                return "preview PLATFORM_PICK post-grab ID7 close"
            if can_command:
                result = self._send_gripper_id7(
                    self.id7_closed,
                    "PLATFORM_PICK post-grab ID7 close",
                )
                if self.servo_bridge.last_command_ok:
                    self.algorithm_stage = "platform_post_grab_close_wait"
                    self.stage_deadline = time.monotonic() + self._zp_settle_s()
                return result
            return "PLATFORM_PICK waiting post-grab ID7 close"

        if self.algorithm_stage == "platform_post_grab_close_wait":
            if now < self.stage_deadline:
                return (
                    "PLATFORM_PICK waiting ID7 close "
                    f"{self.stage_deadline - now:.1f}s"
                )
            if not self.servo_bridge.write_enabled:
                self.id1 = 600
                self.id2 = 600
                self.id6 = 350
                return self._finish_platform_pick("PICKED")
            if can_command:
                result = self._send_fixed_arm_pose_staged(
                    600,
                    600,
                    350,
                    "PLATFORM_PICK restore expanded pose",
                    raising=True,
                    id7=self.id7_closed,
                    id5=CATCHER_HOME_TICK,
                    splitter_id4=DISC_CATCH_SPLITTER_READY_TICK,
                )
                if self.servo_bridge.last_command_ok:
                    return self._finish_platform_pick("PICKED")
                return result
            return "PLATFORM_PICK waiting restore expanded pose"

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
                    self.stage_deadline = time.monotonic() + self._zp_settle_s() + 0.30
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
                    self.stage_deadline = time.monotonic() + self._zp_settle_s()
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
        if (
            self.active_chassis_station == "PLATFORM_PICK"
            and self.chassis_station_stage in {
                "platform_preselect",
                "platform_entry_hold",
            }
        ):
            if self.chassis_station_stage == "platform_entry_hold":
                self.id1 = 600
                self.id2 = 600
                self.id6 = 350
                self.id7 = self.id7_closed
                self.id5 = CATCHER_HOME_TICK
                self.splitter_id4 = DISC_CATCH_SPLITTER_READY_TICK
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
                expand_id1 = 600
                expand_id2 = 600
                expand_id6 = 350
                self.id7 = self.id7_closed
                self.id5 = CATCHER_HOME_TICK
                self.splitter_id4 = DISC_CATCH_SPLITTER_READY_TICK
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
        self.letter_detector = ABCDDetector()

    def detect(self, frame):
        if frame is None or not isinstance(frame, np.ndarray) or frame.ndim != 3:
            return []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        rings = self._detect_rings(hsv)
        colored_balls = self._detect_balls(hsv, rings)
        letters = self.letter_detector.detect(frame)
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

    def _detect_balls(self, hsv, rings=None):
        results = []
        rings = rings or []
        for detector in BALL_DETECTORS:
            results.extend(detector.detect(hsv, rings))
        return results

    def _detect_rings(self, hsv):
        results = []
        for color in SHAPE_COLORS:
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
        self._error = None
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            started = time.perf_counter()
            ok, frame = self.cap.read()
            elapsed = time.perf_counter() - started
            if not ok:
                with self._lock:
                    self._error = "camera read failed"
                return
            with self._lock:
                self._frame = frame
                self._last_read_s = elapsed

    def latest(self):
        with self._lock:
            if self._frame is None:
                return False, None, self._last_read_s, self._error
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
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
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
    parser.add_argument("--disable-servo-trigger", action="store_true")
    parser.add_argument(
        "--servo-uart",
        default=os.environ.get(
            "SERVO_UART",
            "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        ),
    )
    parser.add_argument("--servo-baud", type=int, default=int(os.environ.get("SERVO_BAUD", "115200")))
    parser.add_argument(
        "--direct-servo-bus",
        action="store_true",
        default=os.environ.get("DIRECT_SERVO_BUS", "").lower() in {"1", "true", "yes"},
        help="use the Hiwonder HTD-85 USB board for IDs 1/2/3/6 and the ZL ZP USB board for IDs 4/5/7",
    )
    parser.add_argument("--direct-arm-time-ms", type=int, default=int(os.environ.get("DIRECT_ARM_TIME_MS", "600")))
    parser.add_argument("--direct-zp-time-ms", type=int, default=int(os.environ.get("DIRECT_ZP_TIME_MS", "300")))
    parser.add_argument("--direct-gripper-time-ms", type=int, default=int(os.environ.get("DIRECT_GRIPPER_TIME_MS", "210")))
    parser.add_argument("--direct-splitter-time-ms", type=int, default=None)
    parser.add_argument("--direct-repeat", type=int, default=int(os.environ.get("DIRECT_SERVO_REPEAT", "1")))
    parser.add_argument(
        "--direct-arm-uart",
        default=os.environ.get(
            "DIRECT_ARM_UART",
            os.environ.get(
                "SERVO_ARM_UART",
                "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C82109853-if00",
            ),
        ),
    )
    parser.add_argument(
        "--direct-zp-uart",
        default=os.environ.get(
            "DIRECT_ZP_UART",
            os.environ.get(
                "SERVO_ZP_UART",
                "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
        ),
    )
    parser.add_argument("--trigger-command", default=os.environ.get("SERVO_TRIGGER_COMMAND", "linkstart"))
    parser.add_argument("--trigger-color", default="red", choices=BALL_COLORS)
    parser.add_argument("--trigger-kind", default="letter", choices=("ball", "letter", "ring", "any"))
    parser.add_argument("--trigger-stable-frames", type=int, default=3)
    parser.add_argument("--trigger-reset-frames", type=int, default=8)
    parser.add_argument("--trigger-cooldown", type=float, default=5.0)
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
        help="finish the active chassis station after this many seconds with no target",
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
        else load_last_id4_target(args.preview_id4)
    )
    servo_trigger = SerialTrigger(
        args.servo_uart,
        args.servo_baud,
        args.trigger_command,
        args.trigger_color,
        args.trigger_kind,
        args.trigger_stable_frames,
        args.trigger_reset_frames,
        args.trigger_cooldown,
        enabled=not args.disable_servo_trigger and not auto_grasp_enabled,
    )
    arm_preview = ArmPreviewPublisher(
        args.enable_arm_preview or auto_grasp_enabled,
        args.preview_id1,
        args.preview_id2,
        preview_id4,
        args.preview_id6,
    )
    if args.direct_splitter_time_ms is None:
        args.direct_splitter_time_ms = int(
            os.environ.get("DIRECT_SPLITTER_TIME_MS", "500")
        )

    servo_bridge_class = DirectBusServoBridge if args.direct_servo_bus else AbsoluteServoBridge
    if args.direct_servo_bus:
        servo_bridge = servo_bridge_class(
            args.servo_uart,
            args.servo_baud,
            enabled=auto_grasp_enabled,
            write_enabled=execute_auto_grasp,
            arm_device=args.direct_arm_uart,
            zp_device=args.direct_zp_uart,
            arm_time_ms=args.direct_arm_time_ms,
            zp_time_ms=args.direct_zp_time_ms,
            gripper_time_ms=args.direct_gripper_time_ms,
            splitter_time_ms=args.direct_splitter_time_ms,
            repeat=args.direct_repeat,
        )
    else:
        servo_bridge = servo_bridge_class(
            args.servo_uart,
            args.servo_baud,
            enabled=auto_grasp_enabled,
            write_enabled=execute_auto_grasp,
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
    white_line_last_query_sequence = None
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
    camera_executor = concurrent.futures.ThreadPoolExecutor(
        max_workers=1,
        thread_name_prefix="camera-open",
    )
    camera_open_future = None
    secondary_open_future = None
    detection_future = None
    detection_pending_frame = None
    detection_source_frame = None
    detection_epoch = 0

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

    def service_secondary_preselect():
        nonlocal secondary_retry_at
        if (
            grasp_controller.chassis_station_stage != "platform_preselect"
            or secondary_reader is None
        ):
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
        if not secondary_ok or secondary_frame is None:
            return
        if secondary_frame.shape[:2] != (args.height, args.width):
            secondary_frame = cv2.resize(
                secondary_frame,
                (args.width, args.height),
                interpolation=cv2.INTER_AREA,
            )
        secondary_detections = detector.detect(secondary_frame)
        preselect_info = grasp_controller.update_platform_preselect(
            secondary_detections,
            detection_fresh=True,
        )
        if preselect_info:
            resolve_station_outcome(preselect_info)

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

    def _process_white_line_queries(frame, reason):
        nonlocal white_line_last_query_sequence
        pending_white_line_queries = chassis_link.consume_white_line_queries()
        if not pending_white_line_queries:
            return
        query_sequence = pending_white_line_queries[-1]
        if query_sequence != white_line_last_query_sequence:
            # A new H7 white-line phase must start from the current camera
            # frame, never from a held result of a previous route stage.
            white_line_detector.reset_tracking()
            white_line_last_query_sequence = query_sequence
        if frame is None or getattr(frame, "size", 0) == 0:
            for sequence in pending_white_line_queries:
                chassis_link.send_white_line_result(sequence, None)
                print(
                    f"WHITE LINE seq={sequence} not found ({reason})",
                    flush=True,
                )
            return
        line_measurement = white_line_detector.detect(frame)
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
                    f"seq={sequence} "
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
                    chassis_link.finish_active("SKIPPED_NO_TARGET")
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
            for preselect in chassis_link.consume_preselects():
                preselect_status = grasp_controller.begin_platform_preselect(
                    preselect.get("count", 2)
                )
                print(
                    f"CHASSIS PRESELECT STARTED | {preselect_status}",
                    flush=True,
                )
            platform_slot_requests = chassis_link.consume_platform_slots()
            if not grasp_controller.set_field_mode(chassis_link.field_mode):
                print(
                    "CHASSIS FIELD update rejected while station is active",
                    flush=True,
                )
            for aux in chassis_link.consume_aux_zp():
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
                aux_status = grasp_controller.servo_bridge.send_zp_aux(**aux_kwargs)
                aux_success = (
                    grasp_controller.servo_bridge.write_enabled
                    and grasp_controller.servo_bridge.last_command_ok
                )
                chassis_link.complete_aux_zp(aux, aux_success, aux_status)
                print(
                    f"CHASSIS AUX_ZP {target_name}={aux.get('pulse')} "
                    f"T={aux.get('time_ms')}ms "
                    f"done={'yes' if aux_success else 'no'} | {aux_status}",
                    flush=True,
                )
            for aux in chassis_link.consume_aux_htd85():
                aux_status = grasp_controller.servo_bridge.send_htd85_aux(
                    servo_id=aux.get("servo_id"),
                    pulse=aux.get("pulse"),
                    time_ms=aux.get("time_ms"),
                )
                aux_success = (
                    grasp_controller.servo_bridge.write_enabled
                    and grasp_controller.servo_bridge.last_command_ok
                )
                chassis_link.complete_aux_htd85(aux, aux_success, aux_status)
                print(
                    f"CHASSIS AUX_HTD85 ID{aux.get('servo_id')}="
                    f"{aux.get('pulse')} T={aux.get('time_ms')}ms "
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
                    chassis_link.restart_target_watch(
                        grasp_controller._arm_settle_s()
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
                station_status = resolve_station_outcome(station_status)
                print(
                    f"CHASSIS STATION {station} STOPPED | {station_status}",
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
                # Do not tear down a healthy camera during that short window.
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
                detection_pending_frame = None
                detection_source_frame = None
                detections = []
                detection_smoother.tracks = []
                chassis_link.set_ready(False)
                service_station_without_frame()
                time.sleep(0.02)
                continue
            _process_white_line_queries(frame, "frame available")
            service_secondary_preselect()
            detect_elapsed = 0.0
            if detection_future is not None and detection_future.done():
                try:
                    detections, detect_elapsed = detection_future.result()
                    detections = detection_smoother.update(detections)
                    perf_detect_frames += 1
                    detection_source_frame = detection_pending_frame
                    detection_epoch += 1
                    for detection in detections:
                        detection["detection_epoch"] = detection_epoch
                    detection_fresh = True
                except Exception as exc:
                    print(f"detection worker failed: {exc}", flush=True)
                detection_future = None
                detection_pending_frame = None
            if (
                detection_frame_index % detection_interval == 0
                and detection_future is None
            ):
                detection_pending_frame = frame.copy()
                detection_future = detection_executor.submit(
                    detection_process_worker,
                    detection_pending_frame,
                    detection_scale,
                )
            detection_frame_index += 1
            post_started = time.perf_counter()
            active_target_letters = (
                grasp_controller.platform_selected_letters
                if grasp_controller.active_chassis_station == "PLATFORM_PICK"
                and grasp_controller.platform_selected_letters
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
            trigger_info = (
                servo_trigger.update(display_detections)
                if detection_fresh
                else ""
            )
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
                            station_info = (
                                "PLATFORM_PICK skipped wrong target "
                                f"kind={best_rejected.get('kind')} "
                                f"letter={best_rejected.get('letter', '-')}"
                            )
                            grasp_controller.skip_platform_slot(
                                "LETTER_OR_RING_NOT_SELECTED"
                            )
                            station_info = resolve_station_outcome(station_info)
                    if station_info is None:
                        station_info = grasp_controller.update_chassis_station(
                            chassis_link.active_task,
                            detections,
                            frame.shape,
                            detection_fresh=detection_fresh,
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
                    grasp_info = resolve_station_outcome(grasp_info)
                elif not grasp_controller.startup_complete():
                    grasp_info = grasp_controller.update(None, frame.shape)
                else:
                    grasp_info = (
                        "waiting chassis ARM START"
                        if chassis_link.fd is not None
                        else chassis_link.status
                    )
                    arm_preview.publish(grasp_info, preview_target, "WAIT_CHASSIS")
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
                        chassis_link.finish_active("SKIPPED_NO_TARGET")
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
            if trigger_info:
                info = f"{info} | {trigger_info}"
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
        servo_trigger.close()
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
