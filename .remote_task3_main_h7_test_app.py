#!/usr/bin/env python3
"""Standalone task-three test: main-camera letter pickup while orbiting.

The H7 owns a closed-loop 500 mm radius, 360 degree orbit. RK owns the
main-camera ABCD detector and pauses the orbit only for a confirmed letter.
Arm motion reuses the task-two direct servo-board path and the measured IK
model from the deployed workspace.
"""

from __future__ import annotations

import argparse
import copy
import os
import select
import sys
import time
from collections import deque

import cv2
import numpy as np


ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# Keep one implementation of the tested servo framing and arm behavior.
from task2_secondary_h7_test_app import (  # noqa: E402
    ARM_TIME_MS,
    CENTER_DEADBAND_PX,
    CENTER_ID2_RANGE,
    CENTER_ID6_RANGE,
    CENTER_STEP_TICKS,
    CENTER_TIME_MS,
    CENTER_TRACK_MAX_JUMP_PX,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    GRIPPER_TIME_MS,
    HIGH,
    LETTER_WORK,
    LETTERS,
    MAIN_LETTER_MIN_CONFIDENCE,
    TARGET_TRACK_MISSING_TIMEOUT_S,
    ServoBoards,
    import_grasp_model,
    import_main_detector,
    letter_detections,
    open_camera,
    solve_descend_pose,
)


MAIN_CAMERA = "/dev/v4l/by-path/platform-fc800000.usb-usb-0:1:1.0-video-index0"
H7_DEVICE = "/dev/h7_chassis"
ARM_DEVICE = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C82109853-if00"
ZP_DEVICE = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

TARGET_VOTE_WINDOW = 10
TARGET_REQUIRED_VOTES = 2
# A moving target can shift substantially between two detector frames. Keep
# the main-camera detector strict, but do not reject a valid track because the
# chassis moved the target more than the task-two stationary tolerance.
TARGET_CENTER_TOL_PX = 220.0
TARGET_TRACK_MAX_JUMP_PX = max(320.0, CENTER_TRACK_MAX_JUMP_PX)
TARGET_RECHECK_DELAY_S = 0.15
ORBIT_RESCUE_INTERVAL_FRAMES = 3
ORBIT_RESCUE_MIN_CONFIDENCE = 38.0
TASK3_LETTERS = tuple(sorted(LETTERS))
TASK3_ORBIT_RADIUS_MM = 500
TASK3_ORBIT_ANGLE_DEG = 360
TASK3_LETTER_MIN_CONFIDENCE = max(40.0, MAIN_LETTER_MIN_CONFIDENCE - 5.0)


class Task3H7Link:
    """Sequence-aware line transport for the isolated TASK3 protocol."""

    def __init__(self, path: str):
        self.path = path
        self.fd = None
        self.rx = bytearray()

    def open(self) -> None:
        if not os.path.exists(self.path):
            raise RuntimeError(f"H7 CDC missing: {self.path}")
        # Importing the tested UART setup avoids pyserial ownership changes.
        from task2_secondary_h7_test_app import open_uart

        self.fd = open_uart(self.path)
        print(f"TASK3 H7 OPEN {self.path}", flush=True)

    def send(self, line: str) -> None:
        if self.fd is None:
            self.open()
        payload = (line.rstrip("\r\n") + "\r\n").encode("ascii")
        offset = 0
        while offset < len(payload):
            offset += os.write(self.fd, payload[offset:])
        termios_drain(self.fd)
        print(f"TASK3 H7 TX {line.rstrip()}", flush=True)

    def _read_lines(self, timeout_s: float) -> list[str]:
        if self.fd is None:
            return []
        ready, _, _ = select.select([self.fd], [], [], timeout_s)
        if not ready:
            return []
        self.rx.extend(os.read(self.fd, 4096))
        lines = []
        while b"\n" in self.rx:
            raw, _, self.rx = self.rx.partition(b"\n")
            line = raw.decode("ascii", "replace").strip("\r")
            if line:
                print(f"TASK3 H7 RX {line}", flush=True)
                lines.append(line)
        return lines

    def poll(self) -> list[str]:
        """Read already available H7 lines without delaying camera capture."""
        return self._read_lines(0.0)

    @staticmethod
    def _matches(line: str, status: str, sequence: int) -> bool:
        fields = [item.strip() for item in line.split(",")]
        return (
            len(fields) >= 6
            and fields[:4] == ["H7", "TEST", "TASK3", status]
            and fields[4] == "SEQ"
            and fields[5] == str(sequence)
        )

    def wait_status(self, sequence: int, status: str, timeout_s: float) -> str:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for line in self._read_lines(min(0.1, max(0.0, deadline - time.monotonic()))):
                if self._matches(line, status, sequence):
                    return line
                if self._matches(line, "ERR", sequence):
                    raise RuntimeError(f"H7_{status}_ERR {line}")
        raise RuntimeError(f"H7_{status}_TIMEOUT")

    def pause_until_confirmed(
        self, sequence: int, field: str, timeout_s: float
    ) -> str:
        """Retry PAUSE until H7 confirms that the motors are stopped.

        The ACK is useful diagnostics, but PAUSED is the safety point at which
        RK may move the arm. Retrying also covers a CDC packet lost while H7
        was transmitting an unrelated status line.
        """
        deadline = time.monotonic() + timeout_s
        next_send = 0.0
        command = f"RK,TEST,TASK3,PAUSE,SEQ,{sequence},FIELD,{field}"
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.send(command)
                next_send = now + 0.25
            for line in self._read_lines(
                min(0.05, max(0.0, deadline - time.monotonic()))
            ):
                if self._matches(line, "PAUSED", sequence):
                    return line
                if self._matches(line, "ERR", sequence):
                    raise RuntimeError(f"H7_PAUSE_ERR {line}")
        raise RuntimeError(f"H7_PAUSED_TIMEOUT retries=PAUSE")

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def termios_drain(fd: int) -> None:
    """Drain a raw UART without importing termios at module import time."""
    import termios

    termios.tcdrain(fd)


def _motion_enhanced_frame(frame: np.ndarray) -> np.ndarray:
    """Recover local black-glyph contrast from short motion-blurred frames."""
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lightness, a_channel, b_channel = cv2.split(lab)
    lightness = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(
        lightness
    )
    enhanced = cv2.cvtColor(
        cv2.merge((lightness, a_channel, b_channel)), cv2.COLOR_LAB2BGR
    )
    blurred = cv2.GaussianBlur(enhanced, (0, 0), 1.2)
    return cv2.addWeighted(enhanced, 1.45, blurred, -0.45, 0.0)


def _configure_orbit_rescue_detector(detector):
    """Relax only the orbit rescue pass; task-two main detection stays strict."""
    for name, limit in (
        ("min_confidence", 0.38),
        ("dark_letter_min_confidence", 0.38),
    ):
        if hasattr(detector, name):
            try:
                setattr(detector, name, min(float(getattr(detector, name)), limit))
            except (TypeError, ValueError):
                pass
    return detector


def _merge_letter_candidates(candidates: list[dict]) -> list[dict]:
    """Keep the strongest observation of each nearby letter candidate."""
    merged = []
    for candidate in sorted(
        candidates,
        key=lambda item: float(item.get("confidence", 0.0)),
        reverse=True,
    ):
        label = str(candidate.get("letter", "")).upper()
        center = np.asarray(candidate.get("center", (0, 0)), dtype=np.float32)
        duplicate = any(
            label == str(existing.get("letter", "")).upper()
            and np.linalg.norm(
                center - np.asarray(existing.get("center", (0, 0)), dtype=np.float32)
            ) <= 0.35 * max(
                float(candidate.get("bbox", (0, 0, 1, 1))[2]),
                float(candidate.get("bbox", (0, 0, 1, 1))[3]),
                float(existing.get("bbox", (0, 0, 1, 1))[2]),
                float(existing.get("bbox", (0, 0, 1, 1))[3]),
            )
            for existing in merged
        )
        if not duplicate:
            merged.append(candidate)
    return merged


def _letter_candidates(
    frame: np.ndarray,
    detector,
    grabbed: set[str],
    min_confidence: float,
    rescue_detector=None,
    frame_index: int = 0,
) -> list[dict]:
    detections = letter_detections(detector, frame, min_confidence)
    rescue = []
    # A moving target may be blurred without making the strict pass empty:
    # run a bounded rescue pass periodically and fuse it with the strict one.
    if rescue_detector is not None and (
        not detections or frame_index % ORBIT_RESCUE_INTERVAL_FRAMES == 0
    ):
        rescue = letter_detections(
            rescue_detector,
            _motion_enhanced_frame(frame),
            ORBIT_RESCUE_MIN_CONFIDENCE,
        )
    detections = _merge_letter_candidates(detections + rescue)
    return [
        item for item in detections
        if str(item.get("letter", "")).upper() in TASK3_LETTERS
        and str(item.get("letter", "")).upper() not in grabbed
    ]


def _nearest_target(candidates: list[dict], frame_shape) -> dict | None:
    if not candidates:
        return None
    height, width = frame_shape[:2]
    center = np.asarray((width / 2.0, height / 2.0), dtype=np.float32)
    return min(
        candidates,
        key=lambda item: float(
            np.linalg.norm(
                np.asarray(item.get("center", center), dtype=np.float32) - center
            )
        ),
    )


def _draw_view(frame: np.ndarray, candidates: list[dict], grabbed: set[str], state: str):
    view = frame.copy()
    for item in candidates:
        x, y = map(int, item.get("center", (0, 0)))
        label = str(item.get("letter", "?")).upper()
        cv2.circle(view, (x, y), 8, (0, 255, 0), 2)
        cv2.putText(
            view, label, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 255, 0), 2,
        )
    cv2.putText(
        view, f"TASK3 MAIN / {state} / GRABBED={','.join(sorted(grabbed)) or '-'}",
        (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2,
    )
    return view


def _stable_target(
    candidates: list[dict],
    frame_shape,
    history: deque,
) -> dict | None:
    target = _nearest_target(candidates, frame_shape)
    if target is None:
        history.append(("none", None, None))
        return None

    key = str(target.get("letter", "")).upper()
    center = tuple(map(int, target.get("center", (0, 0))))
    history.append((key, center, dict(target)))
    stable = [
        item for item_key, item_center, item in history
        if item_key == key
        and item_center is not None
        and item is not None
        and np.linalg.norm(
            np.asarray(item_center, dtype=np.float32)
            - np.asarray(center, dtype=np.float32)
        ) <= TARGET_CENTER_TOL_PX
    ]
    if len(stable) < TARGET_REQUIRED_VOTES:
        return None
    return max(
        stable,
        key=lambda item: float(item.get("confidence", 0.0)),
    )


def _letter_grasp(boards: ServoBoards, target: dict, grasp_model) -> None:
    """Execute the task-two letter grasp sequence at a paused orbit station."""
    descend = solve_descend_pose(target, grasp_model)
    boards.open_gripper()
    boards.arm(
        {
            1: descend["id1"],
            2: descend["id2"],
            6: int(target.get("center_id6", HIGH[6])),
        }
    )
    print(
        f"TASK3 LETTER_DESCEND letter={target['letter']} "
        f"depth={descend['distance_cm']:.1f}cm "
        f"ID1={descend['id1']} ID2={descend['id2']} model={descend['model']}",
        flush=True,
    )
    time.sleep(ARM_TIME_MS / 1000.0)
    boards.close_gripper()
    boards.pose_high()
    boards.arm(LETTER_WORK)
    time.sleep(ARM_TIME_MS / 1000.0)
    boards.open_gripper()
    boards.close_gripper()
    boards.pose_high()
    print(
        "TASK3 LETTER_DONE "
        f"letter={target['letter']} ID1={LETTER_WORK[1]} "
        f"ID2={LETTER_WORK[2]} ID6={LETTER_WORK[6]} "
        f"GRIPPER={GRIPPER_OPEN}->{GRIPPER_CLOSED}",
        flush=True,
    )


def _center_task3_target(
    camera: cv2.VideoCapture,
    detector,
    rescue_detector,
    reference: dict,
    boards: ServoBoards,
) -> dict:
    """Re-center a paused target with the same rescue path used in orbit."""
    id2 = int(HIGH[2])
    id6 = int(HIGH[6])
    corrections = 0
    frame_index = 0
    last_target = reference
    missing_started = None
    reference_label = str(reference.get("letter", "")).upper()

    while True:
        ok, frame = camera.read()
        if not ok or frame is None:
            time.sleep(0.01)
            continue
        candidates = [
            item for item in _letter_candidates(
                frame,
                detector,
                set(),
                TASK3_LETTER_MIN_CONFIDENCE,
                rescue_detector,
                frame_index,
            )
            if str(item.get("letter", "")).upper() == reference_label
        ]
        frame_index += 1
        if not candidates:
            now = time.monotonic()
            if missing_started is None:
                missing_started = now
            if now - missing_started >= TARGET_TRACK_MISSING_TIMEOUT_S:
                print(
                    "TASK3 TARGET TRACK_TIMEOUT "
                    f"missing_s={now - missing_started:.1f}",
                    flush=True,
                )
                raise RuntimeError("TARGET_TRACK_TIMEOUT")
            continue

        missing_started = None
        previous_center = np.asarray(
            last_target.get("center", ()), dtype=np.float32
        )
        if previous_center.shape == (2,):
            distances = [
                float(
                    np.linalg.norm(
                        np.asarray(item.get("center", previous_center), dtype=np.float32)
                        - previous_center
                    )
                )
                for item in candidates
            ]
            nearest_index = int(np.argmin(distances))
            target = (
                candidates[nearest_index]
                if distances[nearest_index] <= TARGET_TRACK_MAX_JUMP_PX
                else max(
                    candidates,
                    key=lambda item: float(item.get("confidence", 0.0)),
                )
            )
        else:
            target = max(
                candidates,
                key=lambda item: float(item.get("confidence", 0.0)),
            )
        last_target = target

        height, width = frame.shape[:2]
        cx, cy = target.get("center", (width / 2.0, height / 2.0))
        error_x = float(cx) - width / 2.0
        error_y = float(cy) - height / 2.0
        if abs(error_x) <= CENTER_DEADBAND_PX and abs(error_y) <= CENTER_DEADBAND_PX:
            target["frame_shape"] = frame.shape
            target["center_id6"] = id6
            print(
                f"TASK3 TARGET CENTERED dx={error_x:.0f} dy={error_y:.0f} "
                f"ID2={id2} ID6={id6} corrections={corrections}",
                flush=True,
            )
            return target

        next_id2 = id2
        next_id6 = id6
        if abs(error_x) > CENTER_DEADBAND_PX:
            next_id6 += -CENTER_STEP_TICKS if error_x > 0.0 else CENTER_STEP_TICKS
        if abs(error_y) > CENTER_DEADBAND_PX:
            next_id2 += -CENTER_STEP_TICKS if error_y > 0.0 else CENTER_STEP_TICKS
        next_id2 = max(CENTER_ID2_RANGE[0], min(CENTER_ID2_RANGE[1], next_id2))
        next_id6 = max(CENTER_ID6_RANGE[0], min(CENTER_ID6_RANGE[1], next_id6))
        if next_id2 == id2 and next_id6 == id6:
            target["frame_shape"] = frame.shape
            target["center_id6"] = id6
            print(
                f"TASK3 TARGET CENTER_LIMIT dx={error_x:.0f} dy={error_y:.0f} "
                f"ID2={id2} ID6={id6} corrections={corrections}",
                flush=True,
            )
            return target

        id2, id6 = next_id2, next_id6
        boards.arm({2: id2, 6: id6}, CENTER_TIME_MS)
        corrections += 1
        print(
            f"TASK3 TARGET CENTER_STEP dx={error_x:.0f} dy={error_y:.0f} "
            f"ID2={id2} ID6={id6} step={CENTER_STEP_TICKS} "
            f"motion={CENTER_TIME_MS}ms",
            flush=True,
        )
        time.sleep(CENTER_TIME_MS / 1000.0)


def run(args) -> int:
    camera = boards = h7 = None
    started = False
    completed = False
    sequence = (int(time.time() * 1000.0)) & 0xFFFFFFFF
    grabbed: set[str] = set()
    history: deque = deque(maxlen=TARGET_VOTE_WINDOW)
    try:
        detector = import_main_detector()
        orbit_rescue_detector = _configure_orbit_rescue_detector(
            copy.copy(detector)
        )
        grasp_model = import_grasp_model()
        camera = open_camera(args.main_camera)
        print(
            f"TASK3 MAIN CAMERA_OPEN path={args.main_camera} "
            "ROLE=MAIN ONLY",
            flush=True,
        )
        if not args.execute_h7:
            print("TASK3 CAMERA_ONLY READY detector=ABCD", flush=True)
            deadline = time.monotonic() + args.camera_timeout_s
            while time.monotonic() < deadline:
                ok, frame = camera.read()
                if not ok or frame is None:
                    continue
                candidates = _letter_candidates(frame, detector, set(), MAIN_LETTER_MIN_CONFIDENCE)
                cv2.imshow(
                    "Task3 main",
                    _draw_view(frame, candidates, set(), "CAMERA_ONLY"),
                )
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            return 0

        boards = ServoBoards(args.arm_uart, args.zp_uart)
        boards.pose_high()
        print(
            "TASK3 ARM_HIGH ID1=600 ID2=600 ID6=650 "
            "ZP4=1200 ZP5=800 ZP7=1300",
            flush=True,
        )
        h7 = Task3H7Link(args.h7_device)
        h7.open()
        field = args.field.upper()
        h7.send(
            f"RK,TEST,TASK3,START,SEQ,{sequence},FIELD,{field},"
            f"RADIUS_MM,{TASK3_ORBIT_RADIUS_MM},"
            f"ANGLE_DEG,{TASK3_ORBIT_ANGLE_DEG}"
        )
        h7.wait_status(sequence, "ACK", args.ack_timeout_s)
        h7.wait_status(sequence, "RUNNING", args.running_timeout_s)
        started = True
        print(
            f"TASK3 ORBIT_STARTED radius={TASK3_ORBIT_RADIUS_MM}mm "
            f"angle={TASK3_ORBIT_ANGLE_DEG}deg field={field}",
            flush=True,
        )

        orbit_frame_index = 0
        while True:
            for line in h7.poll():
                if h7._matches(line, "DONE", sequence):
                    completed = True
                    break
                if h7._matches(line, "ERR", sequence):
                    raise RuntimeError(f"H7_ORBIT_ERR {line}")
            if completed:
                break

            ok, frame = camera.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            candidates = _letter_candidates(
                frame,
                detector,
                grabbed,
                TASK3_LETTER_MIN_CONFIDENCE,
                orbit_rescue_detector,
                orbit_frame_index,
            )
            orbit_frame_index += 1
            target = _stable_target(candidates, frame.shape, history)
            cv2.imshow(
                "Task3 main",
                _draw_view(frame, candidates, grabbed, "ORBIT"),
            )
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                raise KeyboardInterrupt
            if target is None:
                continue

            label = str(target.get("letter", "")).upper()
            history.clear()
            print(
                f"TASK3 TARGET_CONFIRMED letter={label} "
                f"center={target.get('center')} depth_cm={target.get('distance_cm', '-')}",
                flush=True,
            )
            h7.pause_until_confirmed(sequence, field, args.pause_timeout_s)
            print(f"TASK3 ORBIT_PAUSED letter={label}", flush=True)

            try:
                centered = _center_task3_target(
                    camera, detector, orbit_rescue_detector, target, boards
                )
            except RuntimeError as exc:
                if not str(exc).startswith("TARGET_TRACK_TIMEOUT"):
                    raise
                # The orbit is already safely paused. If the confirmed target
                # disappears during the bounded re-centering window, release
                # the pause and keep scanning instead of leaving the chassis
                # stopped until the outer error handler sends STOP.
                print(
                    "TASK3 TARGET_LOST_AFTER_PAUSE "
                    f"missing_timeout={TARGET_TRACK_MISSING_TIMEOUT_S:.1f}s "
                    "action=RESUME_ORBIT",
                    flush=True,
                )
                h7.send(
                    f"RK,TEST,TASK3,RESUME,SEQ,{sequence},FIELD,{field}"
                )
                h7.wait_status(sequence, "RESUMED", args.resume_timeout_s)
                history.clear()
                print("TASK3 ORBIT_RESUMED reason=TARGET_TRACK_TIMEOUT", flush=True)
                continue
            _letter_grasp(boards, centered, grasp_model)
            grabbed.add(label)
            history.clear()
            time.sleep(TARGET_RECHECK_DELAY_S)

            h7.send(
                f"RK,TEST,TASK3,RESUME,SEQ,{sequence},FIELD,{field}"
            )
            h7.wait_status(sequence, "RESUMED", args.resume_timeout_s)
            print(
                f"TASK3 ORBIT_RESUMED grabbed={','.join(sorted(grabbed))}",
                flush=True,
            )

        print(
            f"TASK3 TEST COMPLETE orbit={TASK3_ORBIT_ANGLE_DEG}deg "
            f"radius={TASK3_ORBIT_RADIUS_MM}mm "
            f"grabbed={','.join(sorted(grabbed)) or 'none'}",
            flush=True,
        )
        return 0
    except KeyboardInterrupt:
        print("TASK3 ABORTED", flush=True)
        return 2
    except Exception as exc:
        print(f"TASK3 ERROR {exc}", flush=True)
        return 1
    finally:
        if h7 is not None and started and not completed:
            try:
                h7.send(
                    f"RK,TEST,TASK3,STOP,SEQ,{sequence},FIELD,{args.field.upper()}"
                )
                h7.wait_status(sequence, "STOPPED", 2.0)
            except Exception as exc:
                print(f"TASK3 STOP_RESULT {exc}", flush=True)
        if camera is not None:
            camera.release()
        if boards is not None:
            boards.close()
        if h7 is not None:
            h7.close()
        cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="RoboCup task-three standalone orbit and ABCD test"
    )
    parser.add_argument("--field", choices=("red", "blue"), required=True)
    parser.add_argument("--main-camera", default=MAIN_CAMERA)
    parser.add_argument("--h7-device", default=H7_DEVICE)
    parser.add_argument("--arm-uart", default=ARM_DEVICE)
    parser.add_argument("--zp-uart", default=ZP_DEVICE)
    parser.add_argument("--execute-h7", action="store_true")
    parser.add_argument("--camera-timeout-s", type=float, default=30.0)
    parser.add_argument("--ack-timeout-s", type=float, default=4.0)
    parser.add_argument("--running-timeout-s", type=float, default=8.0)
    parser.add_argument("--pause-timeout-s", type=float, default=5.0)
    parser.add_argument("--resume-timeout-s", type=float, default=3.0)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
