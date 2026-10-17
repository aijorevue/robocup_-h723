#!/usr/bin/env python3
"""Check the H7/RK task-three contract without touching either board."""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path


def _require(pattern: str, text: str, label: str) -> None:
    if re.search(pattern, text, re.MULTILINE) is None:
        raise AssertionError(label)


def _python_constants(path: Path) -> dict[str, object]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    constants: dict[str, object] = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        try:
            value = ast.literal_eval(node.value)
        except (ValueError, TypeError):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name):
                constants[target.id] = value
    return constants


def _expect(constants: dict[str, object], name: str, expected: object) -> None:
    actual = constants.get(name)
    if actual != expected:
        raise AssertionError(f"{name}={actual!r}, expected {expected!r}")


def verify(root: Path) -> int:
    h7_config = (root / "include" / "app_config.h").read_text(encoding="utf-8")
    h7_main = (root / "src" / "main.c").read_text(encoding="utf-8")
    rk_source_path = root / "board_target_vision.py"
    rk_source = rk_source_path.read_text(encoding="utf-8")
    rk_constants = _python_constants(rk_source_path)

    checks = 0
    for pattern, label in (
        (r"ROUTE_FORMAL_TASK3_RED_RING_PREPLACE_FIRST_SHIFT_DISTANCE_M\s+0\.162f", "H7 RED first shift"),
        (r"ROUTE_FORMAL_TASK3_RED_RING_PREPLACE_SECOND_SHIFT_DISTANCE_M\s+0\.292f", "H7 RED second shift"),
        (r"ROUTE_FORMAL_TASK3_RED_PRE_RING_FORWARD_MS\s+1000U", "H7 RED first forward"),
        (r"ROUTE_FORMAL_TASK3_RED_RING_PREPLACE_POST_SHIFT_FORWARD_MS\s+1500U", "H7 RED second forward"),
        (r"ROUTE_TASK3_BLUE_RING_PREPLACE_FIRST_SHIFT_DISTANCE_M\s+0\.159f", "H7 BLUE first shift"),
        (r"ROUTE_TASK3_BLUE_RING_PREPLACE_SECOND_SHIFT_DISTANCE_M\s+0\.292f", "H7 BLUE second shift"),
        (r"ROUTE_TASK3_BLUE_PRE_RING_FORWARD_MS\s+1000U", "H7 BLUE first forward"),
        (r"ROUTE_TASK3_BLUE_RING_PREPLACE_POST_SHIFT_FORWARD_MS\s+1500U", "H7 BLUE second forward"),
    ):
        _require(pattern, h7_config, label)
        checks += 1

    for pattern, label in (
        (r"PRE_RING_SHIFT_LEFT_162", "H7 RED 162 mm log"),
        (r"PRE_RING_SHIFT_LEFT_159", "H7 BLUE 159 mm log"),
        (r"PRE_RING_FORWARD_1000MS", "H7 RED/BLUE 1000 ms log"),
        (r"PRE_RING_SHIFT_LEFT_292", "H7 RED/BLUE 292 mm log"),
        (r"POST_SHIFT_FORWARD_1500MS", "H7 RED/BLUE 1500 ms log"),
    ):
        _require(pattern, h7_main, label)
        checks += 1

    for name, expected in (
        ("COLUMN_TASK3_CAPTURE_WINDOW_X_MIN", 50.0),
        ("COLUMN_TASK3_CAPTURE_WINDOW_X_MAX", 750.0),
        ("COLUMN_TASK3_CAPTURE_WINDOW_Y_MIN", 100.0),
        ("COLUMN_TASK3_CAPTURE_WINDOW_Y_MAX", 700.0),
        ("TASK3_RING_PLACE_RETURN_HIGH", (620, 440, 413)),
        ("TASK3_RING_PLACE_RELEASE_ID2_TICK", 423),
        ("TASK3_RING_PLACE_RELEASE_POST_ID1_TICK", 570),
        ("TASK3_RING_PLACE_RELEASE_POST_ID2_TICK", 430),
        ("TASK3_RING_PLACE_RELEASE_POST_CONFIRM_TIME_MS", 1000),
        ("TASK3_SUPPLEMENT_HIGH", (650, 605, 410)),
        ("TASK3_SUPPLEMENT_GRIPPER_OPEN_TICK", 450),
        ("TASK3_SUPPLEMENT_GRIPPER_CLOSED_TICK", 275),
        ("TASK3_SUPPLEMENT_PLACE", (530, 350, 620)),
    ):
        _expect(rk_constants, name, expected)
        checks += 1

    _require(r"ID1_ID2_RELEASE_ADJUST", rk_source, "RK release-adjust action")
    _require(r"for source_detection in detections:\s*\n\s*detection = dict\(source_detection\)", rk_source,
             "RK scaling must copy detector mappings")
    _require(r"center_array = np\.asarray\(center, dtype=np\.float64\)", rk_source,
             "RK window must accept array-like centers")
    _require(r"np\.isfinite\(center_array\[:2\]\)\.all\(\)", rk_source,
             "RK window must reject non-finite centers")
    checks += 4

    print(f"robot contract checks: PASS ({checks})")
    return 0


if __name__ == "__main__":
    workspace = Path(__file__).resolve().parents[1]
    try:
        raise SystemExit(verify(workspace))
    except (AssertionError, OSError, SyntaxError) as exc:
        print(f"robot contract checks: FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
