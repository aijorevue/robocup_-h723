"""Non-blocking H7/RK serial link and its small wire protocol."""

from __future__ import annotations

import glob
import os
import select
import time

try:
    import termios
except ImportError:  # Enables protocol unit tests on Windows.
    termios = None

from .field_mode import FieldMode, parse_field


class ChassisArmLink:
    VALID_TASKS = {"DISC_CATCH", "PLATFORM_PICK", "COLUMN_CATCH"}
    MAX_LINE_BUFFER = 4096
    SERVO_ADAPTER_GLOBS = (
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_*",
        "/dev/serial/by-id/usb-1a86_USB_Serial-*",
    )

    def __init__(self, enabled, device, baudrate, no_target_timeout_s):
        self.enabled = bool(enabled)
        self.device_arg = device
        self.baudrate = int(baudrate)
        self.no_target_timeout_s = max(0.5, float(no_target_timeout_s))
        self.fd = None
        self.device = None
        self.active_task = None
        self.active_sequence = None
        self.active_slot = None
        self.field_mode = FieldMode.RED
        self.station_started = 0.0
        self.last_target_seen = 0.0
        self.line_buffer = ""
        self.status = "chassis link disabled"
        self.pending_starts = []
        self.pending_preselects = []
        self.pending_platform_slots = []
        self.pending_stops = []
        self.pending_preps = []
        self.pending_aux_zp = []
        self.pending_white_line_queries = []
        self.reset_pending = False
        self.reset_in_progress = False
        self.last_reset_completed = 0.0
        self.next_retry = 0.0
        self.last_ready_sent = 0.0
        self.ready_to_run = False
        self.last_completed_task = None
        self.last_completed_sequence = None
        self.last_completed_outcome = None
        self.last_completed_reason = None
        self.last_completed_details = ()
        self.last_aux_sequence = None
        self.last_aux_success = False
        self.last_backpressure_log = 0.0
        self.last_open_failure_log = 0.0
        if self.enabled:
            self._open()

    def _candidate_devices(self):
        if self.device_arg and self.device_arg != "auto":
            return [self.device_arg]
        candidates = []
        if os.path.exists("/dev/h7_chassis"):
            candidates.append("/dev/h7_chassis")
        candidates.extend(
            device
            for device in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if not self._is_servo_adapter_device(device)
        )
        candidates.extend(
            device
            for device in sorted(glob.glob("/dev/serial/by-id/*"))
            if not self._is_servo_adapter_device(device)
        )
        return list(dict.fromkeys(candidates))

    @classmethod
    def _is_servo_adapter_device(cls, device):
        """Keep both USB servo boards out of H7 USB CDC auto-discovery."""

        device_text = str(device)
        if (
            "usb-1a86_USB_Single_Serial_" in device_text
            or "usb-1a86_USB_Serial-" in device_text
        ):
            return True
        real_device = os.path.realpath(device)
        for pattern in cls.SERVO_ADAPTER_GLOBS:
            for adapter in glob.glob(pattern):
                if os.path.realpath(adapter) == real_device:
                    return True
        return False

    def _open(self):
        now = time.monotonic()
        if now < self.next_retry:
            return False
        self.next_retry = now + 1.0
        if termios is None:
            self.status = "CHASSIS LINK requires a POSIX serial device"
            return False
        for device in self._candidate_devices():
            fd = None
            try:
                fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                attrs = termios.tcgetattr(fd)
                attrs[0] = 0
                attrs[1] = 0
                attrs[2] = (
                    (attrs[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB))
                    | termios.CS8
                    | termios.CLOCAL
                    | termios.CREAD
                )
                if hasattr(termios, "CRTSCTS"):
                    attrs[2] &= ~termios.CRTSCTS
                attrs[3] = 0
                baud_flag = getattr(termios, f"B{self.baudrate}", termios.B115200)
                attrs[4] = baud_flag
                attrs[5] = baud_flag
                attrs[6][termios.VMIN] = 0
                attrs[6][termios.VTIME] = 0
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
                self.fd = fd
                self.device = device
                self.status = f"CHASSIS LINK open {device}"
                print(self.status, flush=True)
                self._announce_ready()
                return True
            except OSError as exc:
                if fd is not None:
                    os.close(fd)
                if isinstance(exc, FileNotFoundError):
                    self.status = f"CHASSIS LINK waiting for {device}"
                else:
                    self.status = f"CHASSIS LINK open failed {device}: {exc}"
                if now - self.last_open_failure_log >= 10.0:
                    print(self.status, flush=True)
                    self.last_open_failure_log = now
        if self.device_arg == "auto":
            self.status = (
                "CHASSIS LINK waiting for /dev/h7_chassis or an H7 USB CDC "
                "device (Hiwonder/ZL servo boards excluded)"
            )
            if now - self.last_open_failure_log >= 10.0:
                print(self.status, flush=True)
                self.last_open_failure_log = now
        return False

    def _close_fd(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.device = None
        self.line_buffer = ""

    def _write_payload(self, payload):
        """Drain partial non-blocking writes instead of silently truncating a line."""

        offset = 0
        deadline = time.monotonic() + 0.2
        while offset < len(payload):
            try:
                written = os.write(self.fd, payload[offset:])
                if written <= 0:
                    return False
                offset += written
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                _, writable, _ = select.select([], [self.fd], [], min(0.02, remaining))
                if not writable:
                    continue
        return True

    @staticmethod
    def _sequence_from_parts(parts):
        for index, item in enumerate(parts):
            if item != "SEQ":
                continue
            if index + 1 >= len(parts):
                return True, None
            value = parts[index + 1]
            if not value.isdecimal():
                return True, None
            sequence = int(value)
            if not 0 < sequence <= 0xFFFFFFFF:
                return True, None
            return True, sequence
        return False, None

    @staticmethod
    def _same_sequence(requested, active):
        return requested is None or requested == active

    @staticmethod
    def _task_line(task, state, sequence=None, *details):
        parts = ["RK", "ARM", task, state]
        if sequence is not None:
            parts.extend(("SEQ", str(sequence)))
        parts.extend(str(detail) for detail in details)
        return ",".join(parts)

    def _send_task_state(self, task, state, sequence=None, *details):
        return self.send_line(self._task_line(task, state, sequence, *details))

    def _send_aux_state(self, state, sequence=None, *details):
        parts = ["RK", "AUX_ZP", state]
        if sequence is not None:
            parts.extend(("SEQ", str(sequence)))
        parts.extend(str(detail) for detail in details)
        return self.send_line(",".join(parts))

    def _replay_last_outcome(self):
        if self.last_completed_task is None or self.last_completed_outcome is None:
            return False
        return self._send_task_state(
            self.last_completed_task,
            self.last_completed_outcome,
            self.last_completed_sequence,
            "REASON",
            self.last_completed_reason or "UNKNOWN",
            "FIELD",
            self.field_mode.wire_name,
        )

    def send_line(self, text):
        if not self.enabled:
            return False
        if self.fd is None and not self._open():
            return False
        payload = (text.rstrip() + "\r\n").encode("ascii", "replace")
        try:
            if not self._write_payload(payload):
                raise BlockingIOError
            print(f"CHASSIS TX {text.rstrip()}", flush=True)
            return True
        except BlockingIOError:
            now = time.monotonic()
            self.status = "CHASSIS LINK TX backpressure"
            if now - self.last_backpressure_log >= 10.0:
                print(self.status, flush=True)
                self.last_backpressure_log = now
            return False
        except OSError as exc:
            self.status = f"CHASSIS LINK write failed: {exc}"
            print(self.status, flush=True)
            self._close_fd()
            return False

    @staticmethod
    def _field_from_parts(parts):
        for index, item in enumerate(parts[:-1]):
            if item in {"FIELD", "FIELD="}:
                value = parts[index + 1]
                if value in {"RED", "BLUE"}:
                    return FieldMode(value.lower())
        for item in parts:
            if item in {"RED", "BLUE"}:
                return FieldMode(item.lower())
        return None

    @staticmethod
    def _int_from_parts(parts, key):
        for index, item in enumerate(parts[:-1]):
            if item == key:
                value = parts[index + 1]
                if value.lstrip("-").isdecimal():
                    return int(value)
        return None

    def _set_field_from_wire(self, field_mode, source):
        if field_mode is None:
            return False
        if self.active_task is not None and field_mode != self.field_mode:
            self.send_line(
                f"RK,FIELD,BUSY,{self.field_mode.wire_name},{self.active_task}"
            )
            print(
                f"CHASSIS FIELD change rejected during {self.active_task}: "
                f"{field_mode.wire_name}",
                flush=True,
            )
            return False
        changed = field_mode != self.field_mode
        self.field_mode = field_mode
        if changed:
            self.status = f"CHASSIS FIELD {field_mode.wire_name} from {source}"
            print(self.status, flush=True)
        self.send_line(f"RK,FIELD,ACK,{self.field_mode.wire_name}")
        return True

    def _force_field_from_reset(self, field_mode):
        if field_mode is None:
            return
        changed = field_mode != self.field_mode
        self.field_mode = field_mode
        if changed:
            self.status = f"CHASSIS FIELD {field_mode.wire_name} from ARM,SYNC,RESET"
            print(self.status, flush=True)

    def _announce_ready(self):
        if (
            not self.ready_to_run
            or self.active_task is not None
            or self.reset_pending
            or self.reset_in_progress
        ):
            return False
        self.last_ready_sent = time.monotonic()
        return self.send_line(
            f"RK,ARM,READY,PROTO,2,FIELD,{self.field_mode.wire_name}"
        )

    def set_ready(self, ready):
        became_ready = bool(ready) and not self.ready_to_run
        self.ready_to_run = bool(ready)
        if became_ready:
            self._announce_ready()

    def _handle_line(self, line):
        normalized = line.strip().upper()
        if not normalized:
            return
        print(f"CHASSIS RX {line.strip()}", flush=True)
        parts = [part.strip() for part in normalized.split(",")]
        sequence_present, sequence = self._sequence_from_parts(parts)
        if sequence_present and sequence is None:
            task = parts[1] if len(parts) > 1 and parts[0] == "ARM" else "UNKNOWN"
            self._send_task_state(task, "ERR", None, "REASON", "BAD_SEQ")
            return

        if parts[:3] == ["VISION", "WHITE_LINE", "QUERY"]:
            if sequence is None:
                self.send_line("RK,VISION,WHITE_LINE,ERR,REASON,BAD_SEQ")
                return
            phase = None
            for index, item in enumerate(parts[:-1]):
                if item == "PHASE":
                    phase = parts[index + 1]
                    break
            if phase not in {"TASK1_AFTER_ARC", "TASK2_AFTER_SECONDARY_SHIFT"}:
                self.send_line(
                    "RK,VISION,WHITE_LINE,ERR,REASON,INVALID_PHASE"
                )
                return
            # H7 repeats queries until a fresh result arrives. Keep only the
            # newest request so a slow camera cannot build an obsolete queue.
            self.pending_white_line_queries[:] = [sequence]
            return

        if parts[0] == "FIELD":
            requested = self._field_from_parts(parts[1:])
            if requested is None:
                self.send_line("RK,FIELD,ERR,UNKNOWN_FIELD")
            else:
                self._set_field_from_wire(requested, "FIELD")
            return

        if parts[:2] == ["ARM", "SYNC"]:
            requested = self._field_from_parts(parts[2:])
            if "RESET" in parts[2:]:
                self._force_field_from_reset(requested)
                now = time.monotonic()
                if (
                    not self.reset_pending
                    and not self.reset_in_progress
                    and self.active_task is None
                    and now - self.last_reset_completed < 2.0
                ):
                    self.send_line(
                        f"RK,ARM,RESET,DONE,FIELD,{self.field_mode.wire_name}"
                    )
                    return
                if not self.reset_pending and not self.reset_in_progress:
                    self.reset_pending = True
                    self.pending_starts.clear()
                    self.pending_stops.clear()
                    self.status = (
                        f"CHASSIS reset requested field={self.field_mode.wire_name}"
                    )
                    print(self.status, flush=True)
                self.send_line(
                    f"RK,ARM,RESET,ACK,FIELD,{self.field_mode.wire_name}"
                )
                return
            if requested is not None:
                self._set_field_from_wire(requested, "ARM,SYNC")
            if self.active_task is not None:
                self.send_line(
                    f"RK,ARM,{self.active_task},SYNC_WAIT,FIELD,"
                    f"{self.field_mode.wire_name}"
                )
            else:
                self._announce_ready()
            return

        if (
            len(parts) >= 3
            and parts[0] == "ARM"
            and parts[1] == "PLATFORM_PICK"
            and parts[2] == "PRESELECT"
        ):
            if not self.ready_to_run:
                self._send_task_state(
                    "PLATFORM_PICK", "BUSY", sequence, "REASON", "STARTUP"
                )
                return
            if self.reset_pending or self.reset_in_progress:
                self._send_task_state(
                    "PLATFORM_PICK", "BUSY", sequence, "REASON", "RESET"
                )
                return
            count = self._int_from_parts(parts, "COUNT")
            requested = self._field_from_parts(parts[3:])
            if count != 2:
                self._send_task_state(
                    "PLATFORM_PICK", "ERR", sequence, "REASON", "COUNT_MUST_BE_2"
                )
                return
            if self.active_task is None:
                if requested is not None:
                    self._set_field_from_wire(requested, "ARM,PRESELECT")
                now = time.monotonic()
                self.active_task = "PLATFORM_PICK"
                self.active_sequence = sequence
                self.station_started = now
                self.last_target_seen = now
                self.pending_preselects.append(
                    {"task": "PLATFORM_PICK", "sequence": sequence, "count": count}
                )
                self.status = (
                    f"CHASSIS platform preselect seq={sequence or 'legacy'} "
                    f"field={self.field_mode.wire_name} count={count}"
                )
                print(self.status, flush=True)
            elif self.active_task != "PLATFORM_PICK" or not self._same_sequence(
                sequence, self.active_sequence
            ):
                self._send_task_state(
                    "PLATFORM_PICK",
                    "BUSY",
                    sequence,
                    "ACTIVE",
                    self.active_task,
                    "ACTIVE_SEQ",
                    self.active_sequence
                    if self.active_sequence is not None
                    else "LEGACY",
                )
                return
            self._send_task_state(
                "PLATFORM_PICK",
                "PRESELECT_ACK",
                self.active_sequence,
                "FIELD",
                self.field_mode.wire_name,
                "COUNT",
                2,
            )
            return

        if len(parts) >= 3 and parts[0] == "ARM" and parts[2] == "STATUS":
            task = parts[1]
            if self.active_task == task and self._same_sequence(
                sequence, self.active_sequence
            ):
                self._send_task_state(
                    task,
                    "ACK",
                    self.active_sequence,
                    "FIELD",
                    self.field_mode.wire_name,
                )
            elif (
                self.last_completed_task == task
                and self._same_sequence(sequence, self.last_completed_sequence)
            ):
                self._replay_last_outcome()
            elif self.active_task is not None:
                self._send_task_state(
                    task,
                    "BUSY",
                    sequence,
                    "ACTIVE",
                    self.active_task,
                    "ACTIVE_SEQ",
                    self.active_sequence if self.active_sequence is not None else "LEGACY",
                )
            else:
                self._send_task_state(task, "IDLE", sequence)
            return

        if len(parts) >= 3 and parts[0] == "ARM" and parts[2] == "START":
            task = parts[1]
            if task not in self.VALID_TASKS:
                self._send_task_state(task, "ERR", sequence, "REASON", "UNKNOWN_TASK")
                return
            if not self.ready_to_run:
                self._send_task_state(task, "BUSY", sequence, "REASON", "STARTUP")
                return
            if self.reset_pending or self.reset_in_progress:
                self._send_task_state(task, "BUSY", sequence, "REASON", "RESET")
                return
            requested = self._field_from_parts(parts[3:])
            if (
                sequence is not None
                and self.last_completed_task == task
                and self.last_completed_sequence == sequence
            ):
                self._replay_last_outcome()
                return
            if self.active_task is None:
                if requested is not None:
                    self._set_field_from_wire(requested, "ARM,START")
                now = time.monotonic()
                self.active_task = task
                self.active_sequence = sequence
                self.active_slot = (
                    self._int_from_parts(parts[3:], "SLOT")
                    if task == "PLATFORM_PICK"
                    else None
                )
                self.station_started = now
                self.last_target_seen = now
                self.pending_starts.append(task)
                if task == "PLATFORM_PICK":
                    self.pending_platform_slots.append(
                        {
                            "task": task,
                            "sequence": sequence,
                            "slot": self._int_from_parts(parts[3:], "SLOT"),
                        }
                    )
                self.status = (
                    f"CHASSIS station {task} seq={sequence or 'legacy'} "
                    f"active field={self.field_mode.wire_name}"
                )
                print(self.status, flush=True)
            elif self.active_task != task or not self._same_sequence(
                sequence, self.active_sequence
            ):
                self._send_task_state(
                    task,
                    "BUSY",
                    sequence,
                    "ACTIVE",
                    self.active_task,
                    "ACTIVE_SEQ",
                    self.active_sequence if self.active_sequence is not None else "LEGACY",
                )
                return
            self._send_task_state(
                task,
                "ACK",
                self.active_sequence,
                "FIELD",
                self.field_mode.wire_name,
            )
            return

        if len(parts) >= 3 and parts[0] == "ARM" and parts[2] == "PREP_HIGH":
            task = parts[1]
            if task != "DISC_CATCH":
                self._send_task_state(task, "ERR", sequence, "REASON", "UNKNOWN_PREP")
                return
            if not self.ready_to_run:
                self._send_task_state(task, "BUSY", sequence, "REASON", "STARTUP")
                return
            if self.reset_pending or self.reset_in_progress:
                self._send_task_state(task, "BUSY", sequence, "REASON", "RESET")
                return
            requested = self._field_from_parts(parts[3:])
            if requested is not None:
                self._set_field_from_wire(requested, "ARM,PREP_HIGH")
            self.pending_preps.append(
                {
                    "task": task,
                    "sequence": sequence,
                    "id1": self._int_from_parts(parts, "ID1"),
                    "id2": self._int_from_parts(parts, "ID2"),
                }
            )
            return

        if len(parts) >= 3 and parts[:3] == ["ARM", "AUX_ZP", "SET"]:
            channel = self._int_from_parts(parts, "CHANNEL")
            servo_id = self._int_from_parts(parts, "SERVO_ID")
            pulse = self._int_from_parts(parts, "PULSE")
            time_ms = self._int_from_parts(parts, "TIME")
            requested = self._field_from_parts(parts[3:])
            channel_valid = channel in {12, 23} and servo_id is None
            servo_valid = channel is None and servo_id == 3
            request_valid = (
                (channel_valid or servo_valid)
                and pulse is not None
                and 500 <= pulse <= 2500
                and time_ms is not None
                and 0 <= time_ms <= 9999
            )
            if not request_valid:
                self._send_aux_state(
                    "ERR", sequence, "REASON", "BAD_REQUEST",
                    "FIELD", self.field_mode.wire_name,
                )
                return
            if requested is not None and self.active_task is None:
                self.field_mode = requested
            if sequence is not None and sequence == self.last_aux_sequence:
                self._send_aux_state("ACK", sequence, "FIELD", self.field_mode.wire_name)
                if self.last_aux_success:
                    self._send_aux_state("DONE", sequence, "FIELD", self.field_mode.wire_name)
                return
            if sequence is None or not any(
                item.get("sequence") == sequence for item in self.pending_aux_zp
            ):
                self.pending_aux_zp.append(
                    {
                        "sequence": sequence,
                        "channel": channel,
                        "servo_id": servo_id,
                        "pulse": pulse,
                        "time_ms": time_ms,
                    }
                )
                self._send_aux_state("ACK", sequence, "FIELD", self.field_mode.wire_name)
            return

        if len(parts) >= 3 and parts[0] == "ARM" and parts[2] == "STOP":
            task = parts[1]
            if self.active_task != task or not self._same_sequence(
                sequence, self.active_sequence
            ):
                if (
                    self.last_completed_task == task
                    and self._same_sequence(sequence, self.last_completed_sequence)
                ):
                    self._replay_last_outcome()
                    return
                if self.active_task is not None:
                    self._send_task_state(
                        task,
                        "BUSY",
                        sequence,
                        "ACTIVE",
                        self.active_task,
                        "ACTIVE_SEQ",
                        self.active_sequence
                        if self.active_sequence is not None
                        else "LEGACY",
                    )
                else:
                    self._send_task_state(
                        task, "ERR", sequence, "REASON", "NO_ACTIVE_TASK"
                    )
                return
            if task not in self.pending_stops:
                self.pending_stops.append(task)
            self.status = f"CHASSIS station {task} stop requested"
            print(self.status, flush=True)
            self._send_task_state(task, "STOP_ACK", self.active_sequence)
            return

        if normalized in {"RUN", "START"}:
            self._announce_ready()

    def update(self):
        if not self.enabled:
            return []
        if self.fd is None:
            self._open()
            return []
        now = time.monotonic()
        if self.active_task is None and now - self.last_ready_sent >= 1.0:
            self._announce_ready()
            if self.fd is None:
                return []
        try:
            while True:
                readable, _, _ = select.select([self.fd], [], [], 0.0)
                if not readable:
                    break
                data = os.read(self.fd, 256)
                if not data:
                    self.status = "CHASSIS LINK disconnected"
                    print(self.status, flush=True)
                    self._close_fd()
                    return []
                self.line_buffer += data.decode("ascii", "replace")
                while "\n" in self.line_buffer or "\r" in self.line_buffer:
                    line, sep, rest = self.line_buffer.partition("\n")
                    if not sep:
                        line, _, rest = self.line_buffer.partition("\r")
                    self.line_buffer = rest
                    self._handle_line(line)
                if len(self.line_buffer) > self.MAX_LINE_BUFFER:
                    self.status = "CHASSIS LINK RX line too long; buffer cleared"
                    print(self.status, flush=True)
                    self.line_buffer = ""
        except OSError as exc:
            self.status = f"CHASSIS LINK read failed: {exc}"
            print(self.status, flush=True)
            self._close_fd()
        starts = self.pending_starts
        self.pending_starts = []
        return starts

    def consume_preps(self):
        preps = self.pending_preps
        self.pending_preps = []
        return preps

    def consume_aux_zp(self):
        requests = self.pending_aux_zp
        self.pending_aux_zp = []
        return requests

    def complete_aux_zp(self, request, success, reason=""):
        sequence = request.get("sequence")
        self.last_aux_sequence = sequence
        self.last_aux_success = bool(success)
        if success:
            return self._send_aux_state("DONE", sequence, "FIELD", self.field_mode.wire_name)
        return self._send_aux_state(
            "ERR", sequence, "REASON", reason or "AUX_WRITE_FAILED",
            "FIELD", self.field_mode.wire_name,
        )

    def consume_preselects(self):
        preselects = self.pending_preselects
        self.pending_preselects = []
        return preselects

    def consume_platform_slots(self):
        slots = self.pending_platform_slots
        self.pending_platform_slots = []
        return slots

    def complete_prep(self, prep, success, reason=""):
        task = prep.get("task", "DISC_CATCH")
        sequence = prep.get("sequence")
        if success:
            return self._send_task_state(
                task,
                "PREP_HIGH_ACK",
                sequence,
                "FIELD",
                self.field_mode.wire_name,
            )
        return self._send_task_state(
            task,
            "ERR",
            sequence,
            "REASON",
            reason or "PREP_HIGH_FAILED",
            "FIELD",
            self.field_mode.wire_name,
        )

    def consume_stops(self):
        stops = self.pending_stops
        self.pending_stops = []
        return stops

    def consume_white_line_queries(self):
        queries = self.pending_white_line_queries
        self.pending_white_line_queries = []
        return queries

    def send_white_line_result(self, sequence, measurement):
        if measurement is None:
            return self.send_line(
                f"RK,VISION,WHITE_LINE,NOT_FOUND,SEQ,{int(sequence)}"
            )
        return self.send_line(
            "RK,VISION,WHITE_LINE,FOUND,"
            f"SEQ,{int(sequence)},"
            f"Y10,{int(round(measurement['y_at_center'] * 10.0))},"
            f"A100,{int(round(measurement['angle_deg'] * 100.0))},"
            f"W,{int(measurement['frame_width'])},"
            f"H,{int(measurement['frame_height'])}"
        )

    def consume_reset_request(self):
        if not self.reset_pending or self.reset_in_progress:
            return False
        self.reset_pending = False
        self.reset_in_progress = True
        return True

    def complete_reset(self, success, reason=""):
        if not self.reset_in_progress:
            return False
        self.reset_in_progress = False
        self.pending_starts.clear()
        self.pending_preps.clear()
        self.pending_aux_zp.clear()
        self.pending_preselects.clear()
        self.pending_platform_slots.clear()
        self.pending_stops.clear()
        self.pending_white_line_queries.clear()
        if not success:
            self.status = f"CHASSIS reset failed: {reason}"
            self.send_line(
                f"RK,ARM,RESET,ERR,HOME_FAILED,FIELD,{self.field_mode.wire_name}"
            )
            print(self.status, flush=True)
            return False
        self.active_task = None
        self.active_sequence = None
        self.active_slot = None
        self.last_completed_task = None
        self.last_completed_sequence = None
        self.last_completed_outcome = None
        self.last_completed_reason = None
        self.station_started = 0.0
        self.last_target_seen = 0.0
        self.last_reset_completed = time.monotonic()
        self.status = f"CHASSIS reset complete: {reason}"
        self.send_line(
            f"RK,ARM,RESET,DONE,FIELD,{self.field_mode.wire_name}"
        )
        print(self.status, flush=True)
        self._announce_ready()
        return True

    def note_target(self, target):
        if self.active_task is not None and target is not None:
            self.last_target_seen = time.monotonic()

    def restart_target_watch(self, delay_s=0.0):
        """Start the station search timeout after RK reaches its ready pose."""

        if self.active_task is None:
            return False
        now = time.monotonic()
        self.station_started = now
        self.last_target_seen = now + max(0.0, float(delay_s))
        print(
            f"CHASSIS station {self.active_task} target watch started "
            f"delay={max(0.0, float(delay_s)):.2f}s",
            flush=True,
        )
        return True

    def no_target_timed_out(self, now, searching_for_target):
        if self.active_task is None or not searching_for_target:
            return False
        if self.active_task in {"DISC_CATCH", "COLUMN_CATCH"}:
            return False
        return (now - self.last_target_seen) >= self.no_target_timeout_s

    def _complete_active(self, outcome, reason, *details):
        if self.active_task is None:
            return False
        task = self.active_task
        sequence = self.active_sequence
        self._send_task_state(
            task,
            outcome,
            sequence,
            "REASON",
            reason,
            "FIELD",
            self.field_mode.wire_name,
            *details,
        )
        self.status = (
            f"CHASSIS station {task} seq={sequence or 'legacy'} "
            f"{outcome.lower()}: {reason}"
        )
        print(self.status, flush=True)
        self.active_task = None
        self.active_sequence = None
        self.last_completed_task = task
        self.last_completed_sequence = sequence
        self.last_completed_outcome = outcome
        self.last_completed_reason = reason
        self.last_completed_details = tuple(details)
        self.station_started = 0.0
        self.last_target_seen = 0.0
        return True

    def finish_active(self, reason):
        if self.active_task == "PLATFORM_PICK" and self.active_slot is not None:
            return self._complete_active(
                "DONE",
                reason,
                "SLOT",
                self.active_slot,
            )
        return self._complete_active("DONE", reason)

    def finish_preselect(self, letters):
        selected = tuple(sorted(set(str(letter).upper() for letter in letters)))
        if len(selected) != 2 or self.active_task != "PLATFORM_PICK":
            return False
        return self._complete_active(
            "PRESELECT_DONE",
            "SELECTED",
            "COUNT",
            2,
            "LETTER1",
            selected[0],
            "LETTER2",
            selected[1],
        )

    def fail_active(self, reason):
        return self._complete_active("ERR", reason)

    def recover_active(self, reason):
        """Complete a task-one error only after RK has returned the arm home.

        H7 may continue the route after this response because the arm state is
        explicit. A home failure remains an ERR and is handled as a hard stop.
        """
        if self.active_task != "DISC_CATCH":
            return False
        safe_reason = str(reason or "RECOVERED_ERROR").replace(",", "_")
        return self._complete_active(
            "DONE",
            f"RECOVERED_{safe_reason}",
            "ARM_HOME",
            "OK",
        )

    def close(self):
        self._close_fd()
