import os
import sys
import json
import time
import threading
import argparse

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

from mapping import CHANNELS, parse_line, calibrate_neutral, MovementMapper
from classify import (
    ActionClassifier, NEUTRAL, LEAN, LIFT_L, LIFT_R, TURN_LEFT, TURN_RIGHT,
)
from game import SequenceGame, POINTS_PER_STEP, BONUS_ON_COMPLETE

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
SERIAL_PORT = "/dev/cu.usbserial-59690891081"   # find yours with:  ls /dev/cu.*
BAUD = 115200
CALIB_SECONDS = 2.0                      # neutral hold at startup
GUI_UPDATE_MS = 50

# Shared state file consumed by run_professional_rehab.py's live sensor mode.
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "live_state.json")
STATE_WRITE_INTERVAL = 0.03  # seconds, throttles disk writes

# Joint angles published by run_professional_rehab.py, consumed by "mujoco"
# mode: the avatar's real-time motion is monitored and recognized here.
MOTION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "mujoco_motion.json")
MOTION_STALE_SECONDS = 0.5   # older than this = viewer not running

# Full-scale joint angles; must match run_professional_rehab.py's maxima so
# the normalized features land on the same [0, 1] scale the IMUs produce.
LEAN_MAX_DEG = 32.0
LIFT_MAX_DEG = 35.0
TURN_MAX_DEG = 45.0

# how each label is shown: (text, color)
DISPLAY = {
    NEUTRAL:    ("\u2014 neutral \u2014", "#8a8f98"),
    LEAN:       ("Lean forward",      "#e08a2b"),
    LIFT_L:     ("Lift LEFT thigh",   "#27ae60"),
    LIFT_R:     ("Lift RIGHT thigh",  "#2980b9"),
    TURN_LEFT:  ("↺ Turn LEFT",  "#8e44ad"),
    TURN_RIGHT: ("Turn RIGHT ↻", "#16a085"),
}

# how each game event is flashed: (text, color)
EVENT_DISPLAY = {
    "correct": ("+%d" % POINTS_PER_STEP, "#27ae60"),
    "restart": ("Restarted the sequence  +%d" % POINTS_PER_STEP, "#f1c40f"),
    "wrong": ("Wrong move — sequence reset", "#e74c3c"),
    "complete": ("Sequence complete!  +%d bonus" % BONUS_ON_COMPLETE, "#f1c40f"),
}
EVENT_FLASH_SECONDS = 1.5

SIM_KEY_TO_LABEL = {
    "q": NEUTRAL,
    "w": LEAN,
    "e": LIFT_L,
    "r": LIFT_R,
    "t": TURN_LEFT,
    "y": TURN_RIGHT,
}


def render_sequence_html(sequence, progress):
    parts = []
    for i, action in enumerate(sequence):
        name = DISPLAY.get(action, (action, "#cfd3d8"))[0]
        if i < progress:
            color = "#27ae60"       # done
        elif i == progress:
            color = "#f1c40f"       # up next
        else:
            color = "#5a5f68"       # pending
        parts.append('<span style="color:%s;">%s</span>' % (color, name))
    return "  →  ".join(parts)


class SimulationClassifier(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self.state = "calibrating"       # "calibrating" | "running"
        self.progress = 0.0
        self.label = NEUTRAL
        self.features = {"lean": 0.0, "lift_L": 0.0,
                         "lift_R": 0.0, "turn": 0.0}
        self.error = None

        self.game = SequenceGame()
        self.game_event = None
        self.game_event_ts = 0.0

        self._calib_start = time.perf_counter()
        self._last_write = 0.0

    def open(self):
        return None

    def recalibrate(self):
        with self._lock:
            self.state = "calibrating"
            self.progress = 0.0
            self.label = NEUTRAL
            self.features = {"lean": 0.0, "lift_L": 0.0,
                             "lift_R": 0.0, "turn": 0.0}
            self._calib_start = time.perf_counter()
            self.game.reset_edge()

    def handle_key(self, key):
        key = key.lower()
        label = SIM_KEY_TO_LABEL.get(key)
        if label is None:
            return
        with self._lock:
            if self.state != "running":
                return
            if label == NEUTRAL:
                self.label = NEUTRAL
            elif self.label == label:
                self.label = NEUTRAL
            else:
                self.label = label
            self.features = self._features_for_label(self.label)
            if self.state == "running":
                event = self.game.update(self.label)
                if event is not None:
                    self.game_event = event
                    self.game_event_ts = time.time()

    def _features_for_label(self, label):
        if label == LEAN:
            return {"lean": 1.0, "lift_L": 0.0, "lift_R": 0.0, "turn": 0.0}
        if label == LIFT_L:
            return {"lean": 0.0, "lift_L": 1.0, "lift_R": 0.0, "turn": 0.0}
        if label == LIFT_R:
            return {"lean": 0.0, "lift_L": 0.0, "lift_R": 1.0, "turn": 0.0}
        if label == TURN_LEFT:
            return {"lean": 0.0, "lift_L": 0.0, "lift_R": 0.0, "turn": -1.0}
        if label == TURN_RIGHT:
            return {"lean": 0.0, "lift_L": 0.0, "lift_R": 0.0, "turn": 1.0}
        return {"lean": 0.0, "lift_L": 0.0, "lift_R": 0.0, "turn": 0.0}

    def run(self):
        while not self._stop_event.is_set():
            now = time.perf_counter()
            with self._lock:
                if self.state == "calibrating":
                    elapsed = now - self._calib_start
                    self.progress = min(1.0, elapsed / CALIB_SECONDS)
                    if elapsed >= CALIB_SECONDS:
                        self.state = "running"
                        self.progress = 1.0
            self._write_state()
            time.sleep(0.02)

    def _write_state(self):
        now = time.time()
        if now - self._last_write < STATE_WRITE_INTERVAL:
            return
        self._last_write = now
        with self._lock:
            payload = {
                "state": self.state,
                "label": self.label,
                "features": dict(self.features),
                "game": self.game.snapshot(),
                "game_event": self.game_event,
                "game_event_ts": self.game_event_ts,
                "ts": now,
            }
        tmp = STATE_FILE + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(payload, f)
            os.replace(tmp, STATE_FILE)
        except OSError:
            pass

    def snapshot(self):
        with self._lock:
            return (self.state, self.progress, self.label,
                    dict(self.features), self.error)

    def game_snapshot(self):
        with self._lock:
            return self.game.snapshot(), self.game_event, self.game_event_ts

    def stop(self):
        self._stop_event.set()
        self.join(timeout=3.0)


class _IdentityMapper:
    # Adapter so ActionClassifier's gating and debounce rules run unchanged
    # on feature dicts that were computed from MuJoCo joint angles instead
    # of raw IMU channels.
    def update(self, sample):
        return sample


class MujocoMotionClassifier(threading.Thread):
    # Monitors the avatar's real-time joint angles (published by
    # run_professional_rehab.py into MOTION_FILE) and recognizes the current
    # motion with the same decision rules used on the IMU stream. This is the
    # groundwork for sensor control: the recognition pipeline is identical,
    # only the data source differs.
    def __init__(self):
        super().__init__(daemon=True)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self.state = "calibrating"       # "calibrating" | "running"
        self.progress = 0.0
        self.label = NEUTRAL
        self.features = {"lean": 0.0, "lift_L": 0.0,
                         "lift_R": 0.0, "turn": 0.0}
        self.error = None
        self.waiting = True              # no fresh MOTION_FILE data yet

        self.game = SequenceGame()
        self.game_event = None
        self.game_event_ts = 0.0

        self._classifier = ActionClassifier(_IdentityMapper())
        self._baseline = None
        self._calib_angles = []
        self._calib_start = None
        self._last_ts = 0.0
        self._last_write = 0.0

    def open(self):
        return None

    def recalibrate(self):
        with self._lock:
            self.state = "calibrating"
            self.progress = 0.0
            self.label = NEUTRAL
            self.features = {"lean": 0.0, "lift_L": 0.0,
                             "lift_R": 0.0, "turn": 0.0}
            self._classifier = ActionClassifier(_IdentityMapper())
            self._baseline = None
            self._calib_angles = []
            self._calib_start = None
            self.game.reset_edge()

    def _read_motion(self):
        try:
            with open(MOTION_FILE) as f:
                payload = json.load(f)
        except (OSError, ValueError):
            return None
        angles = payload.get("angles")
        ts = payload.get("ts", 0.0)
        if angles is None or time.time() - ts > MOTION_STALE_SECONDS:
            return None
        if ts <= self._last_ts:
            return None                  # same frame as last time
        self._last_ts = ts
        return angles

    def _features_from_angles(self, angles):
        b = self._baseline
        lean = (angles["torso_bend"] - b["torso_bend"]) / LEAN_MAX_DEG
        # The viewer lifts a leg by driving the hip joint negative.
        lift_l = -(angles["left_hip"] - b["left_hip"]) / LIFT_MAX_DEG
        lift_r = -(angles["right_hip"] - b["right_hip"]) / LIFT_MAX_DEG
        # classify.py wants positive turn = RIGHT; the avatar's +z rotation
        # turns it to ITS anatomical left, so negate.
        turn = -(angles["torso_rotation"] - b["torso_rotation"]) / TURN_MAX_DEG
        return {
            "lean": min(1.0, max(0.0, lean)),
            "lift_L": min(1.0, max(0.0, lift_l)),
            "lift_R": min(1.0, max(0.0, lift_r)),
            "turn": min(1.0, max(-1.0, turn)),
        }

    def run(self):
        while not self._stop_event.is_set():
            angles = self._read_motion()
            with self._lock:
                self.waiting = angles is None and \
                    time.time() - self._last_ts > MOTION_STALE_SECONDS

            if angles is not None:
                with self._lock:
                    state = self.state

                if state == "calibrating":
                    now = time.perf_counter()
                    if self._calib_start is None:
                        self._calib_start = now
                    self._calib_angles.append(angles)
                    elapsed = now - self._calib_start
                    done = elapsed >= CALIB_SECONDS and len(self._calib_angles) >= 20
                    with self._lock:
                        self.progress = min(1.0, elapsed / CALIB_SECONDS)
                    if done:
                        n = len(self._calib_angles)
                        baseline = {
                            k: sum(a[k] for a in self._calib_angles) / n
                            for k in self._calib_angles[0]
                        }
                        with self._lock:
                            self._baseline = baseline
                            self.state = "running"
                else:
                    out = self._classifier.update(
                        self._features_from_angles(angles))
                    with self._lock:
                        self.label = out["label"]
                        self.features = dict(out["features"])
                        event = self.game.update(out["label"])
                        if event is not None:
                            self.game_event = event
                            self.game_event_ts = time.time()

            self._write_state()
            time.sleep(0.02)

    def _write_state(self):
        now = time.time()
        if now - self._last_write < STATE_WRITE_INTERVAL:
            return
        self._last_write = now
        with self._lock:
            payload = {
                "state": self.state,
                "label": self.label,
                "features": dict(self.features),
                "game": self.game.snapshot(),
                "game_event": self.game_event,
                "game_event_ts": self.game_event_ts,
                "ts": now,
            }
        tmp = STATE_FILE + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(payload, f)
            os.replace(tmp, STATE_FILE)
        except OSError:
            pass

    def snapshot(self):
        with self._lock:
            return (self.state, self.progress, self.label,
                    dict(self.features), self.error)

    def game_snapshot(self):
        with self._lock:
            return self.game.snapshot(), self.game_event, self.game_event_ts

    def stop(self):
        self._stop_event.set()
        self.join(timeout=3.0)


class LiveClassifier(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.ser = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self.state = "calibrating"       # "calibrating" | "running"
        self.progress = 0.0
        self.label = NEUTRAL
        self.features = {"lean": 0.0, "lift_L": 0.0,
                         "lift_R": 0.0, "turn": 0.0}
        self.error = None

        self.game = SequenceGame()
        self.game_event = None
        self.game_event_ts = 0.0

        self._neutral = []
        self._calib_start = None
        self._classifier = None
        self._last_write = 0.0

    def open(self):
        if serial is None:
            raise RuntimeError("pyserial not found in THIS Python. Install: pip install pyserial")
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2.0)
        self.ser.reset_input_buffer()

    def recalibrate(self):
        with self._lock:
            self.state = "calibrating"
            self.progress = 0.0
            self.label = NEUTRAL
            self._neutral = []
            self._calib_start = None
            self._classifier = None
            self.game.reset_edge()

    def run(self):
        while not self._stop_event.is_set():
            try:
                raw = self.ser.readline()
            except Exception as exc:
                with self._lock:
                    self.error = str(exc)
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if line == "":
                continue
            _, values, ok = parse_line(line)
            if not ok or not all(k in values for k in CHANNELS):
                continue

            with self._lock:
                state = self.state

            if state == "calibrating":
                now = time.perf_counter()
                if self._calib_start is None:
                    self._calib_start = now
                self._neutral.append(values)
                elapsed = now - self._calib_start
                done = elapsed >= CALIB_SECONDS and len(self._neutral) >= 20
                with self._lock:
                    self.progress = min(1.0, elapsed / CALIB_SECONDS)
                if done:
                    cal = calibrate_neutral(self._neutral)
                    clf = ActionClassifier(MovementMapper(cal))
                    with self._lock:
                        self._classifier = clf
                        self.state = "running"
            else:
                out = self._classifier.update(values)
                cs = out["features"]
                with self._lock:
                    self.label = out["label"]
                    self.features = {
                        "lean": cs["lean"], "lift_L": cs["lift_L"],
                        "lift_R": cs["lift_R"], "turn": cs["turn"],
                    }
                    event = self.game.update(out["label"])
                    if event is not None:
                        self.game_event = event
                        self.game_event_ts = time.time()

            self._write_state()

    def _write_state(self):
        now = time.time()
        if now - self._last_write < STATE_WRITE_INTERVAL:
            return
        self._last_write = now
        with self._lock:
            payload = {
                "state": self.state,
                "label": self.label,
                "features": dict(self.features),
                "game": self.game.snapshot(),
                "game_event": self.game_event,
                "game_event_ts": self.game_event_ts,
                "ts": now,
            }
        tmp = STATE_FILE + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(payload, f)
            os.replace(tmp, STATE_FILE)
        except OSError:
            pass

    def snapshot(self):
        with self._lock:
            return (self.state, self.progress, self.label,
                    dict(self.features), self.error)

    def game_snapshot(self):
        with self._lock:
            return self.game.snapshot(), self.game_event, self.game_event_ts

    def stop(self):
        self._stop_event.set()
        self.join(timeout=3.0)
        if self.ser is not None:
            self.ser.close()


def list_ports():
    if serial is None:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("live", "simulate", "mujoco"),
                        default="simulate")
    args = parser.parse_args()

    if args.mode == "simulate":
        reader = SimulationClassifier()
    elif args.mode == "mujoco":
        reader = MujocoMotionClassifier()
    else:
        if SERIAL_PORT not in list_ports():
            print("Port", SERIAL_PORT, "not found. Available:")
            for p in list_ports():
                print("   ", p)
            print("Set SERIAL_PORT at the top (macOS:  ls /dev/cu.*).")
            sys.exit(1)

        reader = LiveClassifier(SERIAL_PORT, BAUD)
        try:
            reader.open()
        except Exception as exc:
            print("Could not open serial:", exc)
            sys.exit(1)
    reader.start()

    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore

    app = pg.mkQApp("Live action")

    class ControlWindow(QtWidgets.QWidget):
        def __init__(self, input_reader, simulation_mode):
            super().__init__()
            self._reader = input_reader
            self._simulation_mode = simulation_mode
            self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)

        def keyPressEvent(self, event):
            if self._simulation_mode:
                text = event.text().lower()
                if text in SIM_KEY_TO_LABEL:
                    self._reader.handle_key(text)
                    event.accept()
                    return
            super().keyPressEvent(event)

    win = ControlWindow(reader, args.mode == "simulate")
    if args.mode == "simulate":
        win.setWindowTitle("Movement recognition - simulation")
    elif args.mode == "mujoco":
        win.setWindowTitle("Movement recognition - MuJoCo motion")
    else:
        win.setWindowTitle("Movement recognition")
    win.resize(520, 440)
    win.setStyleSheet("background:#101216;")
    root = QtWidgets.QVBoxLayout(win)
    root.setContentsMargins(28, 28, 28, 28)
    root.setSpacing(16)

    if args.mode == "simulate":
        _status0 = "Simulation mode: hold still \u2014 readying\u2026"
    elif args.mode == "mujoco":
        _status0 = "Waiting for MuJoCo motion\u2026"
    else:
        _status0 = "Hold still \u2014 calibrating\u2026"
    status = QtWidgets.QLabel(_status0)
    status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    status.setStyleSheet("color:#9aa0a8; font-size:15px;")
    root.addWidget(status)

    if args.mode == "simulate":
        hint = QtWidgets.QLabel("Keys: Q=neutral, W=lean, E=left lift, R=right lift, T=turn left, Y=turn right")
        hint.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet("color:#7f8791; font-size:12px;")
        hint.setWordWrap(True)
        root.addWidget(hint)
    elif args.mode == "mujoco":
        hint = QtWidgets.QLabel(
            "Drive the avatar in run_professional_rehab.py (keys 1-5, one "
            "rep each) — its motion is recognized here in real time. "
            "Labels are the avatar's OWN left/right (screen shows a mirror).")
        hint.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet("color:#7f8791; font-size:12px;")
        hint.setWordWrap(True)
        root.addWidget(hint)

    big = QtWidgets.QLabel("\u2014")
    big.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    big.setStyleSheet("color:#8a8f98; font-size:46px; font-weight:700;")
    big.setWordWrap(True)
    root.addWidget(big, stretch=1)

    bars = {}
    grid = QtWidgets.QGridLayout()
    grid.setHorizontalSpacing(12)
    grid.setVerticalSpacing(10)
    for r, (key, name) in enumerate([
            ("lean", "Lean"), ("lift_L", "Lift L"),
            ("lift_R", "Lift R"), ("turn", "Turn")]):
        lab = QtWidgets.QLabel(name)
        lab.setStyleSheet("color:#cfd3d8; font-size:13px;")
        bar = QtWidgets.QProgressBar()
        bar.setRange(0, 100)
        bar.setTextVisible(False)
        bar.setFixedHeight(14)
        bar.setStyleSheet(
            "QProgressBar{background:#1c1f26; border:none; border-radius:7px;}"
            "QProgressBar::chunk{background:#3b6ea5; border-radius:7px;}")
        grid.addWidget(lab, r, 0)
        grid.addWidget(bar, r, 1)
        bars[key] = bar
    root.addLayout(grid)

    score_label = QtWidgets.QLabel("Score: 0   Rounds: 0")
    score_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    score_label.setStyleSheet("color:#e5e7eb; font-size:18px; font-weight:700;")
    root.addWidget(score_label)

    seq_label = QtWidgets.QLabel()
    seq_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    seq_label.setTextFormat(QtCore.Qt.TextFormat.RichText)
    seq_label.setStyleSheet("font-size:14px;")
    root.addWidget(seq_label)

    flash_label = QtWidgets.QLabel("")
    flash_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    flash_label.setStyleSheet("font-size:13px; font-weight:600;")
    root.addWidget(flash_label)

    if args.mode == "simulate":
        _btn_text = "Recalibrate (restart simulation)"
    elif args.mode == "mujoco":
        _btn_text = "Recalibrate (avatar at neutral)"
    else:
        _btn_text = "Recalibrate (hold still)"
    btn = QtWidgets.QPushButton(_btn_text)
    btn.setStyleSheet(
        "QPushButton{color:#cfd3d8; background:#1c1f26; border:none;"
        "padding:9px; border-radius:8px; font-size:13px;}"
        "QPushButton:hover{background:#262a33;}")
    btn.clicked.connect(reader.recalibrate)
    root.addWidget(btn)

    def update():
        state, progress, label, feats, error = reader.snapshot()
        game, event, event_ts = reader.game_snapshot()
        if error is not None:
            status.setText("Serial error: " + error)
            return
        if args.mode == "mujoco" and getattr(reader, "waiting", False):
            status.setText(
                "Waiting for MuJoCo motion — start run_professional_rehab.py")
            return
        score_label.setText(
            "Score: %d   Rounds: %d" % (game["score"], game["rounds_completed"]))
        seq_label.setText(render_sequence_html(game["sequence"], game["progress"]))
        if event is not None and time.time() - event_ts < EVENT_FLASH_SECONDS:
            text, color = EVENT_DISPLAY.get(event, (event, "#cfd3d8"))
            flash_label.setText(text)
            flash_label.setStyleSheet(
                "color:%s; font-size:13px; font-weight:600;" % color)
        else:
            flash_label.setText("")
        if state == "calibrating":
            remaining = max(0.0, CALIB_SECONDS * (1.0 - progress))
            if args.mode == "simulate":
                status.setText("Simulation mode: readying\u2026  %.1fs" % remaining)
            elif args.mode == "mujoco":
                status.setText(
                    "Keep the avatar neutral \u2014 calibrating\u2026  %.1fs" % remaining)
            else:
                status.setText(
                    "Hold still \u2014 calibrating\u2026  %.1fs" % remaining)
            big.setText("\u2014")
            big.setStyleSheet(
                "color:#8a8f98; font-size:46px; font-weight:700;")
            for b in bars.values():
                b.setValue(0)
            return
        if args.mode == "simulate":
            status.setText("Simulation mode: recognized movement")
        elif args.mode == "mujoco":
            status.setText("Recognizing MuJoCo avatar motion")
        else:
            status.setText("Recognizing movement")
        text, color = DISPLAY.get(label, (label, "#cfd3d8"))
        big.setText(text)
        big.setStyleSheet("color:%s; font-size:46px; font-weight:700;" % color)
        bars["lean"].setValue(int(100 * feats["lean"]))
        bars["lift_L"].setValue(int(100 * feats["lift_L"]))
        bars["lift_R"].setValue(int(100 * feats["lift_R"]))
        bars["turn"].setValue(int(100 * abs(feats["turn"])))

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(GUI_UPDATE_MS)

    app.aboutToQuit.connect(reader.stop)
    win.show()
    win.raise_()
    win.activateWindow()
    win.setFocus()
    pg.exec()


if __name__ == "__main__":
    main()
