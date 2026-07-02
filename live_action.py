import os
import sys
import json
import time
import threading

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not found in THIS Python. Install: pip install pyserial")
    sys.exit(1)

from mapping import CHANNELS, parse_line, calibrate_neutral, MovementMapper
from classify import (
    ActionClassifier, NEUTRAL, LEAN, LIFT_L, LIFT_R, TURN_LEFT, TURN_RIGHT,
)

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
SERIAL_PORT = "/dev/cu.usbserial-0001"   # find yours with:  ls /dev/cu.*
BAUD = 115200
CALIB_SECONDS = 2.0                      # neutral hold at startup
GUI_UPDATE_MS = 50

# Shared state file consumed by run_professional_rehab.py's live sensor mode.
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "live_state.json")
STATE_WRITE_INTERVAL = 0.03  # seconds, throttles disk writes

# how each label is shown: (text, color)
DISPLAY = {
    NEUTRAL:    ("\u2014 neutral \u2014", "#8a8f98"),
    LEAN:       ("Lean forward",      "#e08a2b"),
    LIFT_L:     ("Lift LEFT thigh",   "#27ae60"),
    LIFT_R:     ("Lift RIGHT thigh",  "#2980b9"),
    TURN_LEFT:  ("Turn LEFT",         "#8e44ad"),
    TURN_RIGHT: ("Turn RIGHT",        "#8e44ad"),
}


class LiveClassifier(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.ser = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self.state = "calibrating"       # "calibrating" | "running"
        self.progress = 0.0
        self.label = NEUTRAL
        self.features = {"lean": 0.0, "lift_L": 0.0,
                         "lift_R": 0.0, "turn": 0.0}
        self.error = None

        self._neutral = []
        self._calib_start = None
        self._classifier = None
        self._last_write = 0.0

    def open(self):
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

    def run(self):
        while not self._stop.is_set():
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

    def stop(self):
        self._stop.set()
        self.join(timeout=3.0)
        if self.ser is not None:
            self.ser.close()


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def main():
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
    win = QtWidgets.QWidget()
    win.setWindowTitle("Movement recognition")
    win.resize(520, 440)
    win.setStyleSheet("background:#101216;")
    root = QtWidgets.QVBoxLayout(win)
    root.setContentsMargins(28, 28, 28, 28)
    root.setSpacing(16)

    status = QtWidgets.QLabel("Hold still \u2014 calibrating\u2026")
    status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    status.setStyleSheet("color:#9aa0a8; font-size:15px;")
    root.addWidget(status)

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

    btn = QtWidgets.QPushButton("Recalibrate (hold still)")
    btn.setStyleSheet(
        "QPushButton{color:#cfd3d8; background:#1c1f26; border:none;"
        "padding:9px; border-radius:8px; font-size:13px;}"
        "QPushButton:hover{background:#262a33;}")
    btn.clicked.connect(reader.recalibrate)
    root.addWidget(btn)

    def update():
        state, progress, label, feats, error = reader.snapshot()
        if error is not None:
            status.setText("Serial error: " + error)
            return
        if state == "calibrating":
            remaining = max(0.0, CALIB_SECONDS * (1.0 - progress))
            status.setText(
                "Hold still \u2014 calibrating\u2026  %.1fs" % remaining)
            big.setText("\u2014")
            big.setStyleSheet(
                "color:#8a8f98; font-size:46px; font-weight:700;")
            for b in bars.values():
                b.setValue(0)
            return
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
    pg.exec()


if __name__ == "__main__":
    main()
