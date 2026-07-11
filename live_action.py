import os
import sys
import csv
import json
import time
import datetime
import threading
import argparse

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

from mapping import (
    CHANNELS, REQUIRED_CHANNELS, parse_line, calibrate_neutral,
    calibrate_forward, calibrate_lift_left, calibrate_lift_right,
    MovementMapper,
)
from classify import (
    ActionClassifier, NEUTRAL, LEAN, LIFT_L, LIFT_R, TURN_LEFT, TURN_RIGHT,
    TILT_GATE, TURN_GATE,
)
from game import (
    RoutineGame, POINTS_PER_REP, POINTS_CALIB, POINTS_STEP_COMPLETE,
    BONUS_ON_COMPLETE,
)

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
SERIAL_PORT = "COM5"   # find yours with:  ls /dev/cu.*
BAUD = 115200
CALIB_SECONDS = 5.0                      # every calibration phase, live mode
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
    NEUTRAL:    ("\u2014 Ruhehaltung \u2014", "#8a8f98"),
    LEAN:       ("Nach vorne beugen", "#e08a2b"),
    LIFT_L:     ("Linkes Bein heben", "#27ae60"),
    LIFT_R:     ("Rechtes Bein heben", "#2980b9"),
    TURN_LEFT:  ("↺ Nach links drehen", "#8e44ad"),
    TURN_RIGHT: ("Nach rechts drehen ↻", "#16a085"),
}

# how each game event is flashed: (text, color)
EVENT_DISPLAY = {
    "correct": ("+%d" % POINTS_PER_REP, "#27ae60"),
    "calib_reward": ("Kalibriert!  +%d" % POINTS_CALIB, "#3b9ea5"),
    "step_complete": ("★ Übung geschafft!  +%d ★" % POINTS_STEP_COMPLETE,
                      "#27ae60"),
    "routine_complete": ("Glückwunsch — alles geschafft!  +%d Bonus"
                         % BONUS_ON_COMPLETE, "#f1c40f"),
}
EVENT_FLASH_SECONDS = 1.5

# Achievement sounds: short chimes synthesized to WAV on first run, so the
# repo needs no binary audio assets. event -> (filename, [(freq_hz, dur_s)])
SOUND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sounds")
EVENT_SOUNDS = {
    "correct": ("reward.wav", [(880.0, 0.09), (1174.7, 0.14)]),
    "calib_reward": ("calib.wav", [(659.3, 0.09), (880.0, 0.14)]),
    "step_complete": ("step.wav",
                      [(523.3, 0.09), (659.3, 0.09), (784.0, 0.16)]),
    "routine_complete": ("fanfare.wav",
                         [(523.3, 0.12), (659.3, 0.12), (784.0, 0.12),
                          (1046.5, 0.30)]),
}


def ensure_event_sounds():
    import wave
    import struct
    import math
    os.makedirs(SOUND_DIR, exist_ok=True)
    rate = 22050
    for fname, notes in EVENT_SOUNDS.values():
        path = os.path.join(SOUND_DIR, fname)
        if os.path.exists(path):
            continue
        frames = []
        for freq, dur in notes:
            n = int(rate * dur)
            for i in range(n):
                env = (n - i) / n                    # decay
                if i < 80:
                    env *= i / 80.0                  # attack, avoids clicks
                v = 0.45 * env * math.sin(2.0 * math.pi * freq * i / rate)
                frames.append(struct.pack("<h", int(v * 32767)))
        with wave.open(path, "w") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes(b"".join(frames))

VIDEO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "instruction_video.mp4")
# Background music: loops from game start until the window is closed.
MUSIC_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "rehab_music.mp3")
MUSIC_VOLUME = 0.35   # low enough that the achievement chimes cut through
# Video segment timestamps live in game.py's ROUTINE: each step carries its
# own (start_s, stop_s), and the panel loops whatever the current step says.

SIM_KEY_TO_LABEL = {
    "q": NEUTRAL,
    "w": LEAN,
    "e": LIFT_L,
    "r": LIFT_R,
    "t": TURN_LEFT,
    "y": TURN_RIGHT,
}

# Session data logging (modeled on the old datalogger, imu_collect.py):
# one timestamped folder per session under logs/game_sessions/, continuous
# flushes so a crash loses at most a moment of data.
LOG_BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "logs", "game_sessions")
LOG_FLUSH_EVERY = 50          # sample rows between flushes (~1.3 s live)


class SessionLogger:
    # samples.csv: one row per classifier sample -- raw IMU channels (live
    # mode; empty otherwise), normalized features, label and game context.
    # events.csv: one row per game event (correct / calib / step / routine).
    def __init__(self):
        self._lock = threading.Lock()
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.folder = os.path.join(LOG_BASE_DIR, "session_" + stamp)
        os.makedirs(self.folder, exist_ok=True)
        self._t0 = time.perf_counter()
        self._sf = open(os.path.join(self.folder, "samples.csv"),
                        "w", newline="")
        self._sw = csv.writer(self._sf)
        self._sw.writerow(["t_s"] + CHANNELS +
                          ["lean", "lift_L", "lift_R", "turn", "label",
                           "phase", "step_index", "step_name", "score"])
        self._ef = open(os.path.join(self.folder, "events.csv"),
                        "w", newline="")
        self._ew = csv.writer(self._ef)
        self._ew.writerow(["t_s", "event", "phase", "step_index",
                           "step_name", "score", "rounds"])
        self._n = 0
        self.active = True

    def log_sample(self, values, features, label, game):
        if not self.active:
            return
        row = [round(time.perf_counter() - self._t0, 4)]
        row += [(values.get(k, "") if values else "") for k in CHANNELS]
        row += [round(features.get(k, 0.0), 4)
                for k in ("lean", "lift_L", "lift_R", "turn")]
        row += [label, game["phase"], game["step_index"],
                game["step_name"], game["score"]]
        with self._lock:
            if not self.active:
                return
            self._sw.writerow(row)
            self._n += 1
            if self._n % LOG_FLUSH_EVERY == 0:
                self._sf.flush()

    def log_event(self, event, game):
        if not self.active:
            return
        with self._lock:
            if not self.active:
                return
            self._ew.writerow([round(time.perf_counter() - self._t0, 4),
                               event, game["phase"], game["step_index"],
                               game["step_name"], game["score"],
                               game["rounds_completed"]])
            self._ef.flush()

    def close(self):
        with self._lock:
            if not self.active:
                return
            self.active = False
            self._sf.flush()
            self._sf.close()
            self._ef.flush()
            self._ef.close()


def render_routine_html(game):
    # Current phase + step with live progress, and a peek at the next step.
    # This is the window's PROMINENT display, so it gets the large type.
    return (
        '<span style="color:#f1c40f; font-size:16px; font-weight:600;">%s</span>'
        '<span style="color:#5a5f68; font-size:14px;">  ·  Schritt %d/%d</span>'
        '<br><span style="color:#e5e7eb; font-size:40px; font-weight:800;">'
        '%s</span><br>'
        '<span style="color:#27ae60; font-size:24px; font-weight:700;">%s</span>'
        '<br><span style="color:#5a5f68; font-size:14px;">danach: %s</span>'
        % (game["phase"], game["step_index"], game["step_total"],
           game["step_name"], game["step_detail"], game["next_name"]))


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

        self.game = RoutineGame()
        self.game_event = None
        self.game_event_ts = 0.0

        self._calib_start = time.perf_counter()
        self._last_write = 0.0
        self._last_step_kind = None
        self.logger = None            # SessionLogger, set by the GUI

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
            # the game is fed continuously from run(), not per keypress,
            # so hold/timed routine steps progress while nothing is pressed

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
            recalib_now = False
            with self._lock:
                if self.state == "calibrating":
                    elapsed = now - self._calib_start
                    self.progress = min(1.0, elapsed / CALIB_SECONDS)
                    if elapsed >= CALIB_SECONDS:
                        self.state = "running"
                        self.progress = 1.0
                else:
                    event = self.game.update(self.label, self.features)
                    if event is not None:
                        self.game_event = event
                        self.game_event_ts = time.time()
                    kind = self.game.current_kind()
                    recalib_now = (kind == "calib"
                                   and self._last_step_kind != "calib")
                    self._last_step_kind = kind
                    lg = self.logger
                    if lg is not None:
                        lg.log_sample(None, self.features, self.label,
                                      self.game.snapshot())
            if recalib_now:
                self.recalibrate()   # takes the lock itself
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
    def update(self, sample, dt=None):
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

        self.game = RoutineGame()
        self.game_event = None
        self.game_event_ts = 0.0

        self._classifier = ActionClassifier(_IdentityMapper())
        self._baseline = None
        self._calib_angles = []
        self._calib_start = None
        self._last_ts = 0.0
        self._last_write = 0.0
        self._last_step_kind = None
        self.logger = None            # SessionLogger, set by the GUI

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

            # feed the game every cycle (not only on fresh frames) so
            # hold/timed routine steps keep progressing while still
            recalib_now = False
            with self._lock:
                if self.state == "running":
                    event = self.game.update(self.label, self.features)
                    if event is not None:
                        self.game_event = event
                        self.game_event_ts = time.time()
                    kind = self.game.current_kind()
                    recalib_now = (kind == "calib"
                                   and self._last_step_kind != "calib")
                    self._last_step_kind = kind
                    lg = self.logger
                    if lg is not None:
                        lg.log_sample(None, self.features, self.label,
                                      self.game.snapshot())
            if recalib_now:
                self.recalibrate()   # takes the lock itself

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
    # Four-phase calibration, each phase held for CALIB_SECONDS (5s):
    #   1. "neutral"     -- stand/sit still (existing baseline)
    #   2. "forward"     -- lean forward once (detects lean's true direction)
    #   3. "lift_left"   -- lift the left leg once
    #   4. "lift_right"  -- lift the right leg once
    # After phase 4, calibrate_neutral + the three direction calibrations
    # run once, and classification starts ("running").
    _PHASES = ("neutral", "forward", "lift_left", "lift_right")

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

        self.game = RoutineGame()
        self.game_event = None
        self.game_event_ts = 0.0

        self._classifier = None
        self._last_write = 0.0
        self._last_step_kind = None
        self.logger = None            # SessionLogger, set by the GUI

        # NEW: multi-phase calibration state. calib_phase is read by the
        # GUI (via getattr, no lock needed for a display string) to show
        # the right prompt for the current phase.
        self.calib_phase = self._PHASES[0]
        self._phase_samples = {p: [] for p in self._PHASES}
        self._phase_start = None

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
            self._classifier = None
            self.calib_phase = self._PHASES[0]
            self._phase_samples = {p: [] for p in self._PHASES}
            self._phase_start = None
            self.game.reset_edge()

    def _finish_calibration(self):
        # All four phases done: build the real Calibration from what was
        # recorded, then start classifying.
        cal = calibrate_neutral(self._phase_samples["neutral"])
        cal = calibrate_forward(cal, self._phase_samples["forward"])
        cal = calibrate_lift_left(cal, self._phase_samples["lift_left"])
        cal = calibrate_lift_right(cal, self._phase_samples["lift_right"])
        clf = ActionClassifier(MovementMapper(cal))
        with self._lock:
            self._classifier = clf
            self.state = "running"
            self.calib_phase = self._PHASES[0]   # reset for next recalibrate

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
            if not ok or not all(k in values for k in REQUIRED_CHANNELS):
                continue

            with self._lock:
                state = self.state

            if state == "calibrating":
                phase = self.calib_phase
                now = time.perf_counter()
                if self._phase_start is None:
                    self._phase_start = now
                self._phase_samples[phase].append(values)
                elapsed = now - self._phase_start
                done = elapsed >= CALIB_SECONDS and \
                    len(self._phase_samples[phase]) >= 20
                with self._lock:
                    self.progress = min(1.0, elapsed / CALIB_SECONDS)
                if done:
                    idx = self._PHASES.index(phase)
                    if idx + 1 < len(self._PHASES):
                        with self._lock:
                            self.calib_phase = self._PHASES[idx + 1]
                        self._phase_start = None   # starts timing on next sample
                    else:
                        self._finish_calibration()
            else:
                out = self._classifier.update(values)
                cs = out["features"]
                recalib_now = False
                with self._lock:
                    self.label = out["label"]
                    self.features = {
                        "lean": cs["lean"], "lift_L": cs["lift_L"],
                        "lift_R": cs["lift_R"], "turn": cs["turn"],
                    }
                    event = self.game.update(out["label"], self.features)
                    if event is not None:
                        self.game_event = event
                        self.game_event_ts = time.time()
                    # entering a routine calibration break: run the REAL
                    # automatic recalibration (fresh neutral baseline)
                    kind = self.game.current_kind()
                    recalib_now = (kind == "calib"
                                   and self._last_step_kind != "calib")
                    self._last_step_kind = kind
                    lg = self.logger
                    if lg is not None:
                        lg.log_sample(values, self.features, self.label,
                                      self.game.snapshot())
                if recalib_now:
                    self.recalibrate()   # takes the lock itself

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
                        default="mujoco")
    args = parser.parse_args()

    if args.mode == "simulate":
        reader = SimulationClassifier()
    elif args.mode == "mujoco":
        reader = MujocoMotionClassifier()
    else:
        # the board may still be (re-)enumerating -- USB serial ports can
        # flap for a few seconds after plugging in; wait instead of dying
        _deadline = time.time() + 15.0
        while SERIAL_PORT not in list_ports() and time.time() < _deadline:
            print("Waiting for", SERIAL_PORT, "to enumerate...")
            time.sleep(1.0)
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
    from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
    from PyQt6.QtCore import QUrl
    from PyQt6.QtMultimedia import QAudioOutput, QMediaPlayer
    from PyQt6.QtMultimediaWidgets import QVideoWidget

    app = pg.mkQApp("Live action")

    # background music: loops for the whole session, ends with the window
    music_player = None
    music_audio = None
    if os.path.exists(MUSIC_PATH):
        try:
            music_player = QMediaPlayer()
            music_audio = QAudioOutput()
            music_audio.setVolume(MUSIC_VOLUME)
            music_player.setAudioOutput(music_audio)
            music_player.setSource(QUrl.fromLocalFile(MUSIC_PATH))
            music_player.setLoops(QMediaPlayer.Loops.Infinite)
            music_player.play()
        except Exception:
            music_player = None

    # achievement sounds (synthesized on first run); disabled on any failure
    sound_effects = {}
    try:
        from PyQt6.QtMultimedia import QSoundEffect
        ensure_event_sounds()
        for ev_name, (fname, _notes) in EVENT_SOUNDS.items():
            eff = QSoundEffect()
            eff.setSource(QUrl.fromLocalFile(os.path.join(SOUND_DIR, fname)))
            eff.setVolume(0.6)
            sound_effects[ev_name] = eff
    except Exception:
        sound_effects = {}

    class ThresholdBar(QtWidgets.QProgressBar):
        # progress bar with a vertical marker at the classifier threshold:
        # the movement counts once the fill crosses the line. The line turns
        # green on the bar of the currently expected movement.
        def __init__(self, threshold):
            super().__init__()
            self._threshold = threshold
            self._active = False

        def set_active(self, active):
            if active != self._active:
                self._active = active
                self.update()

        def paintEvent(self, ev):
            super().paintEvent(ev)
            p = QtGui.QPainter(self)
            x = max(1, min(self.width() - 2,
                           int(self.width() * self._threshold)))
            pen = QtGui.QPen(
                QtGui.QColor("#2ecc71" if self._active else "#e5e7eb"))
            pen.setWidth(4 if self._active else 2)
            p.setPen(pen)
            p.drawLine(x, 1, x, self.height() - 2)
            p.end()

    class RewardFlash(QtWidgets.QLabel):
        # floating achievement text: pops at the window center, rises, fades
        def __init__(self, parent):
            super().__init__(parent)
            self.setAttribute(
                QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
            self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            self._fx = QtWidgets.QGraphicsOpacityEffect(self)
            self.setGraphicsEffect(self._fx)
            self.hide()
            self._anim = None

        def pop(self, text, color, duration_ms=1200):
            # Long pops (e.g. the 3 s calibration break) fade in fast, hold
            # bright while gently rising, then fade out -- something to
            # watch so the break doesn't feel idle.
            if self._anim is not None:
                self._anim.stop()
            self.setText(text)
            size = 30 if duration_ms >= 2000 else 26
            self.setStyleSheet("color:%s; font-size:%dpx; font-weight:800;"
                               "background:transparent;" % (color, size))
            self.adjustSize()
            x = (self.parent().width() - self.width()) // 2
            y0 = int(self.parent().height() * 0.45)
            y1 = int(self.parent().height() * 0.18)
            self._fx.setOpacity(0.0)
            self.move(x, y0)
            self.show()
            self.raise_()
            grp = QtCore.QParallelAnimationGroup(self)
            rise = QtCore.QPropertyAnimation(self, b"pos", grp)
            rise.setStartValue(QtCore.QPoint(x, y0))
            rise.setEndValue(QtCore.QPoint(x, y1))
            rise.setDuration(duration_ms)
            rise.setEasingCurve(QtCore.QEasingCurve.Type.InOutSine)
            fade = QtCore.QPropertyAnimation(self._fx, b"opacity", grp)
            fade.setDuration(duration_ms)
            fade.setKeyValueAt(0.0, 0.0)
            fade.setKeyValueAt(0.08, 1.0)   # quick fade-in
            fade.setKeyValueAt(0.75, 1.0)   # hold bright
            fade.setKeyValueAt(1.0, 0.0)    # fade out
            grp.addAnimation(rise)
            grp.addAnimation(fade)
            grp.finished.connect(self.hide)
            self._anim = grp
            grp.start()

    class InstructionVideoPlayer(QtWidgets.QWidget):
        # Loops the video segment attached to the CURRENT ROUTINE STEP
        # (game.py defines start/stop per step), so repeated step names
        # each play their own part of the choreography video.
        def __init__(self, video_path):
            super().__init__()
            self._video_path = video_path
            self._displayed_key = None
            self._segment_start_ms = 0
            self._segment_end_ms = 0

            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(8)

            title = QtWidgets.QLabel("Übungsvideo")
            title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            title.setStyleSheet("color:#e5e7eb; font-size:16px; font-weight:700;")
            layout.addWidget(title)

            self._video_widget = QVideoWidget()
            self._video_widget.setMinimumSize(420, 240)
            self._video_widget.setStyleSheet("background:#000000;")
            self._video_widget.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
            layout.addWidget(self._video_widget, stretch=1)

            self._status = QtWidgets.QLabel()
            self._status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            self._status.setWordWrap(True)
            self._status.setStyleSheet("color:#7f8791; font-size:12px;")
            layout.addWidget(self._status)

            self._player = QMediaPlayer(self)
            self._audio = QAudioOutput(self)
            self._audio.setMuted(True)
            self._player.setAudioOutput(self._audio)
            self._player.setVideoOutput(self._video_widget)
            self._player.positionChanged.connect(self._on_position_changed)
            self._player.mediaStatusChanged.connect(self._on_media_status_changed)

            if os.path.exists(self._video_path):
                self._player.setSource(QUrl.fromLocalFile(self._video_path))
                self._status.setText("Das Video folgt der aktuellen Übung.")
            else:
                self._status.setText("Videodatei nicht gefunden: " + self._video_path)

        def set_segment(self, name, segment):
            # segment: (start_s, stop_s) from the routine step, or None to
            # pause (calibration breaks)
            key = (name, segment)
            if key == self._displayed_key:
                return
            self._displayed_key = key
            if segment is None:
                self._segment_start_ms = 0
                self._segment_end_ms = 0
                self._player.pause()
                self._status.setText("Video pausiert: " + name)
                return
            start_s, end_s = segment
            self._segment_start_ms = int(start_s * 1000)
            self._segment_end_ms = int(end_s * 1000)
            if self._player.source().isEmpty():
                self._status.setText("Video nicht verfügbar")
                return
            self._status.setText("Video läuft: " + name)
            self._player.setPosition(self._segment_start_ms)
            self._player.play()

        def _on_media_status_changed(self, status):
            if status == QMediaPlayer.MediaStatus.LoadedMedia:
                self._player.setPosition(self._segment_start_ms)
                self._player.play()

        def _on_position_changed(self, position):
            if self._segment_end_ms <= self._segment_start_ms:
                return
            if position >= self._segment_end_ms:
                self._player.setPosition(self._segment_start_ms)


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
        win.setWindowTitle("Bewegungserkennung – Simulation")
    elif args.mode == "mujoco":
        win.setWindowTitle("Bewegungserkennung – MuJoCo")
    else:
        win.setWindowTitle("Bewegungserkennung")
    win.resize(1120, 560)
    win.setStyleSheet("background:#101216;")
    root = QtWidgets.QHBoxLayout(win)
    root.setContentsMargins(20, 20, 20, 20)
    root.setSpacing(18)

    left = QtWidgets.QVBoxLayout()
    left.setSpacing(16)
    root.addLayout(left, 3)

    right = QtWidgets.QVBoxLayout()
    right.setSpacing(10)
    root.addLayout(right, 2)

    video_panel = InstructionVideoPlayer(VIDEO_PATH)
    right.addWidget(video_panel, stretch=1)

    if args.mode == "simulate":
        _status0 = "Simulation: bitte warten\u2026"
    elif args.mode == "mujoco":
        _status0 = "Warte auf den MuJoCo-Avatar\u2026"
    else:
        _status0 = "Bitte still halten \u2014 Kalibrierung\u2026"
    status = QtWidgets.QLabel(_status0)
    status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    status.setStyleSheet("color:#9aa0a8; font-size:15px;")
    left.addWidget(status)

    if args.mode == "simulate":
        hint = QtWidgets.QLabel("Tasten: Q=Ruhe, W=Beugen, E=Bein links, R=Bein rechts, T=Drehen links, Y=Drehen rechts")
        hint.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet("color:#7f8791; font-size:12px;")
        hint.setWordWrap(True)
        left.addWidget(hint)
    elif args.mode == "mujoco":
        hint = QtWidgets.QLabel(
            "Avatar mit den Tasten 1-5 steuern (je eine Wiederholung) — "
            "seine Bewegung wird hier live erkannt. Links/rechts ist aus "
            "Sicht des Avatars (der Bildschirm zeigt ein Spiegelbild).")
        hint.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet("color:#7f8791; font-size:12px;")
        hint.setWordWrap(True)
        left.addWidget(hint)

    # The routine (expected sequence) and score are the PROMINENT display;
    # the currently recognized movement is the small readout below the bars.
    seq_label = QtWidgets.QLabel()
    seq_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    seq_label.setTextFormat(QtCore.Qt.TextFormat.RichText)
    seq_label.setStyleSheet("font-size:15px;")
    seq_label.setWordWrap(True)
    left.addWidget(seq_label, stretch=1)

    score_label = QtWidgets.QLabel("Punkte: 0   Runden: 0")
    score_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    score_label.setStyleSheet(
        "color:#e5e7eb; font-size:28px; font-weight:800;")
    left.addWidget(score_label)

    bars = {}
    grid = QtWidgets.QGridLayout()
    grid.setHorizontalSpacing(12)
    grid.setVerticalSpacing(10)
    BAR_THRESHOLDS = {
        "lean": TILT_GATE, "lift_L": TILT_GATE, "lift_R": TILT_GATE,
        "turn_L": TURN_GATE, "turn_R": TURN_GATE,
    }
    for r, (key, name) in enumerate([
            ("lean", "Beugen"), ("lift_L", "Bein links"),
            ("lift_R", "Bein rechts"), ("turn_L", "Drehen links"),
            ("turn_R", "Drehen rechts")]):
        lab = QtWidgets.QLabel(name)
        lab.setStyleSheet("color:#cfd3d8; font-size:13px;") #check
        bar = ThresholdBar(BAR_THRESHOLDS[key])
        bar.setRange(0, 100)
        bar.setTextVisible(False)
        bar.setFixedHeight(14)
        bar.setStyleSheet(
            "QProgressBar{background:#1c1f26; border:none; border-radius:7px;}"
            "QProgressBar::chunk{background:#3b6ea5; border-radius:7px;}")
        grid.addWidget(lab, r, 0)
        grid.addWidget(bar, r, 1)
        bars[key] = bar
    left.addLayout(grid)

    big = QtWidgets.QLabel("—")   # currently recognized movement (small)
    big.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    big.setStyleSheet("color:#8a8f98; font-size:18px; font-weight:600;")
    big.setWordWrap(True)
    left.addWidget(big)

    flash_label = QtWidgets.QLabel("")
    flash_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    flash_label.setStyleSheet("font-size:13px; font-weight:600;")
    left.addWidget(flash_label)

    if args.mode == "simulate":
        _btn_text = "Neu kalibrieren (Simulation neu starten)"
    elif args.mode == "mujoco":
        _btn_text = "Neu kalibrieren (Avatar in Ruhelage)"
    else:
        _btn_text = "Neu kalibrieren (bitte still sitzen)"
    btn = QtWidgets.QPushButton(_btn_text)
    btn.setStyleSheet(
        "QPushButton{color:#cfd3d8; background:#1c1f26; border:none;"
        "padding:9px; border-radius:8px; font-size:13px;}"
        "QPushButton:hover{background:#262a33;}")
    btn.clicked.connect(reader.recalibrate)
    left.addWidget(btn)

    # ---- session data logging (start button; auto-stop on completion) ----
    session_logger = [None]

    log_btn = QtWidgets.QPushButton("● Aufzeichnung starten")
    log_btn.setStyleSheet(
        "QPushButton{color:#cfd3d8; background:#1c1f26; border:none;"
        "padding:9px; border-radius:8px; font-size:13px;}"
        "QPushButton:hover{background:#262a33;}")
    left.addWidget(log_btn)

    log_label = QtWidgets.QLabel("")
    log_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    log_label.setStyleSheet("color:#7f8791; font-size:12px;")
    log_label.setWordWrap(True)
    left.addWidget(log_label)

    def finish_logging():
        lg = session_logger[0]
        if lg is None:
            return
        reader.logger = None
        lg.close()
        session_logger[0] = None
        log_btn.setText("● Aufzeichnung starten")
        log_label.setText("Gespeichert: " + lg.folder)
        log_label.setStyleSheet("color:#27ae60; font-size:12px;")

    def toggle_logging():
        if session_logger[0] is None:
            lg = SessionLogger()
            session_logger[0] = lg
            reader.logger = lg
            log_btn.setText("■ Aufzeichnung stoppen")
            log_label.setText("● Aufzeichnung läuft…")
            log_label.setStyleSheet("color:#e74c3c; font-size:12px;")
        else:
            finish_logging()

    log_btn.clicked.connect(toggle_logging)

    reward_flash = RewardFlash(win)
    handled_event_ts = [0.0]

    # NEW: prompt text shown for each live-mode calibration phase
    LIVE_CALIB_PROMPTS = {
        "neutral":    "Bitte ruhig sitzen \u2014 Kalibrierung l\u00e4uft\u2026",
        "forward":    "Bitte einmal nach vorne beugen \u2014 Kalibrierung\u2026",
        "lift_left":  "Bitte einmal das linke Bein heben \u2014 Kalibrierung\u2026",
        "lift_right": "Bitte einmal das rechte Bein heben \u2014 Kalibrierung\u2026",
    }

    def update():
        state, progress, label, feats, error = reader.snapshot()
        game, event, event_ts = reader.game_snapshot()
        if error is not None:
            status.setText("Sensor-Fehler: " + error)
            return
        if args.mode == "mujoco" and getattr(reader, "waiting", False):
            status.setText(
                "Warte auf den Avatar — bitte run_professional_rehab.py starten")
            return
        video_panel.set_segment(game["step_name"], game.get("video_segment"))
        score_label.setText(
            "Punkte: %d   Runden: %d" % (game["score"], game["rounds_completed"]))
        seq_label.setText(render_routine_html(game))
        # green threshold line on the bar of the currently expected movement
        expected_bar = {
            LEAN: "lean", LIFT_L: "lift_L", LIFT_R: "lift_R",
            TURN_LEFT: "turn_L", TURN_RIGHT: "turn_R",
        }.get(game.get("expected_label"))
        for bar_key, b in bars.items():
            b.set_active(bar_key == expected_bar)
        if event is not None and event_ts > handled_event_ts[0]:
            # fire achievement sound + floating animation ONCE per event
            handled_event_ts[0] = event_ts
            eff = sound_effects.get(event)
            if eff is not None:
                eff.play()
            text, color = EVENT_DISPLAY.get(event, (event, "#cfd3d8"))
            # entering a calibration break: stretch the reward animation
            # across the whole 3 s break so it never feels idle
            long_pop = game.get("step_kind") == "calib"
            reward_flash.pop(text, color, 3000 if long_pop else 1200)
            if session_logger[0] is not None:
                session_logger[0].log_event(event, game)
            if event == "routine_complete":
                # last task done: stop logging and save automatically
                finish_logging()
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
                status.setText("Simulation: bitte warten\u2026  %.1f s" % remaining)
            elif args.mode == "mujoco":
                status.setText(
                    "Avatar ruhig lassen \u2014 Kalibrierung\u2026  %.1f s" % remaining)
            else:
                # NEW: live mode has four calibration sub-phases -- show
                # the prompt for whichever one is currently active.
                phase = getattr(reader, "calib_phase", "neutral")
                prompt = LIVE_CALIB_PROMPTS.get(
                    phase, LIVE_CALIB_PROMPTS["neutral"])
                status.setText("%s  %.1f s" % (prompt, remaining))
            big.setText("\u2014")
            big.setStyleSheet(
                "color:#8a8f98; font-size:18px; font-weight:600;")
            for b in bars.values():
                b.setValue(0)
            return
        if args.mode == "simulate":
            status.setText("Simulation: Bewegung erkannt")
        elif args.mode == "mujoco":
            status.setText("Avatar-Bewegung wird erkannt")
        else:
            status.setText("Ihre Bewegung wird erkannt")
        text, color = DISPLAY.get(label, (label, "#cfd3d8"))
        big.setText(text)
        big.setStyleSheet("color:%s; font-size:18px; font-weight:600;" % color)
        bars["lean"].setValue(int(100 * feats["lean"]))
        bars["lift_L"].setValue(int(100 * feats["lift_L"]))
        bars["lift_R"].setValue(int(100 * feats["lift_R"]))
        # turn is signed (negative = left); show each direction on its own bar
        bars["turn_L"].setValue(int(100 * max(0.0, -feats["turn"])))
        bars["turn_R"].setValue(int(100 * max(0.0, feats["turn"])))

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(GUI_UPDATE_MS)

    app.aboutToQuit.connect(finish_logging)   # save if closed mid-recording
    app.aboutToQuit.connect(reader.stop)
    win.show()
    win.raise_()
    win.activateWindow()
    win.setFocus()
    pg.exec()


if __name__ == "__main__":
    main()
