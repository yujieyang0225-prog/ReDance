# classify.py
# Single-label action recognizer built on mapping.py.
# Online and continuous: every sample it answers "what is she doing now?",
# choosing ONE dominant action or NEUTRAL.
#
# Actions: NEUTRAL, LEAN, LIFT_L, LIFT_R, TURN_LEFT, TURN_RIGHT
#
# Decision rule (transparent, no training; justified by the data):
#   - Accel tilt is absolute and reliable; yaw-rate is transient and noisy.
#   - So a tilt action (lean / lift) takes priority: if any tilt feature is
#     above its gate, the label is the largest tilt action.
#   - Only if no tilt action is present do we consider TURN (by yaw sign).
#   - Otherwise NEUTRAL.
# This is why turn cross-talk during a leg-lift does not cause a wrong label:
# the lift dominates and wins outright.
#
# A debounce requires a candidate to persist a few samples before it commits,
# so a single noisy window cannot flip the displayed action.

import sys
import glob
import os

from mapping import (
    REQUIRED_CHANNELS, parse_line, calibrate_neutral, MovementMapper,
)

NEUTRAL = "NEUTRAL"
LEAN = "LEAN"
LIFT_L = "LIFT_L"
LIFT_R = "LIFT_R"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"

SETTINGS_FILE = os.path.join(os.path.dirname(__file__), "patient_settings.txt")
DEFAULT_THRESHOLDS = {
    "LEAN_GATE": 0.50,
    "LIFT_L_GATE": 0.50,
    "LIFT_R_GATE": 0.50,
    "TURN_L_GATE": 0.50,
    "TURN_R_GATE": 0.50,
}


def _load_patient_thresholds(path):
    values = DEFAULT_THRESHOLDS.copy()
    try:
        with open(path, encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, raw_value = [part.strip() for part in line.split("=", 1)]
                if key not in values:
                    continue
                try:
                    values[key] = float(raw_value)
                except ValueError:
                    continue
    except OSError:
        pass
    return values


# decision thresholds on the normalized features; the GUI draws these as
# threshold lines on the feature bars
_thresholds = _load_patient_thresholds(SETTINGS_FILE)
LEAN_GATE = _thresholds["LEAN_GATE"]
LIFT_L_GATE = _thresholds["LIFT_L_GATE"]
LIFT_R_GATE = _thresholds["LIFT_R_GATE"]
TURN_L_GATE = _thresholds["TURN_L_GATE"]
TURN_R_GATE = _thresholds["TURN_R_GATE"]


class ActionClassifier:
    def __init__(self, mapper,
                 lean_gate=LEAN_GATE,
                 lift_l_gate=LIFT_L_GATE,
                 lift_r_gate=LIFT_R_GATE,
                 turn_l_gate=TURN_L_GATE,
                 turn_r_gate=TURN_R_GATE,
                 turn_presence=0.20, debounce=5):
        self.mapper = mapper
        self.lean_gate = lean_gate
        self.lift_l_gate = lift_l_gate
        self.lift_r_gate = lift_r_gate
        self.turn_l_gate = turn_l_gate
        self.turn_r_gate = turn_r_gate
        self.turn_presence = turn_presence
        self.debounce = debounce
        self._candidate = NEUTRAL
        self._count = 0
        self._committed = NEUTRAL

    def _raw_label(self, cs):
        tilts = {
            LEAN: (cs["lean"], self.lean_gate),
            LIFT_L: (cs["lift_L"], self.lift_l_gate),
            LIFT_R: (cs["lift_R"], self.lift_r_gate),
        }
        tilt_hits = [label for label, (value, gate) in tilts.items()
                     if value >= gate]
        if tilt_hits:
            return max(tilt_hits, key=lambda label: tilts[label][0])
        # TURN only if NO tilt action is even partially present: this is the
        # mutual-exclusion lockout that suppresses the yaw spike at the start
        # of a leg-lift from being read as a turn.
        max_tilt = max(value for value, _gate in tilts.values())
        if max_tilt < self.turn_presence:
            if cs["turn"] >= self.turn_r_gate:
                return TURN_RIGHT
            if -cs["turn"] >= self.turn_l_gate:
                return TURN_LEFT
        return NEUTRAL

    def update(self, sample, dt=None):
        cs = self.mapper.update(sample, dt)
        raw = self._raw_label(cs)
        if raw == self._candidate:
            self._count += 1
        else:
            self._candidate = raw
            self._count = 1
        if self._count >= self.debounce:
            self._committed = self._candidate
        return {"label": self._committed, "features": cs}


# ---------------------------------------------------------------------------
# Offline evaluation harness
# ---------------------------------------------------------------------------
def load_log(path):
    out = []
    for line in open(path, encoding="utf-8", errors="ignore"):
        line = line.strip()
        if line == "" or line.startswith("=~"):
            continue
        _, v, ok = parse_line(line)
        if ok and all(k in v for k in REQUIRED_CHANNELS):
            out.append(v)
    return out


# map a filename to its true action category (turn directions collapse to TURN)
def true_category(path):
    name = os.path.basename(path).lower()
    if "trunk_flexion" in name:
        return "LEAN"
    if "left_hip" in name:
        return "LIFT_L"
    if "right_hip" in name:
        return "LIFT_R"
    if "trunk_rotation" in name:
        return "TURN"
    return "?"


def pred_category(label):
    if label in (TURN_LEFT, TURN_RIGHT):
        return "TURN"
    return label


def evaluate(files):
    cats = ["LEAN", "LIFT_L", "LIFT_R", "TURN"]
    # confusion counted over non-NEUTRAL predictions: "when it names an action,
    # is it the right one?"  NEUTRAL fraction reported separately.
    conf = {t: {p: 0 for p in cats} for t in cats}
    neutral_frac = {}

    print("per-file detected-label counts")
    for f in files:
        s = load_log(f)
        if len(s) < 25:
            continue
        cal = calibrate_neutral(s[:20])     # neutral hold at file start
        clf = ActionClassifier(MovementMapper(cal))
        counts = {}
        n_neutral = 0
        true = true_category(f)
        for v in s:
            # offline replay runs faster than real time; pass the stream's
            # nominal sample interval so yaw integration stays correct
            lab = clf.update(v, dt=1.0 / 38.0)["label"]
            counts[lab] = counts.get(lab, 0) + 1
            if lab == NEUTRAL:
                n_neutral += 1
            else:
                conf[true][pred_category(lab)] += 1
        neutral_frac[f] = n_neutral / len(s)
        shown = {k: counts[k] for k in counts}
        print("  %-26s true=%-7s  %s" % (os.path.basename(f), true, shown))

    print("\nconfusion matrix (non-NEUTRAL samples only)")
    print("%-9s" % "true\\pred", "".join("%9s" % c for c in cats))
    correct = total = 0
    for t in cats:
        row = conf[t]
        rs = sum(row.values())
        print("%-9s" % t, "".join("%9d" % row[p] for p in cats))
        correct += row[t]
        total += rs
    acc = (100.0 * correct / total) if total else 0.0
    print("\naction-label accuracy (among detected actions): %.1f%%  (%d/%d)"
          % (acc, correct, total))
    print("mean NEUTRAL fraction per file: %.0f%%"
          % (100.0 * sum(neutral_frac.values()) / max(1, len(neutral_frac))))


if __name__ == "__main__":
    args = sys.argv[1:]
    if args:
        files = []
        for a in args:
            files += sorted(glob.glob(a))
        evaluate(files)
    else:
        print("usage: python3 classify.py <log glob>   e.g.  python3 classify.py 'logs/*.log'")
