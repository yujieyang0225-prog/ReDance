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


class ActionClassifier:
    def __init__(self, mapper, tilt_gate=0.45, turn_gate=0.35,
                 turn_presence=0.20, debounce=5):
        self.mapper = mapper
        self.tilt_gate = tilt_gate
        self.turn_gate = turn_gate
        self.turn_presence = turn_presence
        self.debounce = debounce
        self._candidate = NEUTRAL
        self._count = 0
        self._committed = NEUTRAL

    def _raw_label(self, cs):
        tilts = {LEAN: cs["lean"], LIFT_L: cs["lift_L"], LIFT_R: cs["lift_R"]}
        best = max(tilts, key=lambda k: tilts[k])
        if tilts[best] >= self.tilt_gate:
            return best
        # TURN only if NO tilt action is even partially present: this is the
        # mutual-exclusion lockout that suppresses the yaw spike at the start
        # of a leg-lift from being read as a turn.
        if tilts[best] < self.turn_presence and abs(cs["turn"]) >= self.turn_gate:
            return TURN_RIGHT if cs["turn"] > 0 else TURN_LEFT
        return NEUTRAL

    def update(self, sample):
        cs = self.mapper.update(sample)
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
            lab = clf.update(v)["label"]
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
