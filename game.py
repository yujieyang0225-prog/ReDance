# game.py
# Routine game: a choreographed warmup + dance sequence of steps.
#
# Step kinds:
#   hold  - keep a label continuously for N seconds (Baseline); the timer
#           restarts if the hold is broken, the routine does not.
#   reps  - N repetitions of the target label(s); one rep = a discrete
#           NEUTRAL -> action edge, so holding a pose doesn't multi-count.
#   combo - N reps where trunk flex AND trunk rotation are held at the same
#           time. The classifier's single label reads LEAN for that pose
#           (tilt priority), so combos are detected on the features instead.
#   timed - instructed free movement for N seconds (Foot Rotate, Tip Toes,
#           Babysteps, Reach-Over: real moves the current sensor set cannot
#           recognize; the app shows the instruction and a countdown).
#
# A wrong move flashes a warning but does NOT reset the routine -- with a
# 15-step choreography a hard reset would be brutal. Completing all steps
# earns a bonus and starts the next round.

import time

NEUTRAL = "NEUTRAL"
LEAN = "LEAN"
LIFT_L = "LIFT_L"
LIFT_R = "LIFT_R"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"

POINTS_PER_REP = 10
POINTS_COMBO = 15
POINTS_STEP_COMPLETE = 20
BONUS_ON_COMPLETE = 100

# feature gates for the combo detector (match classify.py's magnitudes)
COMBO_LEAN_GATE = 0.45
COMBO_TURN_GATE = 0.35


def _hold(name, label, seconds):
    return {"kind": "hold", "name": name, "label": label, "seconds": seconds}


def _reps(name, labels, count):
    return {"kind": "reps", "name": name, "labels": tuple(labels),
            "count": count}


def _combo(name, count):
    return {"kind": "combo", "name": name, "count": count}


def _timed(name, seconds):
    return {"kind": "timed", "name": name, "seconds": seconds}


ROUTINE = [
    # ----- Warmup -----
    ("Warmup", _hold("Baseline", NEUTRAL, 10.0)),
    ("Warmup", _timed("Foot Rotate - 2 each side", 12.0)),
    ("Warmup", _timed("Reach-Over - 2 each side", 12.0)),
    # ----- Dance -----
    ("Dance", _timed("Tip Toes - 6 each side", 15.0)),
    ("Dance", _reps("Lift off R", (LIFT_R,), 3)),
    ("Dance", _reps("Lift off L", (LIFT_L,), 3)),
    ("Dance", _timed("Babysteps - once to open", 8.0)),
    ("Dance", _reps("Trunk Flex", (LEAN,), 2)),
    ("Dance", _combo("Trunk Flex + Trunk Rotate", 2)),
    ("Dance", _reps("Trunk Rotate", (TURN_LEFT, TURN_RIGHT), 2)),
    ("Dance", _timed("Tip Toes - 6 each side", 15.0)),
    ("Dance", _reps("Trunk Flex", (LEAN,), 2)),
    ("Dance", _combo("Trunk Flex + Trunk Rotate", 2)),
    ("Dance", _reps("Trunk Rotate", (TURN_LEFT, TURN_RIGHT), 2)),
    ("Dance", _timed("Babysteps - once to close", 8.0)),
]


class RoutineGame:
    def __init__(self, routine=ROUTINE):
        self.routine = routine
        self.score = 0
        self.rounds_completed = 0
        self._i = 0
        self._count = 0            # reps/combos done in the current step
        self._hold_start = None
        self._timed_start = None
        self._prev_label = NEUTRAL
        self._combo_armed = True

    @property
    def _step(self):
        return self.routine[self._i][1]

    @property
    def _phase(self):
        return self.routine[self._i][0]

    @property
    def expected(self):
        return self._step["name"]

    def reset_edge(self):
        # Call after a recalibration so the next label isn't mistaken for
        # a same-position hold from before the classifier was reset.
        self._prev_label = NEUTRAL
        self._combo_armed = True
        self._hold_start = None

    def _advance(self):
        self.score += POINTS_STEP_COMPLETE
        self._i += 1
        self._count = 0
        self._hold_start = None
        self._timed_start = None
        self._combo_armed = True
        if self._i >= len(self.routine):
            self.score += BONUS_ON_COMPLETE
            self.rounds_completed += 1
            self._i = 0
            return "routine_complete"
        return "step_complete"

    def update(self, label, features=None):
        # Call every sample/frame with the current label (+ features for
        # combo detection). Returns an event string or None:
        #   "correct", "combo", "wrong", "step_complete", "routine_complete"
        step = self._step
        kind = step["kind"]
        now = time.time()
        is_move = self._prev_label == NEUTRAL and label != NEUTRAL
        self._prev_label = label
        if label == NEUTRAL:
            self._combo_armed = True

        if kind == "hold":
            if label == step["label"]:
                if self._hold_start is None:
                    self._hold_start = now
                if now - self._hold_start >= step["seconds"]:
                    return self._advance()
            else:
                self._hold_start = None
            return None

        if kind == "timed":
            if self._timed_start is None:
                self._timed_start = now
            if now - self._timed_start >= step["seconds"]:
                return self._advance()
            return None

        if kind == "reps":
            if is_move:
                if label in step["labels"]:
                    self._count += 1
                    self.score += POINTS_PER_REP
                    if self._count >= step["count"]:
                        return self._advance()
                    return "correct"
                return "wrong"
            return None

        if kind == "combo":
            if features is not None and self._combo_armed:
                if (features.get("lean", 0.0) >= COMBO_LEAN_GATE and
                        abs(features.get("turn", 0.0)) >= COMBO_TURN_GATE):
                    self._combo_armed = False   # re-arms on NEUTRAL
                    self._count += 1
                    self.score += POINTS_COMBO
                    if self._count >= step["count"]:
                        return self._advance()
                    return "combo"
            return None

        return None

    def snapshot(self):
        step = self._step
        kind = step["kind"]
        now = time.time()
        if kind == "hold":
            elapsed = 0.0 if self._hold_start is None else now - self._hold_start
            detail = "hold  %.0f / %.0f s" % (min(elapsed, step["seconds"]),
                                              step["seconds"])
            progress = min(1.0, elapsed / step["seconds"])
        elif kind == "timed":
            elapsed = 0.0 if self._timed_start is None else now - self._timed_start
            detail = "%.0f s left" % max(0.0, step["seconds"] - elapsed)
            progress = min(1.0, elapsed / step["seconds"])
        else:
            detail = "%d / %d" % (self._count, step["count"])
            progress = self._count / float(step["count"])
        if self._i + 1 < len(self.routine):
            next_name = self.routine[self._i + 1][1]["name"]
        else:
            next_name = "finish!"
        return {
            "phase": self._phase,
            "step_index": self._i + 1,
            "step_total": len(self.routine),
            "step_name": step["name"],
            "step_kind": kind,
            "step_detail": detail,
            "step_progress": progress,
            "next_name": next_name,
            "expected": step["name"],
            "score": self.score,
            "rounds_completed": self.rounds_completed,
        }
