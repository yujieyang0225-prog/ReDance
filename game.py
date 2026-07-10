# game.py
# Routine game: a choreographed warmup + dance sequence of steps.
#
# Step kinds:
#   hold  - keep a label continuously for N seconds (Baseline); the timer
#           restarts if the hold is broken, the routine does not.
#   seq   - an ORDERED list of movement labels; each discrete
#           NEUTRAL -> action edge must match the next expected label, so
#           "Tip Toes" really alternates lift_L / lift_R and "Trunk Rotate"
#           really goes left then right. Holding a pose doesn't multi-count.
#   timed - instructed free movement for N seconds (Foot Rotate, Reach-Over:
#           moves the current sensor set cannot recognize; the app shows the
#           instruction and a countdown).
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
POINTS_STEP_COMPLETE = 20
BONUS_ON_COMPLETE = 100


def _hold(name, label, seconds):
    return {"kind": "hold", "name": name, "label": label, "seconds": seconds}


def _seq(name, labels):
    return {"kind": "seq", "name": name, "labels": tuple(labels)}


def _timed(name, seconds):
    return {"kind": "timed", "name": name, "seconds": seconds}


TIPTOES = (LIFT_L, LIFT_R, LIFT_L, LIFT_R, LIFT_L, LIFT_R)

ROUTINE = [
    # ----- Warmup -----
    ("Warmup", _hold("Baseline", NEUTRAL, 10.0)),
    ("Warmup", _timed("Foot Rotate", 20.0)),
    ("Warmup", _timed("Reach-Over", 20.0)),
    # ----- Dance -----
    ("Dance", _seq("Tip Toes", TIPTOES)),
    ("Dance", _seq("Lift off R", (LIFT_R,) * 3)),
    ("Dance", _seq("Lift off L", (LIFT_L,) * 3)),
    ("Dance", _seq("Babysteps", (LIFT_L,) * 3)),
    ("Dance", _seq("Trunk Flex", (LEAN, LEAN))),
    ("Dance", _seq("Trunk Flex", (LEAN, LEAN))),
    ("Dance", _seq("Trunk Rotate", (TURN_LEFT, TURN_RIGHT))),
    ("Dance", _seq("Tip Toes", TIPTOES)),
    ("Dance", _seq("Trunk Flex", (LEAN, LEAN))),
    ("Dance", _seq("Trunk Flex", (LEAN, LEAN))),
    ("Dance", _seq("Trunk Rotate", (TURN_LEFT, TURN_RIGHT))),
    ("Dance", _seq("Babysteps", (LIFT_R,) * 3)),
]


class RoutineGame:
    def __init__(self, routine=ROUTINE):
        self.routine = routine
        self.score = 0
        self.rounds_completed = 0
        self._i = 0
        self._count = 0            # moves done in the current seq step
        self._hold_start = None
        self._timed_start = None
        self._prev_label = NEUTRAL

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
        self._hold_start = None

    def _advance(self):
        self.score += POINTS_STEP_COMPLETE
        self._i += 1
        self._count = 0
        self._hold_start = None
        self._timed_start = None
        if self._i >= len(self.routine):
            self.score += BONUS_ON_COMPLETE
            self.rounds_completed += 1
            self._i = 0
            return "routine_complete"
        return "step_complete"

    def update(self, label, features=None):
        # Call every sample/frame with the current label. Returns an event
        # string or None:
        #   "correct", "wrong", "step_complete", "routine_complete"
        step = self._step
        kind = step["kind"]
        now = time.time()
        is_move = self._prev_label == NEUTRAL and label != NEUTRAL
        self._prev_label = label

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

        if kind == "seq":
            if is_move:
                if label == step["labels"][self._count]:
                    self._count += 1
                    self.score += POINTS_PER_REP
                    if self._count >= len(step["labels"]):
                        return self._advance()
                    return "correct"
                return "wrong"
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
            total = len(step["labels"])
            nxt = step["labels"][self._count] if self._count < total else ""
            detail = "%d / %d  ·  now: %s" % (self._count, total, nxt)
            progress = self._count / float(total)
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
