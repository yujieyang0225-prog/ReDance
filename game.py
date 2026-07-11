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
POINTS_CALIB = 10
POINTS_STEP_COMPLETE = 20
BONUS_ON_COMPLETE = 100

CALIB_STEP_SECONDS = 3.0


def _hold(name, label, seconds, video=None):
    return {"kind": "hold", "name": name, "label": label,
            "seconds": seconds, "video": video}


def _seq(name, labels, video=None):
    return {"kind": "seq", "name": name, "labels": tuple(labels),
            "video": video}


def _timed(name, seconds, video=None):
    return {"kind": "timed", "name": name, "seconds": seconds,
            "video": video}


def _calib():
    # A 3 s sit-still break between dance sections. Entering it makes the
    # classifier run a REAL automatic recalibration (fresh neutral baseline,
    # same as the Recalibrate button; see the classifier threads in
    # live_action.py). Completing the break awards POINTS_CALIB as a reward.
    # No video: the instruction video pauses while the player sits still.
    return {"kind": "calib", "name": "Kalibrierung – bitte still sitzen",
            "seconds": CALIB_STEP_SECONDS, "video": None}


TIPTOES = (LIFT_L, LIFT_R, LIFT_L, LIFT_R, LIFT_L, LIFT_R)

# simple German names for the movements, shown to the patient
LABEL_TEXT_DE = {
    NEUTRAL: "Ruhehaltung",
    LEAN: "nach vorne beugen",
    LIFT_L: "linkes Bein heben",
    LIFT_R: "rechtes Bein heben",
    TURN_LEFT: "nach links drehen",
    TURN_RIGHT: "nach rechts drehen",
}

# Each step carries its own instruction-video segment (start_s, stop_s), so
# repeated step names (Tip Toes, Trunk Flex, ...) loop their OWN part of the
# video. Timestamps from the choreography video cut list.
ROUTINE = [
    # ----- Warmup -----
    ("Aufwärmen", _hold("Ruhig sitzen", NEUTRAL, 10.0, video=(0.0, 2.0))),
    ("Aufwärmen", _timed("Füße kreisen", 20.0, video=(2.0, 17.0))),
    ("Aufwärmen", _timed("Zur Seite greifen", 20.0, video=(17.0, 27.0))),
    # ----- Dance (3 s calibration reward between sections) -----
    ("Tanz", _seq("Zehenspitzen", TIPTOES, video=(27.0, 37.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Rechtes Bein heben", (LIFT_R,) * 3, video=(28.0, 29.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Linkes Bein heben", (LIFT_L,) * 3, video=(29.0, 30.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Kleine Schritte", (LIFT_L,) * 3, video=(37.0, 40.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen", (LEAN, LEAN), video=(40.0, 47.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen", (LEAN, LEAN), video=(47.0, 53.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Oberkörper drehen", (TURN_LEFT, TURN_RIGHT),
                   video=(53.0, 59.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Zehenspitzen", TIPTOES, video=(59.0, 67.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen", (LEAN, LEAN), video=(67.0, 74.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen", (LEAN, LEAN), video=(74.0, 80.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Oberkörper drehen", (TURN_LEFT, TURN_RIGHT),
                   video=(80.0, 87.0))),
    ("Tanz", _calib()),
    ("Tanz", _seq("Kleine Schritte", (LIFT_R,) * 3, video=(93.0, 100.0))),
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

    def current_kind(self):
        return self._step["kind"]

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
        if self.routine[self._i][1]["kind"] == "calib":
            # start the break clock immediately: the classifier's automatic
            # recalibration (~2 s, during which the game gets no updates)
            # runs INSIDE the 3 s window instead of stretching it
            self._timed_start = time.time()
        return "step_complete"

    def update(self, label, features=None):
        # Call every sample/frame with the current label. Returns an event
        # string or None:
        #   "correct", "calib_reward", "step_complete", "routine_complete"
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

        if kind == "calib":
            if self._timed_start is None:
                self._timed_start = now
            if now - self._timed_start >= step["seconds"]:
                # reward instead of the regular step bonus; a calib break
                # is never the final step, so no round handling needed
                self.score += POINTS_CALIB
                self._i += 1
                self._count = 0
                self._hold_start = None
                self._timed_start = None
                return "calib_reward"
            return None

        if kind == "seq":
            if is_move:
                if label == step["labels"][self._count]:
                    self._count += 1
                    self.score += POINTS_PER_REP
                    if self._count >= len(step["labels"]):
                        return self._advance()
                    return "correct"
                # a non-matching move is simply ignored -- no punishment,
                # the player keeps their progress and tries again
            return None

        return None

    def snapshot(self):
        step = self._step
        kind = step["kind"]
        now = time.time()
        if kind == "hold":
            elapsed = 0.0 if self._hold_start is None else now - self._hold_start
            detail = "halten:  %.0f / %.0f s" % (min(elapsed, step["seconds"]),
                                                 step["seconds"])
            progress = min(1.0, elapsed / step["seconds"])
        elif kind in ("timed", "calib"):
            elapsed = 0.0 if self._timed_start is None else now - self._timed_start
            detail = "noch %.0f s" % max(0.0, step["seconds"] - elapsed)
            progress = min(1.0, elapsed / step["seconds"])
        else:
            total = len(step["labels"])
            nxt = step["labels"][self._count] if self._count < total else ""
            detail = "%d / %d  ·  jetzt: %s" % (
                self._count, total, LABEL_TEXT_DE.get(nxt, nxt))
            progress = self._count / float(total)
        if self._i + 1 < len(self.routine):
            next_name = self.routine[self._i + 1][1]["name"]
        else:
            next_name = "geschafft!"
        if kind == "seq" and self._count < len(step["labels"]):
            expected_label = step["labels"][self._count]
        elif kind == "hold":
            expected_label = step["label"]
        else:
            expected_label = None
        return {
            "expected_label": expected_label,
            "video_segment": step.get("video"),
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
