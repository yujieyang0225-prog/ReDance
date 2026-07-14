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

from classify import (
    LEAN_GATE, LIFT_L_GATE, LIFT_R_GATE, TURN_L_GATE, TURN_R_GATE,
)

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

# Sequence progression gates. Keep in sync with classify.py thresholds so
# the routine uses the same movement intensity criterion as recognition.
SEQ_LEAN_GATE = LEAN_GATE
SEQ_LIFT_L_GATE = LIFT_L_GATE
SEQ_LIFT_R_GATE = LIFT_R_GATE
SEQ_TURN_L_GATE = TURN_L_GATE
SEQ_TURN_R_GATE = TURN_R_GATE


def _hold(name, label, seconds, video=None):
    return {"kind": "hold", "name": name, "label": label,
            "seconds": seconds, "video": video,
            "complete_event": "step_complete",
            "complete_points": POINTS_STEP_COMPLETE,
            "skip_event": "step_skipped", "skip_points": 0}


def _prep_hold(name, label, seconds, calibration=None, video=None):
    return {"kind": "prep", "name": name, "label": label,
            "seconds": seconds, "video": video,
            "calibration": calibration,
            "complete_event": "calib_step_complete",
            "complete_points": 0,
            "skip_event": "calib_step_skipped", "skip_points": 0}


def _seq(name, labels, video=None):
    return {"kind": "seq", "name": name, "labels": _normalize_seq_labels(labels),
            "video": video,
            "complete_event": "step_complete",
            "complete_points": POINTS_STEP_COMPLETE,
            "skip_event": "step_skipped", "skip_points": 0}


def _timed(name, seconds, video=None):
    return {"kind": "timed", "name": name, "seconds": seconds,
            "video": video,
            "complete_event": "step_complete",
            "complete_points": POINTS_STEP_COMPLETE,
            "skip_event": "step_skipped", "skip_points": 0}


def _calib():
    # A 3 s sit-still break between dance sections. Entering it makes the
    # classifier run a REAL automatic recalibration (fresh neutral baseline,
    # same as the Recalibrate button; see the classifier threads in
    # live_action.py). Completing the break awards POINTS_CALIB as a reward.
    # No video: the instruction video pauses while the player sits still.
    return {"kind": "calib", "name": "Kalibrierung – bitte still sitzen",
            "seconds": CALIB_STEP_SECONDS, "video": None}


def _normalize_seq_labels(labels):
    labels = tuple(labels)
    if not labels:
        return tuple()
    if isinstance(labels[0], str):
        return (tuple(labels),)
    normalized = []
    for rep in labels:
        if isinstance(rep, str):
            normalized.append((rep,))
        else:
            normalized.append(tuple(rep))
    return tuple(normalized)


def _seq_signal_for_label(label, features):
    # Convert expected seq label to a scalar signal and its gate.
    # Extra simultaneous features are intentionally ignored.
    features = features or {}
    if label == LEAN:
        return max(0.0, float(features.get("lean", 0.0))), SEQ_LEAN_GATE
    if label == LIFT_L:
        return max(0.0, float(features.get("lift_L", 0.0))), SEQ_LIFT_L_GATE
    if label == LIFT_R:
        return max(0.0, float(features.get("lift_R", 0.0))), SEQ_LIFT_R_GATE
    turn = float(features.get("turn", 0.0))
    if label == TURN_LEFT:
        return max(0.0, -turn), SEQ_TURN_L_GATE
    if label == TURN_RIGHT:
        return max(0.0, turn), SEQ_TURN_R_GATE
    return 0.0, 1.0


def _seq_group_ready(labels, features):
    labels = labels if isinstance(labels, tuple) else (labels,)
    for label in labels:
        signal, gate = _seq_signal_for_label(label, features)
        if signal < gate:
            return False
    return True


def _seq_group_released(labels, features):
    labels = labels if isinstance(labels, tuple) else (labels,)
    for label in labels:
        signal, gate = _seq_signal_for_label(label, features)
        if signal >= gate:
            return False
    return True

video_folder = "easy_videos/"

# simple German names for the movements, shown to the patient
LABEL_TEXT_DE = {
    NEUTRAL: "Ruhehaltung",
    LEAN: "nach vorne beugen",
    LIFT_L: "linkes Bein heben",
    LIFT_R: "rechtes Bein heben",
    TURN_LEFT: "nach links drehen",
    TURN_RIGHT: "nach rechts drehen",
}

OPENING_PRELUDE = [
    _prep_hold("Nach vorne beugen", LEAN, 1.0, calibration="forward",
               video="lean.mp4"),
    _prep_hold("Linkes Bein heben", LIFT_L, 1.0, calibration="lift_L",
               video="tiptoe_left.mp4"),
    _prep_hold("Rechtes Bein heben", LIFT_R, 1.0, calibration="lift_R",
               video="tiptoe_right.mp4"),
    _prep_hold("Nach links drehen", TURN_LEFT, 1.0, video="turn_left.mp4"),
    _prep_hold("Nach rechts drehen", TURN_RIGHT, 1.0, video="turn_right.mp4"),
]

# Each step carries its own instruction-video segment (start_s, stop_s), so
# repeated step names (Tip Toes, Trunk Flex, ...) loop their OWN part of the
# video. Timestamps from the choreography video cut list.
ROUTINE = [
    # ----- Opening calibration / walkthrough -----
    ("Kalibrierung", OPENING_PRELUDE[0]),
    ("Kalibrierung", OPENING_PRELUDE[1]),
    ("Kalibrierung", OPENING_PRELUDE[2]),
    ("Kalibrierung", OPENING_PRELUDE[3]),
    ("Kalibrierung", OPENING_PRELUDE[4]),
    # ----- Warmup -----
    ("Aufwärmen", _timed("Rechtes Füße kreisen", 16.0, video="rotate_right.mp4")),
    ("Aufwärmen", _timed("Linkes Füße kreisen", 16.0, video="rotate_left.mp4")),
    # ----- Dance (3 s calibration reward between sections) -----
    ("Tanz", _seq("Rechtes Bein heben", ((LIFT_R,),) * 3, video="tiptoe_right.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Linkes Bein heben", ((LIFT_L,),) * 3, video="tiptoe_left.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Kleine Schritte", ((LIFT_L, LIFT_R,),) * 3, video="baby_walk.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen", ((LEAN,),) * 2, video="lean.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen rechts", ((LEAN, TURN_RIGHT,),), video="lean_right.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen links", ((LEAN, TURN_LEFT,),), video="lean_left.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Oberkörper drehen rechts", ((TURN_LEFT,), (TURN_RIGHT,),), video="turn_right.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Oberkörper drehen links", ((TURN_LEFT,), (TURN_RIGHT,),), video="turn_left.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Rechtes Bein heben", ((LIFT_R,),) * 3, video="tiptoe_right.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Linkes Bein heben", ((LIFT_L,),) * 3, video="tiptoe_left.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen", ((LEAN,),) * 2, video="lean.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen rechts", ((LEAN, TURN_RIGHT,),), video="lean_right.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Nach vorne beugen links", ((LEAN, TURN_LEFT,),), video="lean_left.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Oberkörper drehen rechts", ((TURN_LEFT,), (TURN_RIGHT,),), video="turn_right.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Oberkörper drehen links", ((TURN_LEFT,), (TURN_RIGHT,),), video="turn_left.mp4")),
    ("Tanz", _calib()),
    ("Tanz", _seq("Kleine Schritte", ((LIFT_L, LIFT_R,),) * 3, video="baby_walk.mp4")),
]


class RoutineGame:
    def __init__(self, routine=ROUTINE):
        self.routine = routine
        self.reset()

    def reset(self, routine=None):
        if routine is not None:
            self.routine = routine
        self.score = 0
        self.rounds_completed = 0
        self._i = 0
        self._count = 0            # moves done in the current seq step
        self._hold_start = None
        self._timed_start = None
        self._prev_label = NEUTRAL
        self._seq_trigger_armed = True
        self._last_event = None

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

    def current_step(self):
        return self._step

    def reset_edge(self):
        # Call after a recalibration so the next label isn't mistaken for
        # a same-position hold from before the classifier was reset.
        self._prev_label = NEUTRAL
        self._hold_start = None
        self._seq_trigger_armed = True

    def _finish_step(self, event_name, points):
        step = self._step
        phase = self._phase
        step_index = self._i + 1
        self.score += points
        self._last_event = {
            "event": event_name,
            "points": points,
            "phase": phase,
            "step_index": step_index,
            "step_name": step["name"],
            "step_kind": step["kind"],
            "step": step,
        }
        self._i += 1
        self._count = 0
        self._hold_start = None
        self._timed_start = None
        self._seq_trigger_armed = True
        if self._i >= len(self.routine):
            self.score += BONUS_ON_COMPLETE
            self.rounds_completed += 1
            self._i = 0
            self._last_event = {
                "event": "routine_complete",
                "points": BONUS_ON_COMPLETE,
                "phase": phase,
                "step_index": len(self.routine),
                "step_name": step["name"],
                "step_kind": step["kind"],
                "step": step,
            }
            return self._last_event
        if self.routine[self._i][1]["kind"] == "calib":
            # start the break clock immediately: the classifier's automatic
            # recalibration (~2 s, during which the game gets no updates)
            # runs INSIDE the 3 s window instead of stretching it
            self._timed_start = time.time()
        return self._last_event

    def _advance(self):
        return self._finish_step(
            self._step.get("complete_event", "step_complete"),
            self._step.get("complete_points", POINTS_STEP_COMPLETE))

    def skip_current(self):
        return self._finish_step(
            self._step.get("skip_event", "step_skipped"),
            self._step.get("skip_points", 0))

    def update(self, label, features=None):
        # Call every sample/frame with the current label. Returns an event
        # string or None:
        #   "correct", "calib_reward", "step_complete", "step_skipped",
        #   "calib_step_complete", "calib_step_skipped",
        #   "routine_complete"
        step = self._step
        kind = step["kind"]
        now = time.time()
        self._prev_label = label

        if kind in ("hold", "prep"):
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
                return self._finish_step("calib_reward", POINTS_CALIB)
            return None

        if kind == "seq":
            expected = step["labels"][self._count]
            if not self._seq_trigger_armed:
                if _seq_group_released(expected, features):
                    self._seq_trigger_armed = True
                return None
            if not _seq_group_ready(expected, features):
                self._seq_trigger_armed = True
                return None
            if self._seq_trigger_armed:
                self._seq_trigger_armed = False
                self._count += 1
                self.score += POINTS_PER_REP
                if self._count >= len(step["labels"]):
                    return self._advance()
                self._last_event = {
                    "event": "correct",
                    "points": POINTS_PER_REP,
                    "phase": self._phase,
                    "step_index": self._i + 1,
                    "step_name": step["name"],
                    "step_kind": step["kind"],
                    "step": step,
                }
                return self._last_event
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
        elif kind in ("timed", "calib", "prep"):
            elapsed = 0.0 if self._timed_start is None else now - self._timed_start
            detail = "noch %.0f s" % max(0.0, step["seconds"] - elapsed)
            progress = min(1.0, elapsed / step["seconds"])
        else:
            total = len(step["labels"])
            detail = "%d / %d" % (self._count, total)
            progress = self._count / float(total)
        if self._i + 1 < len(self.routine):
            next_name = self.routine[self._i + 1][1]["name"]
        else:
            next_name = "geschafft!"
        if kind == "seq" and self._count < len(step["labels"]):
            expected_labels = step["labels"][self._count]
            expected_label = expected_labels[0] if len(expected_labels) == 1 else expected_labels
        elif kind in ("hold", "prep"):
            expected_label = step["label"]
            expected_labels = (step["label"],)
        else:
            expected_label = None
            expected_labels = tuple()
        return {
            "expected_label": expected_label,
            "expected_labels": expected_labels,
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
            "last_event": self._last_event,
        }
