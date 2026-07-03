# game.py
# Sequence-recognition game layered on top of classify.py's label stream.
# The player must perform a fixed series of actions, in order, to score
# points: LEAN -> TURN_LEFT -> LIFT_R -> TURN_RIGHT.
#
# Each pass through NEUTRAL before a new action counts as one discrete
# "move" (an edge trigger), so holding a position doesn't score repeatedly.
# A wrong move resets progress to the start -- unless the wrong move
# happens to match the sequence's first step, in which case it counts as
# an immediate restart rather than a lost move.

NEUTRAL = "NEUTRAL"

SEQUENCE = ["LEAN", "TURN_LEFT", "LIFT_R", "TURN_RIGHT"]

POINTS_PER_STEP = 10
BONUS_ON_COMPLETE = 50


class SequenceGame:
    def __init__(self, sequence=SEQUENCE):
        self.sequence = sequence
        self.progress = 0
        self.score = 0
        self.rounds_completed = 0
        self._prev_label = NEUTRAL

    @property
    def expected(self):
        return self.sequence[self.progress]

    def reset_edge(self):
        # Call after a recalibration so the next label isn't mistaken for
        # a same-position hold from before the classifier was reset.
        self._prev_label = NEUTRAL

    def update(self, label):
        # Returns one of: "correct", "restart", "wrong", "complete", or
        # None if this sample isn't a new move (still holding, or NEUTRAL).
        is_move = self._prev_label == NEUTRAL and label != NEUTRAL
        self._prev_label = label
        if not is_move:
            return None

        expected = self.sequence[self.progress]
        if label == expected:
            self.progress += 1
            self.score += POINTS_PER_STEP
            if self.progress == len(self.sequence):
                self.score += BONUS_ON_COMPLETE
                self.rounds_completed += 1
                self.progress = 0
                return "complete"
            return "correct"

        if label == self.sequence[0]:
            self.progress = 1
            self.score += POINTS_PER_STEP
            return "restart"

        self.progress = 0
        return "wrong"

    def snapshot(self):
        return {
            "sequence": self.sequence,
            "progress": self.progress,
            "expected": self.expected,
            "score": self.score,
            "rounds_completed": self.rounds_completed,
        }
