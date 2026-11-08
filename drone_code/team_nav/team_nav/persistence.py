"""Answers one question for the safety rules: "has this condition been true, without a break,
for at least so many seconds?"

REAL DRONE CODE, also used in the tests. It is plain arithmetic, with no ROS or PX4.

Why it exists: on drone 16 single readings are often wrong for a moment (a camera report that
arrives 0.4 s late, a heading that jumps for under a second). So the rules that do something
expensive, such as landing and ending the run, wait until their condition has lasted for some
time. Cheap actions, such as holding still, do not need this and act on one reading.
run_safety.py and height_check.py use this class for every rule that waits, so they all read
the same way.

The caller always gives the time. The class never reads a clock itself, so a test bench can run
faster than real time.

What it does not do: decide what the condition is, or what to do when it has lasted.
"""


class ConditionHeldFor:
    """True once a condition has been true in every update for at least `seconds`."""

    def __init__(self, seconds):
        self.seconds = seconds
        self.became_true_at_seconds = None   # None while the condition is false.

    def update(self, condition_is_true, now_seconds):
        """Call once per cycle. Returns True once the condition has held for long enough."""
        if not condition_is_true:
            self.became_true_at_seconds = None
            return False
        if self.became_true_at_seconds is None:
            self.became_true_at_seconds = now_seconds
        return self.seconds_held(now_seconds) >= self.seconds

    def seconds_held(self, now_seconds):
        """How long the condition has been true without a break, or 0 while it is false."""
        if self.became_true_at_seconds is None:
            return 0.0
        return now_seconds - self.became_true_at_seconds

    def reset(self):
        """Forget the condition, as if it had just been false."""
        self.became_true_at_seconds = None
