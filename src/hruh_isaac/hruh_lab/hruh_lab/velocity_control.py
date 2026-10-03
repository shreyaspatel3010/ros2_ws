"""Simulator-independent joystick velocity limits, watchdog and fall latch."""
import math
import time


class VelocityGuard:
    """Limit motion requests while leaving the learned balance policy running.

    A fall latches motion off until a fresh neutral command arrives. Invalid or
    missing input immediately clears the requested velocity. This is a command
    filter, not a balance controller or a hardware emergency stop.
    """

    def __init__(self, timeout=0.35, clock=time.monotonic):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.timeout = timeout
        self.clock = clock
        self.limits = ((-0.2, 0.4), (-0.2, 0.2), (-0.5, 0.5))
        self.acceleration = (0.4, 0.4, 1.0)
        self.target = [0.0] * 3
        self.value = [0.0] * 3
        self.received = -math.inf
        self.latched = False

    def update(self, command):
        if len(command) != 3 or not all(math.isfinite(x) for x in command):
            self.received = -math.inf
            self.target = [0.0] * 3
            return False
        if self.latched and all(abs(x) < 0.02 for x in command):
            self.latched = False
        self.target = [max(lo, min(hi, x)) for x, (lo, hi) in zip(command, self.limits)]
        self.received = self.clock()
        return True

    def fall(self):
        self.latched = True
        self.target = [0.0] * 3
        self.value = [0.0] * 3

    def step(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        age = self.clock() - self.received
        if self.latched or age < 0 or age > self.timeout:
            self.value = [0.0] * 3
        else:
            self.value = [v + max(-a * dt, min(a * dt, target - v))
                          for v, target, a in zip(self.value, self.target, self.acceleration)]
        return tuple(self.value)
