# mapping.py
# Reusable core that turns IMU samples into a character control state.
# No serial, no Pygame, no plotting -- pure logic so it runs identically on a
# live stream or a replayed log/CSV.
#
# Sensor roles (validated against the labeled movement logs):
#   IMU0 lumbar -> lean (accel tilt) + turn (yaw gyro)
#   IMU3 thighL -> lift_L (accel tilt)
#   IMU4 thighR -> lift_R (accel tilt)
#
# Key principle proven by the data: the resting gravity axis is NOT fixed
# (it sat on Y in one session, X in another). So the yaw gyro axis is
# DETECTED during calibration from the neutral gravity vector, never hardcoded.
#
# Output (one dict per sample):
#   turn    float [-1, 1]   sign = direction
#   lean    float [ 0, 1]
#   lift_L  float [ 0, 1]
#   lift_R  float [ 0, 1]
#   plus boolean *_active flags with hysteresis for discrete triggers.

import json
import math
import time
from dataclasses import dataclass, field, asdict

# ---------------------------------------------------------------------------
# Shared parse contract (identical to the acquisition side)
# ---------------------------------------------------------------------------
LUMBAR = 0
THIGH_L = 3
THIGH_R = 4

CHANNELS = []
for _i in range(5):
    _s = str(_i)
    CHANNELS += ["ax" + _s, "ay" + _s, "az" +
                 _s, "gx" + _s, "gy" + _s, "gz" + _s]

# The recognition pipeline only uses the lumbar and thigh IMUs. The current
# hardware (IMU_getAccGyr_twoAddr_oneMult_fourIMU.ino) streams 4 IMUs and no
# longer sends IMU2, so completeness checks must gate on these channels only.
# Old 5-IMU logs are a superset and still pass.
REQUIRED_IMUS = (LUMBAR, THIGH_L, THIGH_R)
REQUIRED_CHANNELS = []
for _i in REQUIRED_IMUS:
    _s = str(_i)
    REQUIRED_CHANNELS += ["ax" + _s, "ay" + _s, "az" +
                          _s, "gx" + _s, "gy" + _s, "gz" + _s]


def parse_line(line):
    # Returns (t_esp_ms_or_None, values_dict, ok). Same rule as imu_live.py:
    # ok is False if any key:value token fails to parse as a float.
    t_esp = None
    values = {}
    ok = True
    for token in line.split(","):
        token = token.strip()
        if token == "":
            continue
        if ":" in token:
            key, _, raw = token.partition(":")
            try:
                values[key.strip()] = float(raw)
            except ValueError:
                ok = False
        else:
            if t_esp is None:
                try:
                    t_esp = int(float(token))
                except ValueError:
                    pass
    return t_esp, values, ok


def accel_vector(sample, imu):
    s = str(imu)
    return (sample["ax" + s], sample["ay" + s], sample["az" + s])


def gyro_vector(sample, imu):
    s = str(imu)
    return (sample["gx" + s], sample["gy" + s], sample["gz" + s])


# ---------------------------------------------------------------------------
# Small math helpers
# ---------------------------------------------------------------------------
def _norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(v):
    n = _norm(v)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def tilt_angle_deg(acc, base_unit):
    # Angle between the current gravity direction and the neutral one.
    # Mounting-agnostic: no per-axis bookkeeping, drift-free.
    if _norm(acc) < 0.05 or _norm(base_unit) < 0.5:
        # All-zero (mux/cable outage) or zero baseline: the sensor is not
        # delivering data. Report no tilt instead of a bogus 90 degrees
        # that pegs the lift at full scale.
        return 0.0
    au = _unit(acc)
    d = au[0] * base_unit[0] + au[1] * base_unit[1] + au[2] * base_unit[2]
    if d != d:
        # NaN: same story (e.g. failed init mid-stream).
        return 0.0
    return math.degrees(math.acos(_clamp(d, -1.0, 1.0)))


def _median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    m = n // 2
    if n % 2 == 1:
        return s[m]
    return 0.5 * (s[m - 1] + s[m])


def _median_vector(samples, imu):
    xs = [accel_vector(v, imu)[0] for v in samples]
    ys = [accel_vector(v, imu)[1] for v in samples]
    zs = [accel_vector(v, imu)[2] for v in samples]
    return (_median(xs), _median(ys), _median(zs))


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
@dataclass
class Calibration:
    # neutral gravity directions (unit vectors)
    base_lumbar: tuple = (1.0, 0.0, 0.0)
    base_thighL: tuple = (0.0, 0.0, 1.0)
    base_thighR: tuple = (0.0, 0.0, 1.0)

    # yaw gyro axis on the lumbar sensor: 0=gx, 1=gy, 2=gz (DETECTED)
    yaw_axis: int = 0
    yaw_sign: float = 1.0

    # full-scale (defaults from log analysis; refine with fit_full_scale)
    lean_deg: float = 50.0
    lift_deg: float = 38.0
    turn_dps: float = 25.0          # legacy rate scale (kept for old configs)
    turn_deg: float = 45.0          # full-scale INTEGRATED yaw angle

    # conditioning
    deadband_dps: float = 3.5
    ema_alpha: float = 0.3

    # hysteresis for *_active flags (physical units)
    lean_on_deg: float = 25.0
    lean_off_deg: float = 15.0
    lift_on_deg: float = 12.0
    lift_off_deg: float = 8.0
    turn_on_deg: float = 20.0       # on integrated yaw angle, like lean
    turn_off_deg: float = 12.0

    def to_dict(self):
        d = asdict(self)
        for k in ("base_lumbar", "base_thighL", "base_thighR"):
            d[k] = list(d[k])
        return d

    @classmethod
    def from_dict(cls, d):
        d = dict(d)
        for k in ("base_lumbar", "base_thighL", "base_thighR"):
            if k in d:
                d[k] = tuple(d[k])
        return cls(**d)

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def fit_full_scale(self, samples, movement, percentile=0.9):
        # Set a movement's full-scale from a max-effort recording, so the
        # output is patient-specific. movement in:
        #   "lean", "lift_L", "lift_R", "turn"
        if movement == "turn":
            vals = sorted(abs(gyro_vector(v, LUMBAR)[
                          self.yaw_axis]) for v in samples)
            if vals:
                self.turn_dps = max(
                    1.0, vals[int(percentile * (len(vals) - 1))])
            return
        imu = {"lean": LUMBAR, "lift_L": THIGH_L, "lift_R": THIGH_R}[movement]
        base = {"lean": self.base_lumbar, "lift_L": self.base_thighL,
                "lift_R": self.base_thighR}[movement]
        vals = sorted(tilt_angle_deg(accel_vector(v, imu), base)
                      for v in samples)
        peak = vals[int(percentile * (len(vals) - 1))] if vals else 0.0
        if movement == "lean":
            self.lean_deg = max(5.0, peak)
        else:
            self.lift_deg = max(5.0, peak)


def calibrate_neutral(samples):
    # samples: list of value-dicts captured while the patient holds neutral.
    # Builds baselines and auto-detects the yaw axis from the gravity vector.
    cal = Calibration()
    g_lumbar = _median_vector(samples, LUMBAR)
    cal.base_lumbar = _unit(g_lumbar)
    cal.base_thighL = _unit(_median_vector(samples, THIGH_L))
    cal.base_thighR = _unit(_median_vector(samples, THIGH_R))

    # yaw = rotation about gravity -> the gyro axis aligned with gravity
    comps = [abs(g_lumbar[0]), abs(g_lumbar[1]), abs(g_lumbar[2])]
    cal.yaw_axis = comps.index(max(comps))
    # Accel reads +1g on the axis pointing UP; turning RIGHT is clockwise
    # seen from above = a NEGATIVE rate about that up axis (right-hand
    # rule). Flip the sign so positive turn = RIGHT (classify convention).
    cal.yaw_sign = -1.0 if g_lumbar[cal.yaw_axis] >= 0 else 1.0
    return cal


# ---------------------------------------------------------------------------
# Mapper: one sample -> control state (carries EMA + hysteresis state)
# ---------------------------------------------------------------------------
class MovementMapper:
    def __init__(self, calibration):
        self.cal = calibration
        self._ema_lean = 0.0
        self._ema_liftL = 0.0
        self._ema_liftR = 0.0
        self._yaw_angle_deg = 0.0    # integrated yaw ANGLE (holds a pose)
        self._last_t = None
        self._lean_active = False
        self._liftL_active = False
        self._liftR_active = False
        self._turnL_active = False
        self._turnR_active = False

    def _ema(self, prev, new):
        a = self.cal.ema_alpha
        return a * new + (1.0 - a) * prev

    def _hyst(self, active, value, on, off):
        if active:
            return value > off
        return value > on

    def update(self, sample, dt=None):
        c = self.cal

        # dt: seconds since the previous sample, for yaw integration. Live
        # streams leave it None (wall clock); offline replay passes the log's
        # nominal sample interval so integration doesn't collapse to zero.
        if dt is None:
            now = time.monotonic()
            dt = 0.0 if self._last_t is None else min(0.25, now - self._last_t)
            self._last_t = now

        tilt_lumbar = tilt_angle_deg(
            accel_vector(sample, LUMBAR), c.base_lumbar)
        tilt_thighL = tilt_angle_deg(
            accel_vector(sample, THIGH_L), c.base_thighL)
        tilt_thighR = tilt_angle_deg(
            accel_vector(sample, THIGH_R), c.base_thighR)

        self._ema_lean = self._ema(self._ema_lean, tilt_lumbar)
        self._ema_liftL = self._ema(self._ema_liftL, tilt_thighL)
        self._ema_liftR = self._ema(self._ema_liftR, tilt_thighR)

        yaw_raw = gyro_vector(sample, LUMBAR)[c.yaw_axis] * c.yaw_sign
        if yaw_raw != yaw_raw:           # NaN: sensor not delivering data
            yaw_raw = 0.0
        yaw_eff = 0.0 if abs(yaw_raw) < c.deadband_dps else yaw_raw

        # Integrate rate -> angle so turn TRACKS THE POSE: turning left and
        # holding keeps turn at its value (like lean holds its tilt). The
        # deadband stops idle gyro drift from accumulating; clamping at
        # full-scale keeps the unwind responsive when the patient turns back.
        self._yaw_angle_deg = _clamp(
            self._yaw_angle_deg + yaw_eff * dt, -c.turn_deg, c.turn_deg)

        lean = _clamp(self._ema_lean / c.lean_deg, 0.0, 1.0)
        lift_L = _clamp(self._ema_liftL / c.lift_deg, 0.0, 1.0)
        lift_R = _clamp(self._ema_liftR / c.lift_deg, 0.0, 1.0)
        turn = _clamp(self._yaw_angle_deg / c.turn_deg, -1.0, 1.0)

        self._lean_active = self._hyst(self._lean_active, self._ema_lean,
                                       c.lean_on_deg, c.lean_off_deg)
        self._liftL_active = self._hyst(self._liftL_active, self._ema_liftL,
                                        c.lift_on_deg, c.lift_off_deg)
        self._liftR_active = self._hyst(self._liftR_active, self._ema_liftR,
                                        c.lift_on_deg, c.lift_off_deg)
        # Split turn flags, hysteresis on the held angle exactly like lean.
        # Convention: positive turn = RIGHT (matches classify.py).
        self._turnL_active = self._hyst(self._turnL_active,
                                        -self._yaw_angle_deg,
                                        c.turn_on_deg, c.turn_off_deg)
        self._turnR_active = self._hyst(self._turnR_active,
                                        self._yaw_angle_deg,
                                        c.turn_on_deg, c.turn_off_deg)

        return {
            "turn": turn,
            "lean": lean,
            "lift_L": lift_L,
            "lift_R": lift_R,
            "lean_active": self._lean_active,
            "liftL_active": self._liftL_active,
            "liftR_active": self._liftR_active,
            "turnL_active": self._turnL_active,
            "turnR_active": self._turnR_active,
        }


if __name__ == "__main__":
    # tiny self-check with a fabricated neutral sample (gravity on lumbar X)
    neutral = {k: 0.0 for k in CHANNELS}
    neutral["ax0"] = 0.99
    neutral["az3"] = 0.99
    neutral["az4"] = 0.99
    cal = calibrate_neutral([neutral] * 20)
    print("detected yaw axis (0=gx,1=gy,2=gz):",
          cal.yaw_axis, "sign", cal.yaw_sign)
    m = MovementMapper(cal)
    print("neutral ->", m.update(neutral))
