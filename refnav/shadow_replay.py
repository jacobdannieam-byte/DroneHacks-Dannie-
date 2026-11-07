
"""Causal, non-commanding dynamics replay over one reviewed normalized stream."""

from dataclasses import dataclass
from math import dist, pi
from time import perf_counter

from onboard.adapters.geometry import R_ENU_NED
from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint
from refnav.dynamics.adaptive import OnlineFitDynamicsModel
from refnav.dynamics.history import MotionHistory, STEP_SECONDS
from refnav.dynamics.validation import finite_real, vector3, wrap_yaw

STEP_NS = 20_000_000
HORIZON_STEPS = 50
MAX_AGE_NS = 100_000_000


@dataclass(frozen=True)
class StreamContract:
    """Identifiers from the reviewed recorder and clock/frame mapping manifest."""

    frame_id: str
    clock_id: str
    state_source: str
    command_source: str
    mapping_id: str
    clock_origin_ns: int

    def __post_init__(self):
        for value in (self.frame_id, self.clock_id, self.state_source, self.command_source, self.mapping_id):
            if not isinstance(value, str) or not value.strip() or len(value) > 120:
                raise ValueError("invalid stream provenance identifier")
        if type(self.clock_origin_ns) is not int or self.clock_origin_ns < 0:
            raise ValueError("clock origin must be nonnegative integer nanoseconds")


def _ns(event, field):
    value = event[field]
    if type(value) is not int or not 0 <= value <= 10**15:
        raise ValueError(f"invalid {field}")
    return value


def _axes(event, key):
    position = vector3(event[key], key)
    if event["axes"] == "NED":
        return position
    if event["axes"] == "ENU":
        return tuple(R_ENU_NED @ position)
    raise ValueError("unverified world axes")


def _heading(event, key, *, rate=False):
    angle = finite_real(event[key], key)
    if event["axes"] == "ENU":
        return -angle if rate else wrap_yaw((pi / 2) - angle)
    return angle


@dataclass
class _Forecast:
    start_ns: int
    position: tuple
    yaw: float
    endpoints: dict


class ShadowReplay:
    """One pending 1 s forecast, one effective interval and bounded measured history."""

    def __init__(self, contract, *, learned=None, budget_s=.025, monotonic=perf_counter):
        if not isinstance(contract, StreamContract):
            raise ValueError("reviewed stream contract required")
        self.contract = contract
        self.history = MotionHistory()
        self.models = {"d0": DynamicsModel(), "ridge": OnlineFitDynamicsModel()}
        if learned is not None:
            self.models["learned"] = learned
        self.budget_s = finite_real(budget_s, "budget_s")
        if self.budget_s <= 0:
            raise ValueError("budget must be positive")
        self.monotonic = monotonic
        self.sequence = None
        self.received_ns = -1
        self.epoch = -1
        self.capture_ns = -1
        self.rewind = False
        self._clear()

    def _clear(self):
        self.history.clear()
        self.command = self.before = self.pending = None

    def invalidate(self):
        """Drop all partial evidence after a malformed or lost input."""
        self._clear()

    def feed(self, event, *, delivery_ns=None):
        """Consume one event in recorder receive order. Delivery uses the same mapped clock."""
        try:
            return self._feed(event, delivery_ns)
        except (KeyError, TypeError, ValueError, OverflowError):
            self._clear()
            return {"status": "invalid_event"}

    def _feed(self, event, delivery_ns):
        kind = event["kind"]
        source = {"state": self.contract.state_source, "effective": self.contract.command_source}[kind]
        if (event["source"] != source or event["mapping_id"] != self.contract.mapping_id
                or event["frame_id"] != self.contract.frame_id or event["clock_id"] != self.contract.clock_id
                or event["axes"] not in ("NED", "ENU")):
            raise ValueError("unreviewed source, clock, or reference")
        sequence, received, epoch = (_ns(event, key) for key in ("sequence", "received_ns", "epoch"))
        if self.sequence is not None and sequence <= self.sequence:
            return {"status": "out_of_order"}
        gap = self.sequence is not None and sequence != self.sequence + 1
        self.sequence = sequence
        if epoch < self.epoch:
            self._clear()
            return {"status": "old_epoch"}
        if epoch > self.epoch:
            self._clear()
            self.epoch, self.capture_ns, self.received_ns, self.rewind = epoch, -1, -1, False
        if received < self.received_ns or self.rewind:
            self._clear()
            self.rewind = True
            return {"status": "clock_rewind_requires_epoch"}
        self.received_ns = received
        if gap:
            self._clear()
        if delivery_ns is not None and (type(delivery_ns) is not int
                                        or not 0 <= delivery_ns - received <= MAX_AGE_NS):
            self._clear()
            return {"status": "delivery_age_exceeded"}
        reference = LocalNED(self.contract.frame_id, epoch, self.contract.clock_id)
        if kind == "effective":
            result = self._effective(event, reference, received)
        else:
            result = self._state(event, reference, received, delivery_ns)
        return {"sequence": sequence, "epoch": epoch, "received_ns": received, "sequence_gap": gap, **result}

    def _effective(self, event, reference, received):
        start, end = (_ns(event, key) for key in ("effective_from_ns", "effective_until_ns"))
        if event["acceptance"] != "verified_effective" or not start < end <= received:
            self._clear()
            return {"status": "unverified_effective_interval"}
        if received - end > MAX_AGE_NS:
            self._clear()
            return {"status": "stale_effective_interval"}
        if self.command is not None and start < self.command[1]:
            self._clear()
            return {"status": "overlapping_effective_interval"}
        command = PositionSetpoint(reference, start / 1e9, end / 1e9,
                                   _axes(event, "position_m"), _heading(event, "yaw_rad"))
        self.command = start, end, command
        return {"status": "effective_interval"}

    def _state(self, event, reference, received, delivery_ns):
        capture = _ns(event, "capture_ns")
        age = (received if delivery_ns is None else delivery_ns) - capture
        if capture <= self.capture_ns:
            self._clear()
            return {"status": "out_of_order_capture"}
        self.capture_ns = capture
        if capture > received or not 0 <= age <= MAX_AGE_NS or event["healthy"] is not True:
            self._clear()
            return {"status": "stale_or_unhealthy_state", "input_age_ns": age}
        if event["twist_frame"] != "world" or event["yaw_rate_kind"] != "heading_derivative":
            self._clear()
            return {"status": "unverified_velocity_or_heading_frame"}
        state = DroneState(reference, capture / 1e9, _axes(event, "position_m"), _axes(event, "velocity_mps"),
                           _heading(event, "yaw_rad"), _heading(event, "yaw_rate_rad_s", rate=True))
        previous, self.before = self.before, (capture, state)
        if previous is None:
            return {"status": "anchor", "input_age_ns": age}
        before_ns, before = previous
        if (capture - before_ns != STEP_NS or self.command is None
                or not self.command[0] <= before_ns < capture <= self.command[1]):
            self.history.clear()
            self.pending = None
            return {"status": "missing_aligned_transition", "input_age_ns": age}
        command = self.command[2]
        self.history.observe(before, command, state)
        result = {"status": "observed", "input_age_ns": age, "history": self.history.window(state).status}
        if self.pending is not None:
            forecast = self.pending
            if command.position_m != forecast.position or command.yaw_rad != forecast.yaw:
                self.pending = None
                result["comparison"] = "effective_command_changed"
            elif capture == forecast.start_ns + HORIZON_STEPS * STEP_NS:
                self.pending = None
                result["comparison"] = {
                    name: {"position_error_m": dist(pred.position_m, state.position_m),
                           "heading_error_rad": abs(wrap_yaw(pred.yaw_rad - state.yaw_rad))}
                    for name, pred in forecast.endpoints.items()
                }
        if self.pending is None and "comparison" not in result:
            elapsed, endpoints = self._predict(state, command)
            result["batch_ms"] = elapsed * 1000
            if elapsed <= self.budget_s:
                self.pending = _Forecast(capture, command.position_m,
                                         command.yaw_rad, endpoints)
                result["forecast"] = {
                    "anchor_ns": capture, "end_ns": capture + HORIZON_STEPS * STEP_NS,
                    "assumption": "counterfactual_hold_of_last_verified_position_yaw",
                    "endpoints": {name: {"position_m": pred.position_m, "yaw_rad": pred.yaw_rad}
                                  for name, pred in endpoints.items()},
                }
            else:
                result["status"] = "deadline_miss"
        return result

    def _predict(self, state, command):
        end_s = (self.capture_ns + HORIZON_STEPS * STEP_NS) / 1e9
        held = PositionSetpoint(state.reference, state.timestamp_s, end_s, command.position_m, command.yaw_rad)
        start = self.monotonic()
        endpoints = {}
        for name, model in self.models.items():
            kwargs = {} if name == "d0" else {"history": self.history}
            endpoints[name] = model.rollout(state, held, dt=STEP_SECONDS, steps=HORIZON_STEPS, **kwargs)[-1].state
        return self.monotonic() - start, endpoints
