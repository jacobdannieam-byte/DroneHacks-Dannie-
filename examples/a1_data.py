"""Versioned, synthetic-only fixtures for the A1 predictor comparison."""

from dataclasses import asdict, dataclass, replace
from hashlib import sha256
import json

import numpy as np

from examples.d0_synthetic import SyntheticResponse, simulate
from examples.d1_synthetic import Episode, generate
from examples.d2_data import CHANGE_STEP, episodes
from examples.evaluate_adaptive_dynamics_d1 import nominal_episode
from refnav.dynamics import DroneState
from refnav.dynamics.history import STEP_SECONDS

TEST_SEED = 202611170
CALIBRATION_SEED = 202611270
STEPS = 225
TEST_COUNT = 16
CALIBRATION_COUNT = 24
FAULT_STEP = CHANGE_STEP + 20
FAULT_END = FAULT_STEP + 8
NOISE_STD = (0.01, 0.01, 0.01, 0.025, 0.025, 0.025, 0.015, 0.025)
LONG_ANCHORS = {
    "before_switch": 15, "at_switch": CHANGE_STEP, "after_0.4s": FAULT_STEP,
    "fault_end": FAULT_END, "after_0.7s": CHANGE_STEP + 35,
}


@dataclass(frozen=True)
class Replay:
    source_id: str
    episode: Episode
    anchors: dict
    fault: str = "clean"
    noise_seed: int | None = None

    def measurements(self):
        """Return the estimator stream, separate from the scoring truth."""
        episode = self.episode
        values = episode.states
        if self.noise_seed is not None:
            rng = np.random.default_rng(self.noise_seed)
            values = values + rng.normal(size=values.shape) * NOISE_STD
        epoch = replace(episode.reference, reference_epoch=episode.reference.reference_epoch + 1)
        return tuple(
            None if self.fault == "dropped_samples" and FAULT_STEP <= i < FAULT_END else
            DroneState(epoch if self.fault in ("epoch_reset", "late_epoch_transition") and i >= FAULT_STEP
                       else episode.reference, i * episode.dt, row[:3], row[3:6], row[6], row[7])
            for i, row in enumerate(values)
        )


def _nominal(case):
    targets = [(command.position_m, command.yaw_rad) for command in case.commands]
    response = SyntheticResponse()
    states = simulate(case.states[0], targets, dt=case.dt, response=response)
    return replace(case, states=states, response=asdict(response))


def fixtures():
    """Keep the published D2 anchor and use longer independent draws for 2 s horizons."""
    settings = {"steps": STEPS, "step_seconds": STEP_SECONDS}
    clean = generate("test", seed=TEST_SEED, count=TEST_COUNT, **settings)
    shifted = generate("shifted", seed=TEST_SEED, count=TEST_COUNT, **settings)
    switched = generate("shifted", seed=TEST_SEED + 1, count=TEST_COUNT,
                        change_step=CHANGE_STEP, **settings)
    calibration = generate("shifted", seed=CALIBRATION_SEED, count=CALIBRATION_COUNT,
                           change_step=CHANGE_STEP, **settings)
    # Independent in-range dynamics. Add a constant position/yaw target response delay.
    delayed = generate("test", seed=TEST_SEED + 3, count=TEST_COUNT,
                       position_delay_steps=6, yaw_delay_steps=9, **settings)
    nominal_inputs = generate("test", seed=TEST_SEED + 4, count=8, **settings)
    cohorts = {
        "in_range": tuple(Replay(f"test:{TEST_SEED}:{ep.identifier}", ep, LONG_ANCHORS) for ep in clean),
        "shifted": tuple(Replay(f"test:{TEST_SEED}:{ep.identifier}", ep, LONG_ANCHORS) for ep in shifted),
        "switched": tuple(Replay(f"test:{TEST_SEED + 1}:{ep.identifier}", ep, LONG_ANCHORS) for ep in switched),
        "delayed": tuple(Replay(f"test:{TEST_SEED + 3}:{ep.identifier}", ep, LONG_ANCHORS) for ep in delayed),
        "matched_nominal": tuple(Replay(f"nominal:{TEST_SEED + 4}:{ep.identifier}", _nominal(ep), LONG_ANCHORS)
                                 for ep in nominal_inputs),
    }
    for fault in ("noisy", "missing_history", "dropped_samples", "stale_history", "epoch_reset",
                  "late_epoch_transition"):
        cohorts[fault] = tuple(
            replace(replay, fault=fault, noise_seed=TEST_SEED + 1000 + i if fault == "noisy" else None)
            for i, replay in enumerate(cohorts["switched"])
        )
    for split in ("test", "shifted"):
        cohorts[f"legacy_d2_{split}"] = tuple(
            Replay(f"d2:{ep.identifier}", ep, {"after_0.4s": CHANGE_STEP + 20}) for ep in episodes(split)
        )
    cohorts["legacy_nominal"] = (Replay("d2:nominal", nominal_episode(), {"control_0.3s": 15}),)
    cohorts["equilibrium"] = (Replay("d2:equilibrium", nominal_episode(True), {"control_0.3s": 15}),)
    calibrators = tuple(Replay(f"cal:{CALIBRATION_SEED}:{ep.identifier}", ep, LONG_ANCHORS)
                        for ep in calibration)
    return cohorts, calibrators


def digest(value):
    return sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def fingerprints(episode):
    segments = [(*c.position_m, c.yaw_rad, c.effective_from_s, c.effective_until_s)
                for i, c in enumerate(episode.commands) if i == 0 or episode.commands[i - 1] != c]
    schedule = digest([episode.dt, len(episode.commands), segments])
    trace = sha256(np.asarray(episode.states, dtype="<f8").tobytes() + schedule.encode()).hexdigest()
    before = {key: value for key, value in episode.response.items() if key not in ("after", "change_time_s")}
    regimes = [digest(before)]
    if "after" in episode.response:
        regimes.append(digest(episode.response["after"]))
    return {"response": episode.response_id(), "commands": schedule, "trace": trace, "regimes": regimes}


def verify_splits(cohorts, calibrators, d1_manifest, d2_manifest):
    """Check the actual training configurations and trajectories, including both switch regimes."""
    used_for_weights = {"response": set(), "trace": set(), "regimes": set()}
    for generator, manifest in ((generate, d1_manifest), (episodes, d2_manifest)):
        for split in ("train", "validation"):
            cases = generator(split)
            if [ep.response_id() for ep in cases] != manifest["response_ids"][split]:
                raise ValueError(f"{split} fingerprints differ from the checkpoint manifest")
            for ep in cases:
                item = fingerprints(ep)
                for key in ("response", "trace"):
                    used_for_weights[key].add(item[key])
                used_for_weights["regimes"].update(item["regimes"])

    def check(item, seen, source_id):
        if (item["response"] in seen["response"] or item["trace"] in seen["trace"]
                or seen["regimes"].intersection(item["regimes"])):
            raise ValueError(f"checkpoint, calibration or test split overlap for {source_id}")

    def manifest_for(replays, forbidden):
        rows = []
        for replay in replays:
            item = fingerprints(replay.episode)
            check(item, forbidden, replay.source_id)
            rows.append({"id": replay.source_id, **item, "fault": replay.fault, "noise_seed": replay.noise_seed,
                         "anchors": replay.anchors})
        return rows

    calibration = manifest_for(calibrators, used_for_weights)
    forbidden = {key: values.copy() for key, values in used_for_weights.items()}
    for item in calibration:
        for key in ("response", "trace"):
            forbidden[key].add(item[key])
        forbidden["regimes"].update(item["regimes"])
    test = {cohort: manifest_for(replays, forbidden) for cohort, replays in cohorts.items()}
    source_traces = {}
    trace_sources = {}
    for row in (row for items in test.values() for row in items):
        if (source_traces.setdefault(row["id"], row["trace"]) != row["trace"]
                or trace_sources.setdefault(row["trace"], row["id"]) != row["id"]):
            raise ValueError("test trace reuse requires the same explicit paired source")
    return {"checkpoint_fit_and_validation_counts": {key: len(value) for key, value in used_for_weights.items()},
            "calibration": calibration, "test": test}
