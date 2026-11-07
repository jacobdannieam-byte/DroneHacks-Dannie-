"""Run the offline D0 synthetic evaluation and host benchmark.

python -m examples.adaptive_dynamics_d0_demo --output artifacts/adaptive_dynamics_d0
"""

import argparse
import hashlib
import json
import platform
import statistics
import time
import tracemalloc
from dataclasses import asdict, replace
from math import pi
from pathlib import Path

import numpy as np

from examples.d0_synthetic import SyntheticResponse, simulate
from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint

DT = 0.02
STEPS = 250
REFERENCE = LocalNED("synthetic_ned", 0, "synthetic_replay")
INITIAL = DroneState(REFERENCE, 0.0, (0.0, 0.0, -1.0), (0.0, 0.0, 0.0), 3.05, 0.0)


def schedule(*segments):
    commands = []
    for end, position, yaw in segments:
        start = len(commands)
        command = PositionSetpoint(REFERENCE, start * DT, end * DT, position, yaw)
        commands.extend([command] * (end - start))
    return commands


def state_vector(state):
    return (*state.position_m, *state.velocity_mps, state.yaw_rad, state.yaw_rate_rad_s)


def error_series(predicted, observed):
    delta = predicted - observed
    return {
        "position_m": np.linalg.norm(delta[:, :3], axis=1),
        "velocity_mps": np.linalg.norm(delta[:, 3:6], axis=1),
        "heading_rad": np.abs((delta[:, 6] + pi) % (2 * pi) - pi),
        "heading_rate_rad_s": np.abs(delta[:, 7]),
    }


def summarize_errors(errors):
    return {name: {"rmse": float(np.sqrt(np.mean(values ** 2))), "max": float(values.max()),
                   "final": float(values[-1])} for name, values in errors.items()}


def evaluate(model):
    mixed = schedule((20, INITIAL.position_m, INITIAL.yaw_rad),
                     (110, (3.2, -1.8, -1.7), -2.5), (STEPS, (-1.4, 0.6, -0.7), -0.7))
    hold = schedule((STEPS, INITIAL.position_m, INITIAL.yaw_rad))
    reversal = schedule((15, INITIAL.position_m, INITIAL.yaw_rad), (70, (5, 0, -1), 3.05),
                        (170, (-5, 0, -1), 3.05), (STEPS, INITIAL.position_m, INITIAL.yaw_rad))
    scenarios = [
        ("nominal", mixed, SyntheticResponse()),
        ("lower_acceleration", mixed, SyntheticResponse(acceleration_scale=0.55)),
        ("slower_response", mixed, SyntheticResponse(velocity_tau_s=0.65)),
        ("effective_drag", mixed, SyntheticResponse(effective_drag_per_s=0.45)),
        ("heading_delay", mixed, SyntheticResponse(yaw_delay_steps=15)),
        ("zero_tracking_error", hold, SyntheticResponse()),
        ("velocity_demand_reversal", reversal, SyntheticResponse()),
    ]
    results = []
    convergence = None
    for name, commands, response in scenarios:
        targets = [(cmd.position_m, cmd.yaw_rad) for cmd in commands]
        truth = simulate(state_vector(INITIAL), targets, dt=DT, response=response)
        rollout = model.rollout(INITIAL, commands, dt=DT)
        predicted = np.array([state_vector(step.state) for step in rollout])
        one_step = []
        for index, (sample, command) in enumerate(zip(truth[:-1], commands)):
            estimate = DroneState(REFERENCE, index * DT, sample[:3], sample[3:6], sample[6], sample[7])
            one_step.append(state_vector(model.predict(estimate, command, dt=DT).state))
        open_errors = error_series(predicted, truth[1:])
        indices = [9, 24, 49, 99, 149, 249]
        results.append({
            "scenario": name,
            "truth_parameters": {**asdict(response), "yaw_delay_s": response.yaw_delay_steps * DT},
            "setpoint_segments": [asdict(cmd) for i, cmd in enumerate(commands) if i == 0 or cmd != commands[i - 1]],
            "one_step": summarize_errors(error_series(np.array(one_step), truth[1:])),
            "open_loop": summarize_errors(open_errors),
            "open_loop_error_by_horizon": [
                {"horizon_s": (i + 1) * DT, **{key: float(values[i]) for key, values in open_errors.items()}}
                for i in indices
            ],
            "sampled_position_trace": [
                {"horizon_s": (i + 1) * DT, "predicted_ned_m": predicted[i, :3].tolist(),
                 "reference_ned_m": truth[i + 1, :3].tolist()}
                for i in range(9, STEPS, 10)
            ],
            "all_samples_finite": bool(np.isfinite(predicted).all() and np.isfinite(truth).all()),
        })
        if name == "nominal":
            finer_truth = simulate(state_vector(INITIAL), targets, dt=DT, response=response, substeps=20)
            convergence = {key: float(values.max()) for key, values in error_series(truth, finer_truth).items()}
    return {"dt_s": DT, "horizon_s": STEPS * DT, "initial_state": asdict(INITIAL),
            "synthetic_predictor_parameters": asdict(model.parameters),
            "reference_integration": "Independent RK4 at 2 ms with held setpoints",
            "nominal_reference_convergence_max_difference_vs_1_ms": convergence,
            "scenarios": results}


def timing_summary(samples):
    ordered = sorted(samples)
    return {"median_ms": statistics.median(ordered) / 1e6,
            "p95_ms": ordered[(95 * len(ordered) + 99) // 100 - 1] / 1e6,
            "worst_observed_ms": ordered[-1] / 1e6}


def time_calls(run, repeats):
    for _ in range(5):
        run()
    samples = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        run()
        samples.append(time.perf_counter_ns() - start)
    # Sample CPU time separately for at least 500 ms to reduce tick quantization.
    start_cpu = time.process_time_ns()
    start_wall = time.perf_counter_ns()
    while time.perf_counter_ns() - start_wall < 500_000_000:
        run()
    elapsed = time.perf_counter_ns() - start_wall
    cpu = time.process_time_ns() - start_cpu
    tracemalloc.start()
    try:
        result = run()
        retained, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    return {**timing_summary(samples), "samples": repeats,
            "process_cpu_percent_of_one_core": 100 * cpu / elapsed,
            "cpu_sample_wall_s": elapsed / 1e9,
            "traced_retained_bytes_including_result": retained, "traced_peak_bytes": peak}


def paced_host_sample(model, command):
    period_ns = 50_000_000
    start_cpu = time.process_time_ns()
    origin = time.perf_counter_ns()
    lateness, input_age, compute = [], [], []
    misses = 0
    for i in range(40):
        due = origin + i * period_ns
        wait = due - time.perf_counter_ns()
        if wait > 0:
            time.sleep(wait / 1e9)
        capture = time.perf_counter_ns()
        state = replace(INITIAL, timestamp_s=(capture - origin) / 1e9)
        start = time.perf_counter_ns()
        model.rollout(state, command, dt=DT, steps=50)
        finish = time.perf_counter_ns()
        lateness.append(capture - due)
        input_age.append(start - capture)
        compute.append(finish - start)
        misses += finish > due + period_ns
    return {"cycles": len(compute), "target_rate_hz": 20, "deadline_misses": misses,
            "process_cpu_percent_of_one_core": 100 * (time.process_time_ns() - start_cpu) / (finish - origin),
            "release_lateness": timing_summary(lateness), "in_process_synthetic_capture_age": timing_summary(input_age),
            "one_second_rollout": timing_summary(compute)}


def benchmark(model, repeats):
    command = PositionSetpoint(REFERENCE, 0, 10, (3.2, -1.8, -1.7), -2.5)
    alternatives = tuple(replace(command, position_m=(1 + i * 0.4, -1.8, -1.7)) for i in range(8))
    rows = []
    for steps in (10, 50, 100, 250):
        timing = time_calls(lambda: model.rollout(INITIAL, command, dt=DT, steps=steps), repeats)
        rows.append({"steps": steps, "horizon_s": steps * DT, **timing})

    def batch():
        return tuple(model.rollout(INITIAL, option, dt=DT, steps=50) for option in alternatives)

    alignment = {}
    for name, candidate in (
        ("current", command), ("not_yet_effective", replace(command, effective_from_s=0.005)),
        ("expires_inside_step", replace(command, effective_until_s=0.01)),
        ("expired", replace(command, effective_from_s=-1, effective_until_s=0)),
    ):
        try:
            model.predict(INITIAL, candidate, dt=DT)
        except ValueError as error:
            alignment[name] = {"accepted": False, "reason": str(error)}
        else:
            alignment[name] = {"accepted": True}

    root = Path(__file__).resolve().parents[1]
    sources = [*sorted((root / "refnav" / "dynamics").glob("*.py")), Path(__file__),
               root / "examples" / "d0_synthetic.py"]
    return {
        "machine": {"platform": platform.platform(), "architecture": platform.machine(),
                    "processor": platform.processor(), "python": platform.python_version(), "numpy": np.__version__},
        "method": "Five warmup calls, then wall-clock timings with perf_counter_ns. Separate CPU and allocation passes.",
        "backend": "Scalar Python on host CPU. GPU utilization telemetry was unavailable for this CPU workload.",
        "fixture_generation_in_timed_regions": False,
        "source_sha256_lf": {p.resolve().relative_to(root).as_posix(): hashlib.sha256(
            p.read_text(encoding="utf-8").encode("utf-8")).hexdigest() for p in sources},
        "single_prediction": time_calls(lambda: model.predict(INITIAL, command, dt=DT), repeats * 10),
        "rollouts": rows,
        "eight_one_second_rollouts": time_calls(batch, repeats),
        "effective_input_window_checks": alignment,
        "paced_host_sample": paced_host_sample(model, command),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/adaptive_dynamics_d0"))
    parser.add_argument("--repeats", type=int, default=200)
    args = parser.parse_args()
    if not 10 <= args.repeats <= 5000:
        parser.error("--repeats must be between 10 and 5000")
    model = DynamicsModel()
    evaluation = evaluate(model)
    metrics = benchmark(model, args.repeats)
    args.output.mkdir(parents=True, exist_ok=True)
    for name, value in (("evaluation.json", evaluation), ("benchmark.json", metrics)):
        (args.output / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    for row in evaluation["scenarios"]:
        errors = row["open_loop"]
        print(f"{row['scenario']:<25} position RMSE {errors['position_m']['rmse']:.4f} m, "
              f"heading RMSE {errors['heading_rad']['rmse']:.4f} rad")
    print(f"Single step median {metrics['single_prediction']['median_ms']:.4f} ms")
    print(f"Wrote evaluation.json and benchmark.json to {args.output}")


if __name__ == "__main__":
    main()
