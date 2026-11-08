"""Synthetic closed-loop C1 regression; no ROS, command endpoints or flight evidence."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import types

import numpy as np
import refnav.calibration_trial as current
from refnav.core import NAV_AUTO_LAND, NAV_OFFBOARD


def simulate(module, profile, *, drift_mps, tau_s):
    plan = module.CalibrationPlan(**profile['plan'])
    driver = module.CalibrationTrial(plan)
    position, velocity = np.zeros(3), np.zeros(3)
    target = np.zeros(3)
    armed, landed, nav = False, True, 0
    rows = []
    dt = .05
    for tick in range(2400):
        stamp = round((1. + tick * dt) * 1e9)
        obs = module.Observation(tuple(map(float, position)), tuple(map(float, velocity)), stamp, stamp, bool(armed), bool(landed), nav)
        intent = driver.step(stamp, stamp, obs, admitted=True)
        if intent is not None:
            driver.published(intent, stamp, stamp // 1000, wall_ns=stamp)
            if intent.velocity_ned_mps is not None:
                target = np.asarray(intent.velocity_ned_mps)
            if intent.command == 'offboard':
                nav = NAV_OFFBOARD
            elif intent.command == 'arm':
                armed = True
            elif intent.command == 'land':
                nav = NAV_AUTO_LAND
                target = np.array((0., 0., .3))
        rows.append(dict(time_s=stamp/1e9, phase=driver.phase, position_ned_m=position.tolist(),
                         velocity_mps=velocity.tolist(), command_velocity_mps=target.tolist()))
        if driver.phase == 'done':
            break
        # The toy bias models airborne velocity-tracking error, not lift from
        # idle motors while waiting for the first takeoff demand.
        if armed and (not landed or target[2] < 0.):
            velocity += (target + (0., 0., drift_mps) - velocity) * dt / tau_s
            position += velocity * dt
            landed = position[2] >= 0.
            if landed:
                position[2] = 0.
                velocity[2] = 0.
                if nav == NAV_AUTO_LAND:
                    armed = False
    return dict(scope='synthetic first-order velocity response with vertical bias; not native validation',
                drift_mps=drift_mps, tau_s=tau_s, result=driver.result,
                recovery_reason=driver.recovery_reason, events=driver.events,
                max_observed_speed_mps=driver.max_observed_speed_mps, rows=rows)


def historical(revision):
    name = 'refnav._historical_c1_rehearsal'
    module = types.ModuleType(name)
    module.__package__ = 'refnav'
    sys.modules[name] = module
    source = subprocess.check_output(['git', 'show', revision + ':refnav/calibration_trial.py'])
    exec(compile(source, name, 'exec'), module.__dict__)
    return module


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    profile = json.loads(Path('scenarios/calibration/c1_stop_020.json').read_text())
    previous = historical('afc0eb63b0a9be1b7ba67ec18db9cbc25a56b80c')
    rows = []
    for tau in (.20, .35, .60):
        for drift in (-.02, 0., .01, .02):
            for name, module in (('previous', previous), ('corrected', current)):
                rows.append(dict(variant=name, **simulate(module, profile, drift_mps=drift, tau_s=tau)))
    with args.output.open('x') as output:
        json.dump(dict(scope='offline synthetic paired comparison', trials=rows), output, indent=2)
    print(json.dumps([{k: row[k] for k in ('variant', 'drift_mps', 'tau_s', 'result')} for row in rows], indent=2))
    raise SystemExit(int(any(row['result'] != 'complete' for row in rows if row['variant'] == 'corrected')))
