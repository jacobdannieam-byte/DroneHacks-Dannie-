"""Exercise the H1-to-H2 read-only path with identical synthetic ROS messages.

Host timings include actual context, inference, geometry and assessment work.
Messages are fixtures, not native recordings or qualified concurrent hardware.
"""
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

from examples.h1_fixture import profile, setup, tick, MS
from examples.hybrid_shadow_loop import ShadowLoop
from refnav.reasoning.selection import ReasonerConfig


def main():
    output = Path('artifacts/goal_comparison')
    training = json.loads((output / 'models/training.json').read_text())
    model = training['seeds'][0]
    configurations = (ReasonerConfig(), ReasonerConfig('G1'),
                      ReasonerConfig('G2', model['checkpoint'], model['sha256']))
    results = {}
    for config in configurations:
        loop = ShadowLoop(replace(profile(), reasoner=config))
        driver = SimpleNamespace(profile=loop.replay.profile,
                                 feed=lambda topic, msg, stamp: loop.feed(topic, msg, stamp, stamp))
        setup(driver)
        rows = []
        for stamp in range(100, 1900, 200):
            tick(driver, stamp, 0.)
            row = loop.poll((stamp + 3) * MS, (stamp + 3) * MS)
            rows.append(row.report())
        results[config.variant] = {'rows': rows, 'timings': loop.timings.summary()}
        loop.close()
    (output / 'shadow.json').write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')
    print(json.dumps({name: {'reasons': [r['reason'] for r in result['rows']], 'timings': result['timings']}
                      for name, result in results.items()}, indent=2))


if __name__ == '__main__':
    main()
