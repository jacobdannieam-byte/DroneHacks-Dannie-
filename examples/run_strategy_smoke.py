"""Run one named strategy through real H1/H0 consumers on offline ROS fixtures."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

from examples.h1_fixture import profile, setup, tick
from examples.h1_observation import HybridBagReplay
from refnav.strategies import StrategyConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--strategy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = StrategyConfig(**json.loads(args.strategy.read_text()))
    rows = []
    for positive in (True, False):
        selected = replace(profile(offset_rig=positive), strategy_config=config, reasoner=config.reasoner())
        replay = HybridBagReplay(selected, monotonic=lambda: 0.)
        setup(replay)
        tick(replay, 100, 0.)
        rows.append({'case': 'observed_volume' if positive else 'unknown_starting_volume',
                     'result': replay.diagnostic()})
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump({'strategy': config.identifier, 'identity': config.identity,
                   'scope': 'offline synthetic ROS messages; not flight', 'rows': rows}, stream, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
