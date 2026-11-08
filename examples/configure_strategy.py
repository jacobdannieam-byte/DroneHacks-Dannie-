"""Apply a pinned strategy to a reviewed native/replay profile, without launching."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from examples.h1_observation import HybridBagProfile
from refnav.strategies import StrategyConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--strategy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    profile = json.loads(args.profile.read_text(encoding='utf-8'))
    config = StrategyConfig(**json.loads(args.strategy.read_text(encoding='utf-8')))
    profile['strategy'] = asdict(config)
    profile['reasoner'] = asdict(config.reasoner())
    HybridBagProfile.from_dict(profile)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(profile, stream, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
