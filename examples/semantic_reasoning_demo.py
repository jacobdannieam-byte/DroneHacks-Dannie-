"""Offline WorldState replay -> contexts -> independent validated non-executable goals.

python -m examples.semantic_reasoning_demo --output artifacts/reasoning --repeats 40
"""
import argparse
import json
from pathlib import Path

from examples.semantic_world_model_demo import replay
from refnav.reasoning import DeterministicReasoner, MissionGoal, WorldReference, build_context, validate_proposal
from refnav.reasoning.evaluation import benchmark, evaluate_replay


def reasoning_replay():
    world, frames = replay()
    reasoner, reference, mission = DeterministicReasoner(), WorldReference(), MissionGoal()
    result = []
    for frame in frames:
        snapshot = frame['snapshot']
        context = build_context(snapshot, mission, now=snapshot['timestamp'], reference=reference)
        proposal = reasoner.propose(context)
        validation = validate_proposal(proposal,context,now=context.context_created_at,reference=reference)
        result.append({'label':frame['label'], 'world_entity_ids':[e['entity_id'] for e in snapshot['entities']],
                       'mission':mission.to_dict(), 'context':context.to_dict(), 'proposal':proposal.to_dict(),
                       'validation':validation.to_dict()})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('artifacts/reasoning'))
    parser.add_argument('--repeats',type=int,default=40)
    args = parser.parse_args()
    if args.repeats < 2:
        parser.error('--repeats must be >=2')
    args.output.mkdir(parents=True,exist_ok=True)
    rows = reasoning_replay()
    for row in rows:
        print(json.dumps({'event':row['label'],'intent':row['proposal']['intent'],
                          'target':row['proposal']['target_entity_id'],'reasons':row['proposal']['reason_codes'],
                          'validation':row['validation']['reason_codes']}))
    for name,data in [('replay.json',rows),('evaluation.json',evaluate_replay()),('benchmark.json',benchmark(args.repeats))]:
        (args.output/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    lines = ['| World event | Proposal | Target | Validation |','| --- | --- | --- | --- |']
    for row in rows:
        lines.append(f'| {row["label"]} | {row["proposal"]["intent"]} | '
                     f'{row["proposal"]["target_entity_id"] or "none"} | '
                     f'{"valid_non_executable" if row["validation"]["valid"] else "rejected"} |')
    (args.output/'decisions.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(f'Wrote reasoning replay, evaluation, benchmark and decisions to {args.output}')


if __name__ == '__main__':
    main()
