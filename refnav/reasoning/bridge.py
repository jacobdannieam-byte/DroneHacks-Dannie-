"""Independently validate an inspection envelope before metric planning.

Reconstruct inputs from fresh state/map and reject changed identities. This is
an objective resolver only; D3 remains responsible for movement feasibility.
"""
from .inspection import candidates, make_inputs
from .selection import selection_digest


def resolve_inspection(binding, snapshot, state, alignment, now_ns):
    selection = binding.selection
    if selection is None:
        return None, 'inspection_contract_required'
    if (selection.identity != selection_digest(selection)
            or selection.proposal_id != binding.proposal.proposal_id):
        return None, 'inspection_identity_changed'
    inputs = selection.inputs
    metric = snapshot.metric
    if inputs.calibration.calibration_id != metric.calibration_id:
        return None, 'inspection_calibration_changed'
    rebuilt = make_inputs(binding.context, snapshot, state, alignment, inputs.calibration,
                          stage_id=inputs.stage_id, attempts=inputs.attempts,
                          previous_entity_id=inputs.previous_entity_id)
    if rebuilt != inputs:
        return None, 'inspection_inputs_changed'
    if (now_ns - inputs.capture_ns > 250_000_000 or now_ns < inputs.capture_ns
            or any(a.stamp_seconds > binding.context.context_created_at for a in inputs.attempts)):
        return None, 'inspection_expired_or_future'
    options = candidates(binding.context, inputs)
    selected = next((c for c in options if c.entity_id == selection.selected_entity_id
                     == binding.proposal.target_entity_id), None)
    if selected is None:
        return None, 'no_supported_inspection'
    return selected.position_metres, None
