def classify(e):
    required=('independent_evidence','infrastructure_healthy','estimator_healthy','safety_stop','deadline_missed','reproduced_policy_defect')
    if any(type(e.get(k)) is not bool for k in required): return 'INSUFFICIENT_EVIDENCE'
    if not e['independent_evidence']: return 'INSUFFICIENT_EVIDENCE'
    if e['safety_stop']: return 'SAFETY_STOP'
    if not e['infrastructure_healthy']: return 'SIMULATOR_ENVIRONMENT'
    if not e['estimator_healthy']: return 'SENSING_OR_ESTIMATOR'
    if e['deadline_missed']: return 'PERFORMANCE'
    if e['reproduced_policy_defect']: return 'POLICY_CANDIDATE'
    return 'INCONCLUSIVE'
