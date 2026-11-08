"""Notion-grounded planning. Pure validation; no publication or dispatch."""
from .gates import digest
EXISTING=('S1','S0','S2','S3','S4')
DISPOSITIONS=('retain_baseline','integrate','evaluate','defer','reject')
def validate_context(context):
    if not context.get('sources'): raise ValueError('Notion context required')
    for source in context['sources']:
        if not source.get('page_id') or not source.get('retrieved_at') or not source.get('content_sha256'):
            raise ValueError('Versioned Notion source required')
        if source.get('truncated') is not False: raise ValueError('Incomplete Notion context')
    return digest(context)
def next_existing(records):
    for strategy in EXISTING:
        record=records.get(strategy,{})
        if record.get('disposition') not in DISPOSITIONS or not record.get('evidence'):
            return strategy
    return None
def validate_proposal(proposal,records,context):
    validate_context(context)
    for key in ('strategy_id','hypothesis','integration_target','baseline','acceptance_tests','rejection_criteria','notion_issue_url'):
        if not proposal.get(key): raise ValueError('Missing '+key)
    if proposal['baseline']!='S1': raise ValueError('Classical comparison required')
    if proposal['strategy_id'] not in EXISTING:
        if next_existing(records): raise ValueError('Existing strategy review takes priority')
        if not proposal.get('primary_sources'): raise ValueError('Cited research required')
    return digest(proposal)
def report_record(kind,strategy,source_sha,evidence,limitations):
    if kind not in ('research','strategy','issue','experiment','decision','integration'): raise ValueError('Unknown record kind')
    if not evidence: raise ValueError('Evidence required, including failed outcomes')
    return dict(kind=kind,strategy_id=strategy,source_sha=source_sha,evidence=evidence,limitations=limitations,publication_status='draft')
