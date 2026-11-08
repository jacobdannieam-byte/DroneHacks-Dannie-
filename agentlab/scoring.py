import math

def performance(samples):
    if not samples or any(type(x) not in (int,float) or not math.isfinite(x) or x<0 for x in samples):
        raise ValueError('Finite nonnegative measurements required')
    values=sorted(samples)
    def pct(p):
        i=(len(values)-1)*p; a=int(i); b=min(a+1,len(values)-1)
        return values[a]+(values[b]-values[a])*(i-a)
    return dict(p50=pct(.5),p95=pct(.95),p99=pct(.99),worst=values[-1],count=len(values))

def score(e):
    required=('independent_referee','telemetry_verified','landed','disarmed','landed_status_fresh')
    if any(e.get(k) is not True for k in required):
        return dict(status='UNSCORED',official_score=None,reason='Missing independent outcome evidence')
    for k in ('seconds','touches','checkpoints'):
        x=e.get(k)
        if type(x) not in (int,float) or not math.isfinite(x) or x<0:
            return dict(status='UNSCORED',official_score=None,reason='Invalid measurement')
    return dict(status='ENGINEERING_ONLY',official_score=None,metrics={k:e[k] for k in ('seconds','touches','checkpoints')})
