import json
import sqlite3
from pathlib import Path

root = Path(__file__).resolve().parent
bag = root / 'restart-observe/observations/observations_0.db3'
rows = []
with sqlite3.connect(bag.as_uri() + '?mode=ro', uri=True) as db:
    for topic_id, name in db.execute('select id, name from topics'):
        if name not in ('/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_land_detected'):
            continue
        stamps = [r[0] for r in db.execute('select timestamp from messages where topic_id=? order by timestamp', (topic_id,))]
        gaps = [(b-a)/1e9 for a,b in zip(stamps, stamps[1:])]
        rows.append(dict(topic=name, count=len(stamps), max_receipt_gap_s=max(gaps),
                         mean_receipt_gap_s=sum(gaps)/len(gaps),
                         gaps_exceeding_owner_300ms=sum(g>.3 for g in gaps)))
output = root / 'restart-authority-cadence.json'
with output.open('x') as stream:
    json.dump(dict(scope='Recorded callback cadence, not source-stamp or loss-detection latency qualification',
                   result='OWNER_BUDGET_INCOMPATIBLE_WITH_OBSERVED_CADENCE', topics=rows), stream, indent=2)
print(output.read_text())
