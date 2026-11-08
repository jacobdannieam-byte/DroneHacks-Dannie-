# Full-course Hybrid diagnostic evidence

Successful frozen source: `cfdffc79b0cb79dab6abd81d130f1e0f78715067`.
Run: `hybrid-diagnostic-full-course-012`. Branch: `codex/hybrid-model-flight`.
See [the complete analysis](../../docs/HYBRID_FULL_COURSE_RESULT.md).

- [Actual Gazebo video](actual-gazebo-flight.mp4): 199 recorded overhead frames,
  1 fps, with a separately labelled telemetry/decision-score panel.
- [Flight and independent referee audit](model-command-audit.json): full-course,
  command replay and recording PASS; 22 goals, four checkpoints, zero contacts,
  153.57 seconds and pad landing. Safety qualification remains false.
- [Review and source hashes](REVIEW.json), [exact candidate](candidate.json),
  [runtime profile](calibration_profile.json), [owner report](model_owner.json).
- [Complete hash-chained decision log](model_owner.decisions.jsonl.gz),
  [readable decision table](decision-review.csv), and
  [11 published PINN/D0 differences](published-pinn-d0-differences.csv).
- [Classical result](classical-course-summary.json),
  [Classical trajectory](classical-flight.csv),
  [Hybrid trajectory](hybrid-flight.csv).
- [All 12 attempts](campaign-attempts.json), with compact owner and audit reports
  for the 11 earlier attempts in `failed-attempts/`. These include readiness
  refusals as well as flights; they are not 12 equivalent full-course trials.
- [Package hashes](PACKAGE_MANIFEST.json) and [publication checks](validation.json).

The audit counts 14 PINN/D0 differences including three discarded decisions;
11 reached PX4. D0 is a same-state nominal-predictor comparison, not the
historical Classical route follower. No paired D0 flight was run.

## Raw evidence retained outside Git

Git contains this compact package, including the entire compressed decision
log and actual flight video. The final 1,270,235,136-byte ROS bag, ULogs,
source archives, strict recorder samples and earlier videos remain in the
workspace sibling `physical-gazebo-evidence/hybrid-diagnostic-full-course-001`
through `-012`. They are not uploaded by this Git push. `REVIEW.json` retains
the final raw-artifact SHA256 values; `campaign-attempts.json` retains each
frozen candidate/source hash and relative evidence location. The original
Classical evidence remains protected separately.

For a complete raw replay, retrieve the final external `run/observations/`
alongside these artifacts and use the pinned ROS/PX4 image from `candidate.json`.
The original bag replay already passed and its report is included here. The
compact package alone cannot reproduce bag deserialization or establish new
hardware qualification. Documentation HEAD may follow the frozen flown commit.

Registration and model fallback were off. The route was fixed; depth-section
assessment did not control this diagnostic. Normal-profile clearance and
timing requirements are not passed merely because the diagnostic completed.
