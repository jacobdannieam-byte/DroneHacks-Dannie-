# Controlled-SITL map-assisted clearance

The optional backend is implemented. S0 flight release remains blocked by the items in [the bounded plan](S0_BOUNDED_FLIGHT_PLAN.md). Observed-only mode remains the default; profiles and examples supply no flight approval.

`SpatialSnapshot.clearance` holds a typed certificate, finite exclusion lease and live deployment verifier. `SpatialSnapshot.metric` remains the original camera-derived grid. No prior writes FREE cells or refreshes camera evidence. Prior-supported candidates carry `authorized_static` provenance. Receipts and held targets bind certificate, lease and stopping identities; logs retain observed cell counts separately from prior-backed segments.

The certificate pins a permitted participant survey, positive empty-volume coverage, frame/epoch, alignment identity, transform errors, validity and stopping-report hash. The deployment binder supports one same-frame surveyed box. An obstacle drawing's negative space is insufficient. Generated simulator world geometry remains external validation input, not route-planner input.

The separate lease has ROS and monotonic deadlines, each at most 60 seconds; a deployment verifier must confirm controlled occupancy on every recheck. Certificate validity is at most 300 seconds. Source files are hash-rechecked. Expiry, reset/alignment changes, stale sensors, source changes, occupied evidence or out-of-certificate volume refuse clearance. No automatic renewal occurs.

## Configuration boundary

The optional `map_assisted` launch section requires `certificate_file`, `certificate_sha256`, `participant_map_file`, and `participant_map_sha256`. The participant map uses schema `dronehacks.participant_clearance_map.v1`, `permitted_use: controlled_sitl`, a nonempty `review_id`, exact `reference`, and `surveyed_empty_boxes` with `lower_m` and `upper_m`. The certificate box must match an explicit surveyed box. The documentation-only [schema example](../scenarios/fixtures/s0_participant_prior.schema-example.json) is intentionally rejected by the runtime.

Loading a profile stores the certificate but grants no clearance. Deployment must call `runtime.install_map_clearance(lease, verify_exclusion, monotonic_ns=...)` with a reviewed measured stopping envelope installed. Native activation additionally requires qualified landing/recovery admission. No lease, stopping measurement or recovery permission is fabricated from disarmed checks.

## Shared checks

Planning, mission arrival, gateway reconstruction, held targets and stopping queries use the same semantics. D3 checks forecast motion with all-direction stopping inflation and rejects predicted speeds outside the measured envelope. Execution checks the entire displacement to the target, including braking space, not only the current position or forecast endpoint. Sampled predictions do not guarantee a native controller speed cap; reachable-speed and timing qualification remain required.

Immediately before native publication, the runtime rechecks exact startup output, route stopping volume and recovery admission. Both clocks are sampled again after potentially slow source verification; state/depth freshness, lease and target expiry are rechecked. Failures withhold output and yield authority. Launch geometry is a distinct deployment-authorized contract, without route or landing authority.

## Remaining release limits

- The conservative all-attitude launch envelope intersects the floor immediately after liftoff. A justified attitude/contact transition is still needed; its regression preserves refusal.
- Actual braking, landing drift, recovery and owner timing remain unmeasured for this composition. Native map-assisted activation refuses a missing recovery guard.
- Scans and S2/S3/S4 native execution remain restricted. The proposed first S0 run is a single segment in the wider enclosure, not the narrow phone course.
