# Classic and Hybrid clearance bootstrap

Status: source-based design comparison, 2026-11-08. No implementation, flight authorization, or new moving test. Classic was read only at protected revision `ad31ae55ebf7cd3b2eaf57917537242831e3c752`; paths in the Classic section refer to that revision, not the working tree.

## What the successful Classic actually does

The baseline is `simulator/test_flights/route_test.py`, not the generic `refnav/core.py` forward-corridor example. Its own description says it plans once and follows the route; camera correction and replanning are future competition-program work. It reads the participant `--drone-map`, deliberately not the simulator `--world` file. The default is `drone_code/team_nav/course_maps/phone_scan_2026-11-06.json`; the default mission is `dronehacks_2026_side_gap_first.json`. Other supplied maps contribute possible low obstacles for laser avoidance. File availability does not establish competition permission or survey accuracy.

`route_test.py:plan_the_route` uses the map launch mark and a supplied 90-degree start heading through `start_frame.py`. It assumes this map-to-tracking alignment rather than estimating and bounding it online. Startup waits for simulator readiness, checks that laser and PX4 odometry arrived, remembers the launch floor, streams targets for two seconds, requests the simulator start service, and climbs to 0.8 m (0.10 m arrival tolerance). These are the methods in `simulator/test_flights/simulator_test_flight.py`; they are not a depth-derived proof that the complete takeoff or stopping volume is free.

| Component | Actual assumption/check | What it does not establish |
|---|---|---|
| `team_nav/route_planner.py` | Static map geometry, ordered openings/checkpoints, shortest-path cost with wall proximity penalty, straightening and boulder avoidance | Fresh observation of map-negative space or dynamic obstacle clearance |
| `team_nav/drone_dimensions.py` | Horizontal radius 0.30 m plus 0.05 m safety margin; half-height 0.10 m plus 0.10 m vertical margin | Measured all-attitude body envelope or Hybrid uncertainty/stopping bounds |
| `team_nav/clearance_grid.py` | 0.05 m grid; straight segments sampled at up to 0.025 m intervals; geometry intersecting the relevant height slab | Continuous swept-body and braking qualification |
| `team_nav/route_follower.py` | Sliding target nominally 0.5 m/s, horizontal lead capped at 0.65 m, corner arrival tolerances | Hard actual speed bound, online wall checking or guaranteed stopping distance |
| `route_test.py:stop_the_run_if_needed` | Simulator touch oracle, onboard-style tilt/spin TouchDetector, HeightCheck comparing laser/PX4 with map context | Predictive collision avoidance; the simulator touch oracle is a test instrument, not transferable flight sensing |

The `team_nav/` paths above are under `drone_code/team_nav/`. Depth sensor presence in the launch does not mean this route harness consumes depth clearance. It does not.

## Evidence and its limits

`artifacts/strategy_comparison/classical_baseline.json` binds saved run `run-AdMalPxA` to the protected revision: completion true, four checkpoints, zero touches, 84.3 seconds, path 17.97392015946742 m, real-time factor 0.999. The minimum reported clearance is 0.0751 m, sampled by the referee at propeller points; it is not Hybrid D3 swept-body clearance. The saved source/map/mission/vehicle/sensor comparisons match after CRLF normalization. This is one successful native Gazebo rigid-body course run, not repeated paired evidence or, by itself, evidence of a real-aircraft flight. Preserve the team's protected baseline independently of that distinction.

The artifact explicitly leaves permitted-map, initial-pose, seed and termination parity unconfirmed. These must be resolved before a fair same-contract comparison. Neither the successful result nor a simulator world generated from a map establishes that the map is independently surveyed or that a real scene has remained empty.

## Why Hybrid refuses the initial blind volume

`refnav/world_model/spatial_pipeline.py` and `spatial_volume.py` build local metric evidence from calibrated sensor rays, pose association and uncertainty. Unobserved and expired cells are UNKNOWN; free evidence expires after 500 ms by default. `refnav/trajectory_evaluation.py` checks swept volume with body and uncertainty margins. `refnav/hybrid_gateway.py` revalidates the selected candidate and displacement; `refnav/stopping_envelope.py` separately checks stopping support when attached. A current forward depth frame can provide real free cells ahead while leaving the camera's near range, rear body region and stopping sweep unknown. Faster GPU processing cannot supply that missing observation.

This is a stronger and different contract than Classic's trusted map-negative space. Refusal is not evidence that the geometric implementation is defective. Reusing a map for route goals, semantic openings or P2 localization is not permission to convert its unoccupied regions into fresh metric FREE. `refnav/world_model/planner_map_input.py` gates map-pose diagnostics by version/reference/epoch/freshness; it does not confer clearance authority. Current native runtime does not attach that map consumer.

Startup is also a separate boundary: `refnav/core.py:step` checks Hybrid owner readiness and usable depth before arming, then has explicit climb/hover phases before route navigation. Do not claim that navigation's D3 checks alone qualify every takeoff, floor-contact or landing phase. A landed vehicle intersecting a conservative body sphere and the floor needs a reviewed contact-phase contract, not a global floor exemption.

S0/S2/S3/S4 share the authoritative clearance boundary. Candidate generation, information gain, learned ranking and residual prediction do not make missing geometry observable. A bounded stationary reobservation may collect new evidence, but cannot promise to clear a persistent blind zone. Rotation or translation to collect views must itself have clearance authority.

## Options and recommended minimal integration

Final integration direction: [the concrete plan](HYBRID_CLEARANCE_BOOTSTRAP_PLAN.md) selects an explicitly controlled-SITL map-assisted route mode (option 3), beginning with one bounded S0 segment. The bounded-bootstrap option below remains an alternative only where actual observed-only handoff is demonstrable; it is not the prerequisite for this selected route mode.

1. **Keep strictly observed-free execution.** Provide sensor coverage or independently authorized acquisition that actually observes the required whole-body and stopping volume. Validate occlusion, near range, all relevant directions, freshness and uncertainty. A yaw scan with the current forward camera alone is not a proven solution to the near-body blind region. This option preserves the current evidence contract but may require sensor changes.
2. **Explicit, bounded map-backed bootstrap (alternative if observed-only handoff is feasible).** Add a separate clearance provenance and admission contract for a reviewed launch/acquisition region. This intentionally adds an assumption; it is not a repair to depth sensing. Never seed metric FREE cells, refresh prior timestamps on camera arrivals, or silently make UNKNOWN pass. Require independently supplied permitted geometry and a map-to-navigation transform with bounded error. If only simulator world truth is available, this option is inadmissible under the flight-algorithm boundary.
3. **Map-assisted whole-course execution.** Reuse Classic route geometry as route-goal proposals and add explicit map-backed clearance plus current sensor vetoes, local replanning and stopping/recovery checks. This is a larger contract change than bootstrap. It must be separately reviewed and compared against Classic with matching map and initial-alignment assumptions. Do not quietly extend option 2 across the course when acquisition fails.

The smallest useful work now is an offline typed-prior evaluator and diagnostics, initially for S0, with S2/S3/S4 consuming identical results in shadow. A proposed grant must identify map content hash/version, allowed-use approval, surveyed free-region boundaries and excluded space, coordinate reference/reset epoch, transform error bounds, body/controller/stopping profile, phase, time/distance limits, and abort/recovery region. A sparse obstacle drawing's missing objects are not an exhaustive free-region survey. A simulator configuration may be checked by an external harness for immutability, but live truth poses, contacts and world geometry must not feed the policy.

Admission must require the complete candidate and stopping volumes to be covered by fresh observed evidence or the explicit bounded prior grant, with the provenance of each portion retained. Fresh occupied evidence and sensor/map disagreement veto the grant; unknown forward travel remains a refusal unless it is specifically inside the reviewed prior region and that policy has been approved. Current depth, estimator health, authority, clock and measured stopping gates remain mandatory. No inference from a static map can guarantee absence of newly introduced dynamic obstacles.

The bootstrap state machine should be explicit: disarmed validation -> independently qualified takeoff/contact phase -> bounded evidence acquisition -> observed-only handoff, or reviewed stop/recovery. Handoff requires the actual observed swept and stopping volumes to pass the existing checks; it cannot be based on elapsed time or successful ascent. If that cannot happen with the available sensor geometry, report the structural limit and stop rather than extending the grant. A new flight approval would be required after implementation and offline validation.

## Required offline evidence before a motion proposal

- Map identity/permission and transform/reset mismatch refuse admission; uncertainty inflation cannot exceed surveyed boundaries.
- A hole, omitted obstacle, floor/ceiling edge or incomplete stopping region refuses; no forward-only check substitutes for complete volume coverage.
- Fresh occupied evidence overrides a prior; expired observations revert to UNKNOWN without refreshing the prior. Map contradiction revokes admission.
- Time/distance/phase limits remain bounded across repeated frames; no grant renewal by retries or strategy change.
- Acquisition cannot issue movement outside its grant; handoff fails with any required observed cell unknown. Failure follows the separately validated recovery contract.
- Identical geometry yields identical admission across S0/S2/S3/S4; learned ranking cannot promote a rejected candidate. Reports distinguish observed, prior-backed and refused volume and never label prior-backed success observed-only.

No new motion success is claimed here. Existing native disarmed shadow data can validate sensor ingestion and refusal reasons; offline fixtures can validate this proposed contract; neither qualifies the new map-assisted motion mode.
