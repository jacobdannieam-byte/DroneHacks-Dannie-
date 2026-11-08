# DroneHacks autonomous drone companion

This repository contains the team's GPS-denied quadrotor companion code.
The managed environment uses ROS 2 Jazzy, NVIDIA cuVSLAM, PX4 and
Micro XRCE-DDS. The `main` branch is the stable native reference.
Use `integration/unified-sitl-readiness` for the combined Phase 1–8
diagnostics and the first managed simulator smoke.

For offline verification and service status, run from the matching checkout.

```bash
python3 -m pytest -q tests
python3 onboard/services.py status
```

In the simulator, start with the [read-only SITL runbook](docs/unified_sitl_readiness.md)
while the vehicle is landed and disarmed. The native mission entrypoint
is `python3 main.py` in the approved companion configuration. See the
[flight and operator procedure](AI_CONTEXT.md) and the [onboard guide](onboard/README.md)
before an approved flight. After keyboard RC, land, release RC and use
**Reset world** in the simulator UI before starting an automated run.

| Stage | Current code |
| --- | --- |
| Shared perception and representation | `onboard/slam/`, `onboard/vio/`, `onboard/adapters/` and `onboard/config/` handle sensing, frames and estimator interfaces. `refnav/depth.py` supplies baseline corridor clearance. On readiness, `refnav/world_model/` adds bounded metric evidence and optional persistent semantics. |
| Navigation | The classical reference follows configured route legs through `refnav/core.py`. On readiness, `refnav/reasoning/`, `refnav/dynamics/` and `refnav/predictive_planner.py` supply offline and diagnostic hybrid planning. |
| Execution | `refnav/node.py` is the native mission's ROS/PX4 command owner. It uses the existing health, depth, Offboard, takeover and landing supervision. The planned strategy integration is documented separately. |

The [classical emulator at `1f60fb5`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/tree/1f60fb55c6f9b7d9bb9240c6919b7a4634ac74ae/simulator)
uses a separate Docker/Humble and PX4 SIH graph with simulated sensor
messages and the organizer interface. Its run instructions and evidence
are specific to that environment. The managed hybrid still needs
native integration and course qualification.

## Repository audit and proposed migration

- [Branch inventory](docs/repository_audit/BRANCH_INVENTORY.md) for tips, ancestry, changes and draft PRs.
- [Capability matrix](docs/repository_audit/CAPABILITY_MATRIX.md) for implementations, dependencies and tests.
- [Target architecture](docs/repository_audit/TARGET_ARCHITECTURE.md) for strategy contracts and the command boundary.
- [Migration plan](docs/repository_audit/MIGRATION_PLAN.md) for the proposed landing order and archival decisions.
- [Simulation matrix](docs/repository_audit/SIMULATION_MATRIX.md) for entrypoints, observed results and runtime blockers.
- Results, issues and retests for [classical](docs/simulation_results/classical.md) and [hybrid](docs/simulation_results/hybrid.md).
- [Cleanup checklist](docs/repository_audit/CLEANUP_CHECKLIST.md) for remaining review and qualification tasks.

The [canonical roadmap](https://app.notion.com/p/411f393aa76881c79ae2fb7f6d7d96f7)
tracks native phase gates. The [prompt library](https://app.notion.com/p/411f393aa76881c4a006d2d3c695488c)
contains the research and validation context.

For historical team setup and course context, see the
[code walkthrough](Dannie_Computer_Setup/CODEBASE_WALKTHROUGH.md),
[session notes](Dannie_Computer_Setup/Session_context.md),
[competition notes](Dannie_Computer_Setup/Website_context.txt) and
[connection guide](Dannie_Computer_Setup/CONNECT_FROM_LAPTOP.md).
