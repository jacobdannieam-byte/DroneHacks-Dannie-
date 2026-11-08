# Bootstrap investigation validation

- Base source: b2234db3434ce49e5d7737dbc66408db76cf514d, code-integration-hybrid.
- Near-clip focused regression: 11 passed in 0.57 s; worker broader focused suite: 78 passed.
- Final full repository suite: `python -m pytest tests/ -q -p no:cacheprovider --basetemp ../taskforce-bootstrap-005/pytest-final`: 752 passed in 35.25 s.
- Initial full suite: 751 passed, one Windows sandbox TEMP rename PermissionError in qualification-recorder readiness test. No production code was changed for this environment failure; using a fresh workspace test directory passed all tests.
- `git diff --check`: passed.
- Repeatable GPU probe: `examples/gpu_render_probe.ps1`, raw final output `../taskforce-bootstrap-005/gpu-repeatable-03`. Actual OGRE2 renderer RTX 5070 D3D12, delivered 320x240 depth image, no PX4 or flight owner. Gazebo required the configured timeout/KILL cleanup; this is rendering evidence, not clean-shutdown or full simulator qualification.
- Raw final depth.txt SHA256: 7013307e895b258d98328c5e8831e6ec266d4d3f324a537d9f51da8273700bee. Renderer and server logs included beside this file.
- Docker container list empty after probe. No aircraft arming, policy flight, Classic edit, map-control enablement or background registration.
- Prior-backed navigation remains a design, not an implemented or qualified mode. See docs/HYBRID_CLEARANCE_BOOTSTRAP_PLAN.md.
