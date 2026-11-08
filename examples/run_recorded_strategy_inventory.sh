#!/usr/bin/env bash
# Separate process namespace, no ROS network, read-only source and saved bag.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
baseline="$(dirname "$root")/classical-smoke-ad31ae5/simulator/gazebo_trial/generated/hybrid_runs/run-AdMalPxA/observations"
docker run --rm --network none --read-only --tmpfs /tmp \
  --name strategy-recorded-inventory \
  -v "$root:/strategy:ro" -v "$baseline:/recording:ro" \
  -v "$root/artifacts/strategy_comparison:/output:rw" \
  --entrypoint /bin/bash drone-simulator-gazebo-hybrid -lc \
  'source /opt/ros/humble/setup.bash; source /opt/px4_messages/install/setup.bash; python3 /strategy/examples/recorded_native_inventory.py /recording/observations_0.db3 > /output/classical_native_inventory.json'
