#!/bin/bash
# switch_strategy.sh - Fast slot switching on skybeta-16 with persistent audit logging
set -euo pipefail

LOG_FILE="$HOME/team_ws/flight_records/strategy_switches.log"
MANIFEST_FILE="$HOME/strategies/CURRENT_ACTIVE_STRATEGY.json"
mkdir -p "$(dirname "$LOG_FILE")" "$HOME/strategies"

log_event() {
    local msg="[$(date '+%Y-%m-%d %H:%M:%S')] $1"
    echo "$msg"
    echo "$msg" >> "$LOG_FILE"
}

if [ "${1:-}" = "log" ] || [ "${1:-}" = "history" ]; then
    echo "=== Strategy Switch Audit Log ==="
    if [ -f "$LOG_FILE" ]; then
        cat "$LOG_FILE"
    else
        echo "No switches logged yet."
    fi
    exit 0
fi

SLOT="${1:-}"

case "$SLOT" in
    1) TARGET="$HOME/strategies/slot1/team_nav"; NAME="Slot 1 (Deterministic First Leg)"; BRANCH="classical-dannie-attemptone" ;;
    2) TARGET="$HOME/strategies/slot2/team_nav"; NAME="Slot 2 (Dynamic A* Planner)"; BRANCH="astar_reactive" ;;
    3) TARGET="$HOME/strategies/slot3/team_nav"; NAME="Slot 3 (Exploratory cuVSLAM)"; BRANCH="visual_slam_frontier" ;;
    *)
        echo "Usage: $0 {1|2|3|log}"
        echo "  1: Slot 1 (Deterministic First Leg)"
        echo "  2: Slot 2 (Dynamic A* Planner)"
        echo "  3: Slot 3 (Exploratory cuVSLAM)"
        echo "  log: View history of all strategy switches"
        exit 1
        ;;
esac

if [ ! -d "$TARGET" ]; then
    echo "ERROR: Target directory $TARGET does not exist!"
    log_event "ERROR: Attempted to switch to $NAME, but $TARGET does not exist."
    exit 1
fi

log_event "SWITCH INITIATED: Activating $NAME from $TARGET..."

# 1. Ensure any running flight software is stopped safely
cd ~/team_ws
if [ -f ./drone.sh ]; then
    ./drone.sh start watch_only || true
fi

# 2. Re-point the symlink in ~/team_ws/src
ln -sfn "$TARGET" ~/team_ws/src/team_nav
log_event "Symlink updated: ~/team_ws/src/team_nav -> $TARGET"

# 3. Write active strategy manifest
cat <<EOF > "$MANIFEST_FILE"
{
  "active_slot": "$SLOT",
  "strategy_name": "$NAME",
  "source_path": "$TARGET",
  "branch_hint": "$BRANCH",
  "activated_at": "$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
}
EOF
cp "$MANIFEST_FILE" ~/team_ws/CURRENT_STRATEGY.json

# 4. Trigger Docker container rebuild
log_event "Rebuilding ROS 2 workspace in container (approx 45 s)..."
docker exec skybeta-stack team_build.sh

# 5. Refresh drone helper script permissions
chmod +x ~/team_ws/src/team_nav/drone.sh
ln -sfn src/team_nav/drone.sh ~/team_ws/drone.sh

log_event "SWITCH COMPLETED: $NAME is active, built, and ready to fly!"
echo "Check status with: ./drone.sh status"
