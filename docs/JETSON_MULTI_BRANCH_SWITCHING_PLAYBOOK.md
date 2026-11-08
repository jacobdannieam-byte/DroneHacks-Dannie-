# Jetson Multi-Branch Strategy Switching Playbook
**DroneHacks 2026 — Team Quiet Sparrow (Drone 16)**

---

## 1. Executive Summary & Feasibility Assessment

**Is downloading and switching between three different automation branches on the Jetson doable?**

**Yes, absolutely.** However, doing a naive `git checkout <branch>` on the drone during competition introduces critical failure points:
1. **No Internet in the Flight Net:** The venue network / arena netting often has zero external internet. You cannot run `git fetch` or `git pull` from GitHub while queued on the flight line.
2. **Untracked File Lockouts:** When `team_nav` runs, ROS 2 and Python write `.pyc` caches and flight telemetry into the directory. A standard `git checkout` will fail with:
   ```text
   error: The following untracked working tree files would be overwritten by checkout...
   ```
3. **Rebuild Latency:** Every time source code changes, you must rebuild the ROS 2 workspace via `docker exec skybeta-stack team_build.sh`, which takes **45 to 60 seconds**. In a 3-minute flight slot, every second counts.

Below are the **three qualified architectures** to switch methods, ranked from fastest/safest to most flexible.

---

## Architecture Comparison

| Architecture | Switch Speed | Rebuild Required? | Network Dependent? | Competition Risk |
|---|---|---|---|---|
| **Tier 1: Single-Package Mode Dispatch** *(Recommended)* | **0 seconds (Instant)** | **No** | None | **Lowest** (No rebuild, no symlink breakage) |
| **Tier 2: On-Drone Multi-Slot Symlinks** | **~45 seconds** | **Yes (`team_build.sh`)** | None (Pre-staged) | **Low** (Safe if pre-staged on laptop) |
| **Tier 3: Laptop-to-Drone Branch Deployer** | **~60 seconds** | **Yes (`team_build.sh`)** | USB Cable (`192.168.55.1`) | **Low** (Laptop owns git history) |

---

## Tier 1: Single-Package Mode Dispatch (0-Second Switch, No Rebuild)

The cleanest architecture in ROS 2 is to keep all three automation algorithms in the same repository branch under `team_nav`, selected via command-line flags or environment variables.

### How it works
Your flight entrypoint (`drone_code/team_nav/team_nav/course_run.py` or `practice_flight`) accepts a `--strategy` argument:

```bash
# Method 1: Deterministic Waypoints (First Leg Qualification)
./drone.sh practice_flight first_maze_leg --bare-bones-safety

# Method 2: Dynamic A* Corridor Navigation
./drone.sh course_run --strategy astar

# Method 3: Exploratory cuVSLAM Frontier
./drone.sh course_run --strategy exploratory
```

### Why this is best for competition:
- **Zero build time:** You do not wait 60 seconds for `team_build.sh`.
- **Instant pivot:** If Method 2 fails in Flight 1, you can switch to Method 1 in 5 seconds while still in the flight cage.

---

## Tier 2: On-Drone Multi-Slot Symlink Switcher

If your three methods are on completely separate git branches with fundamentally different package layouts, pre-stage all three on the Jetson into separate folders.

### 1. Directory Layout on the Jetson (`skybeta-16`)
Create a dedicated `~/strategies/` directory outside `~/team_ws`:

```text
/home/dronehacks/
├── strategies/
│   ├── slot1_deterministic/       <-- Branch: classical-dannie-attemptone
│   │   └── team_nav/
│   ├── slot2_astar/               <-- Branch: astar_reactive
│   │   └── team_nav/
│   └── slot3_exploratory/         <-- Branch: visual_slam_frontier
│       └── team_nav/
├── switch_strategy.sh             <-- Instant switcher script
└── team_ws/
    └── src/
        └── team_nav -> /home/dronehacks/strategies/slot1_deterministic/team_nav  (Symlink)
```

### 2. Pre-Staging the Three Branches from Your Laptop
Before walking to the arena net, push all three branches from your laptop to the Jetson over the USB cable (`192.168.55.1`) or venue Wi-Fi:

```bash
# In your laptop terminal:
DRONE="dronehacks@192.168.55.1"

# 1. Create strategy slots on the Jetson
ssh "$DRONE" "mkdir -p ~/strategies/slot1 ~/strategies/slot2 ~/strategies/slot3"

# 2. Stage Branch 1
git checkout classical-dannie-attemptone
rsync -a --delete --exclude __pycache__ drone_code/team_nav/ "$DRONE:strategies/slot1/team_nav/"

# 3. Stage Branch 2
git checkout astar_reactive
rsync -a --delete --exclude __pycache__ drone_code/team_nav/ "$DRONE:strategies/slot2/team_nav/"

# 4. Stage Branch 3
git checkout visual_slam_frontier
rsync -a --delete --exclude __pycache__ drone_code/team_nav/ "$DRONE:strategies/slot3/team_nav/"
```

### 3. The On-Drone Switcher Script (`~/switch_strategy.sh`)
Create this script on the Jetson at `~/switch_strategy.sh`. It automatically logs every switch to an audit file, writes the active strategy manifest, and rebuilds the ROS workspace:

```bash
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
```

Make it executable on the Jetson:
```bash
chmod +x ~/switch_strategy.sh
```

### 4. How to Switch in the Net
While SSH-ed into the Jetson:
```bash
# Switch to Method 1:
~/switch_strategy.sh 1

# View switch history:
~/switch_strategy.sh log

# Check drone status:
cd ~/team_ws && ./drone.sh status

# Fly:
./drone.sh practice_flight first_maze_leg --bare-bones-safety
```

---

## Tier 3: Laptop Multi-Branch Deployer (`deploy_branch.sh`)

If you prefer to keep all files on your laptop and simply push the desired branch over USB cable whenever you want to test:

Create `drone_code/deploy_branch.sh` on your laptop (includes deployment audit logging):

```bash
#!/bin/bash
# drone_code/deploy_branch.sh <branch-name>
set -euo pipefail

BRANCH="${1:-}"
DRONE_ADDRESS="${DRONE_ADDRESS:-192.168.55.1}"
DRONE_LOGIN="dronehacks@$DRONE_ADDRESS"
DEPLOY_LOG="telemetry/deployments.log"
mkdir -p "$(dirname "$DEPLOY_LOG")"

if [ -z "$BRANCH" ]; then
    echo "Usage: $0 <branch-name>"
    exit 1
fi

COMMIT_SHA="$(git rev-parse --short HEAD)"
TIMESTAMP="$(date '+%Y-%m-%d %H:%M:%S')"
echo "[$TIMESTAMP] DEPLOY START: Branch $BRANCH ($COMMIT_SHA) -> $DRONE_LOGIN" | tee -a "$DEPLOY_LOG"

echo "Switching laptop working tree to: $BRANCH"
git checkout "$BRANCH"

echo "Deploying $BRANCH to $DRONE_LOGIN..."
drone_code/install_on_drone_16.sh

echo "[$TIMESTAMP] DEPLOY SUCCESS: $BRANCH ($COMMIT_SHA) installed on drone 16." | tee -a "$DEPLOY_LOG"
```

---

## 5. Stage & Step Telemetry Logging: Verifying "What We See is Actually Happening"

When testing or qualifying in the arena, observer teammates watching through the net need immediate, unambiguous proof that the drone's physical location matches its software target at every stage and step.

The flight code (`team_nav`) provides **three synchronized levels of verification logging**:

### Level 1: Human-Readable Terminal Events (`events.jsonl` & stdout)
At every stage transition and progress milestone, the flight software computes exact $(X, Y, Z)$ course coordinates, heading, and hazard clearance, printing structured step logs:

```text
[12:35:05] [CLIMB] Step: climb_complete | Drone: (-2.51, +0.91, 0.75) m | Target: (-2.51, +0.91, 0.75) m | PosErr: 0.01 m | Yaw: +90.0° | Laser: 0.75 m | TrackAge: 0.03s | Reached cruise altitude (0.75 m).
[12:35:08] [FIRST CORRIDOR] Step: corridor_ingress | Drone: (-2.51, +0.91, 0.75) m | Target: (-2.51, +0.91, 0.75) m | PosErr: 0.02 m | Yaw: +90.0° | Laser: 0.74 m | TrackAge: 0.03s | Heading North down Corridor 1 at 0.30 m/s.
[12:35:11] [FIRST CORRIDOR] Step: checkpoint_1_cleared | Drone: (-2.51, +1.80, 0.75) m | Target: (-2.51, +1.80, 0.75) m | PosErr: 0.02 m | Yaw: +90.0° | Laser: 0.75 m | TrackAge: 0.03s | CHECKPOINT 1 CLEARED (+10 Points) - Exit of Corridor 1!
[12:35:13] [FIRST CORRIDOR] Step: turn_pocket_arrived | Drone: (-2.51, +2.18, 0.75) m | Target: (-2.51, +2.18, 0.75) m | PosErr: 0.01 m | Yaw: +90.0° | Laser: 0.75 m | TrackAge: 0.03s | Arrived in Turn Pocket! 0.87 m clearance to North perimeter net.
[12:35:16] [TURN EAST] Step: turn_start | Drone: (-2.51, +2.18, 0.75) m | Target: (-2.51, +2.18, 0.75) m | Yaw: +90.0° | Starting calibrated 15°/s turn East. Monitoring lateral drift.
[12:35:19] [TURN EAST] Step: turn_30deg | Drone: (-2.51, +2.18, 0.75) m | Yaw: +30.0° (Tgt: +30.0°, Err: 0.0°) | Laser: 0.75 m | Turn progress: 60° completed (Heading 30°).
[12:35:22] [TURN EAST] Step: turn_complete | Drone: (-2.51, +2.18, 0.75) m | Yaw: +0.0° (Tgt: +0.0°, Err: 0.0°) | Laser: 0.75 m | Turn complete: Nose pointing directly East (0.0°) into maze!
[12:35:25] [ENTER MAZE] Step: checkpoint_2_cleared | Drone: (-1.80, +2.18, 0.75) m | Target: (-1.80, +2.18, 0.75) m | PosErr: 0.02 m | Yaw: +0.0° | CHECKPOINT 2 CLEARED (+15 Points) - Passed Divider M1 into maze corridor!
[12:35:30] [ENTER MAZE] Step: landing_spot_reached | Drone: (-0.50, +2.18, 0.75) m | Target: (-0.50, +2.18, 0.75) m | PosErr: 0.01 m | Yaw: +0.0° | Arrived at designated landing target (-0.50 m, +2.18 m)!
[12:35:34] [LANDING] Step: touchdown_disarmed | Motors disarmed on arena floor in 28.4 s! Points banked.
```

### Level 2: Real-Time Live Monitoring Over SSH
While the drone is powered and armed in the cage, stream telemetry in real time:

```bash
# In your SSH terminal:
# 1. Tail the latest flight events live:
tail -f ~/team_ws/flight_records/$(ls -t ~/team_ws/flight_records | head -n 1)/events.jsonl

# 2. Or attach to the running competitor tmux window:
docker exec -it skybeta-stack tmux attach -t skybeta:team_launch
```

### Level 3: High-Frequency Numerical Telemetry (`stage_tracking.csv`)
For post-flight forensics and referee verification, every cycle records into `stage_tracking.csv`:
- `course_east`, `course_north`, `course_height` (Arena $(X, Y, Z)$)
- `target_east`, `target_north`, `target_height`
- `pos_error_m` (Euclidean distance error)
- `heading_deg` vs `target_heading_deg`
- `laser_height_m` (Raw laser altitude to floor)
- `tracking_age_s` (Camera tracking silence gap)
- `north_net_dist_m` (Real-time safety buffer to North net)

---

## 6. Pre-Arming Safety Checklist When Switching Methods

Whenever you switch between automation branches on the drone:

1. **Verify Flight Interface Reset:**
   ```bash
   cd ~/team_ws
   ./drone.sh status
   ```
   Ensure no leftover trajectory setpoint publisher is active.
2. **Verify Telemetry Freshness:**
   ```bash
   ./drone.sh hand_test all_table_checks
   ```
   Confirm camera tracking (`cuVSLAM` / `oakd_vslam`), bottom laser rangefinder, and IMU are publishing valid numbers.
3. **Check Disarmed & Landed Reports:**
   PX4 mode must report `disarmed` and `landed: True` before commanding takeoff.
4. **Arming Approval:**
   Obtain physical pilot and safety observer clearance before arming and switching remote to Position mode.

