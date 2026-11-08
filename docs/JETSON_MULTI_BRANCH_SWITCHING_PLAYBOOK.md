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
Create this script on the Jetson at `~/switch_strategy.sh`:

```bash
#!/bin/bash
# switch_strategy.sh - Fast slot switching on skybeta-16
set -euo pipefail

SLOT="${1:-}"

case "$SLOT" in
    1) TARGET="$HOME/strategies/slot1/team_nav"; NAME="Slot 1 (Deterministic First Leg)" ;;
    2) TARGET="$HOME/strategies/slot2/team_nav"; NAME="Slot 2 (Dynamic A* Planner)" ;;
    3) TARGET="$HOME/strategies/slot3/team_nav"; NAME="Slot 3 (Exploratory cuVSLAM)" ;;
    *)
        echo "Usage: $0 {1|2|3}"
        echo "  1: Slot 1 (Deterministic First Leg)"
        echo "  2: Slot 2 (Dynamic A* Planner)"
        echo "  3: Slot 3 (Exploratory cuVSLAM)"
        exit 1
        ;;
esac

if [ ! -d "$TARGET" ]; then
    echo "ERROR: Target directory $TARGET does not exist!"
    exit 1
fi

echo "=========================================="
echo "Activating: $NAME"
echo "=========================================="

# 1. Ensure any running flight software is stopped safely
cd ~/team_ws
if [ -f ./drone.sh ]; then
    ./drone.sh start watch_only || true
fi

# 2. Re-point the symlink in ~/team_ws/src
ln -sfn "$TARGET" ~/team_ws/src/team_nav
echo "Symlink updated: ~/team_ws/src/team_nav -> $TARGET"

# 3. Trigger Docker container rebuild
echo "Rebuilding ROS 2 workspace in container (approx 45 s)..."
docker exec skybeta-stack team_build.sh

# 4. Refresh drone helper script permissions
chmod +x ~/team_ws/src/team_nav/drone.sh
ln -sfn src/team_nav/drone.sh ~/team_ws/drone.sh

echo "=========================================="
echo "SUCCESS: $NAME is now active and built!"
echo "Check status with: ./drone.sh status"
echo "=========================================="
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

# Check status:
cd ~/team_ws && ./drone.sh status

# Fly:
./drone.sh practice_flight first_maze_leg --bare-bones-safety
```

---

## Tier 3: Laptop Multi-Branch Deployer (`deploy_branch.sh`)

If you prefer to keep all files on your laptop and simply push the desired branch over USB cable whenever you want to test:

Create `drone_code/deploy_branch.sh` on your laptop:

```bash
#!/bin/bash
# drone_code/deploy_branch.sh <branch-name>
set -euo pipefail

BRANCH="${1:-}"
DRONE_ADDRESS="${DRONE_ADDRESS:-192.168.55.1}"
DRONE_LOGIN="dronehacks@$DRONE_ADDRESS"

if [ -z "$BRANCH" ]; then
    echo "Usage: $0 <branch-name>"
    exit 1
fi

echo "Switching laptop working tree to: $BRANCH"
git checkout "$BRANCH"

echo "Deploying $BRANCH to $DRONE_LOGIN..."
drone_code/install_on_drone_16.sh

echo "Done. $BRANCH is compiled and ready on drone 16."
```

---

## 4. Pre-Arming Safety Checklist When Switching Methods

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

