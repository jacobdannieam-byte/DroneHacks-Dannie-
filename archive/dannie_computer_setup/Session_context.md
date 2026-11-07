# My DroneHacks session context

## What I want help with
filepath
"C:\Users\jacob\Documents\antigravity\Dronehacks_stuff"

My main goal:
Understand how this simulation environment works and how to use it myself.
Then learn how to use Gemini in Antigravity to make small, tested improvements
toward the autonomous-drone challenge.
Later, explore safe ways to automate navigation through the course.

What I can currently do:
- Log into the website, open Workbench, and see the project files.
- Intermediate Python and Linux terminal skills.
- Basic ROS 2 familiarity; previously worked with Singularity containers, SLAM_box, and Isaac Sim.

What I cannot do yet:
- Complex ROS 2 debugging and multi-node architectures.
- I do not yet have a clear mental map of how PX4, ROS 2, Micro XRCE-DDS, and the simulation interact.
- Need guidance on inspecting topics and estimator health without accidentally commanding the drone.
- Need help understanding which files start missions and how to run starter tests safely.

Treat these as starting assumptions. Check my understanding through simple explanations and a few focused questions, rather than assuming I know nothing or requiring me to describe every knowledge gap upfront.

The last thing I tried:
Created my own branch in my own repository to inspect the files.

My experience:
- Python: Intermediate
- Linux terminal: Intermediate
- ROS / PX4: Beginner

## My laptop and AI tools

- Laptop operating system: Windows and WSL
- Antigravity: Installed and updated
- Selected models available in Antigravity: Gemini 3.8 Flash (High Thinking), GPT-OSS 120B, Claude 3.7 Sonnet / Opus
- The folder Antigravity has open: C:\Users\jacob\Documents\antigravity\Dronehacks_stuff
- Does Antigravity access the live drone companion? Not directly; working from the local repository copy.
- Where I will use Perplexity Computer: Browser / Desktop app.

## DroneHacks website & Hardware Context

- Challenge page: https://dronehacks.groups.stanford.edu/connect/challenge
- Challenge access: Requires login; full rules and course details are attached in Website_context.txt.
- Selected simulation world: Tresidder Oak indoor drone net (20×20 ft, 10 ft tall, 68 in walls, GPS-denied obstacle course: Maze, Dark Tunnel, Chicane, Landing Pad).
- Selected drone: Sky Beta (NVIDIA Jetson Orin Nano 8GB companion, Holybro Pixhawk 6C running PX4, Luxonis OAK-D S2 stereo depth camera, downward Holybro VL53L1X rangefinder).
- Session status: Turned off
- Have I used keyboard/manual RC controls? No
- Have I reset the world since then? Yes
- Have I completed a successful flight? No

## Lab & Simulation Guardrails

- The container companion workspace path is `/var/lib/flyx-lab/workspace`.
- PX4 status and local-position topics on this container use the `_v1` suffix (e.g., `/fmu/out/vehicle_status_v1`).
- The simulation uses `use_sim_time` and `/clock`.
- GPS is strictly disabled indoors; PX4 relies on visual odometry (`/fmu/in/vehicle_visual_odometry` via cuVSLAM / `onboard/vio/ev_bridge.py`).
- After any keyboard RC usage, the simulation requires landing, releasing RC, and clicking "Reset world" in the web UI. Simulator reset is a web UI action, not a flight code API.
- Flight altitude must stay below wall tops (68 in / ~1.72 m) except on the optional return bonus.

## Files and evidence I am attaching

- `CONNECT_FROM_LAPTOP.md` (connection instructions and environment paths)
- `Website_context.txt` (complete challenge rules, scoring, checkpoints, and hardware specs)
- `repomix-output.txt` (packaged codebase repository snapshot)
- `AI_CONTEXT.md` (official lab agent instructions and interface contracts)
- This `Session_context.md`

## Permissions for the first task

Start with read-only onboarding.

You may read my attachments, explain the architecture, identify missing information, and propose commands.
Do not edit flight code, install packages, restart services, reset the simulation, publish control messages, arm, or fly.
Do not assume access to the live simulator.

In an existing companion terminal (Ctrl + backtick in Workbench, path `/var/lib/flyx-lab/workspace`), you can suggest these read-only baseline inspection commands:

```bash
pwd
python3 --version
printenv ROS_DISTRO ROS_DOMAIN_ID
ros2 topic list
python3 -m pytest tests/ -q
```

ros2 topic list
python3 -m pytest tests/ -q
```