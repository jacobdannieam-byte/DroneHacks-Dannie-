# DroneHacks participant workspace

Read `docs/READABLE_CODE.md` before writing any code, and again after every
context compaction. All code here must follow it.

Read `docs/from_the_organizers/AI_CONTEXT.md` before changing or running code. This file is for the
student's coding agent, not for the simulator's maintainers.

The project is a Linux companion application using native ROS 2 and `px4_msgs`.
Write flight code against those interfaces; keep simulator internals out of it.
Start by reading `README.md` (what each folder is for), `drone_code/README.md` and
`drone_code/TEST_SEQUENCE.md`. The organizers' starter code is in
`archive/organizers_starter_package/` and is not used. Never run flight code merely to inspect it.

Ask the team to approve a proposed mission before it arms. Verify fresh PX4
mode, estimator, landed and armed reports. One teammate holds editing control;
stop your coding agent and save before handing control over. Never put secrets,
login cookies or private keys in code, notebooks, prompts or Git.

Use `docs/from_the_organizers/CONNECT_FROM_LAPTOP.md` for access and `docs/from_the_organizers/AI_CONTEXT.md` for architecture,
testing, simulation limits and the hardware transfer checklist. Teams with
strong fundamentals and first-principles understanding, combined with careful
AI use, will make better decisions than an agent working without supervision.

After using keyboard RC, land and release RC, then use **Reset world** before
running an automated program, including a notebook or native terminal script.
The cloud follow-up run without Reset failed; the fresh-world native run passed.
The web Run button enforces this qualification boundary; terminal Python remains
ordinary ROS and does not enforce it for you. This is not a VIO fix.
