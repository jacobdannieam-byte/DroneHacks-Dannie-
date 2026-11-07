# Team Quiet Sparrow, drone 16 (DroneHacks 2026)

This repository holds the code for our drone. Each folder has one purpose.

## The folders

| Folder | Purpose | Where it runs |
|---|---|---|
| [`drone_code/`](drone_code/README.md) | The code that flies drone 16. Also the scripts that copy it to the drone and copy the records back. | The drone. The scripts run on the Mac. |
| [`simulator/`](simulator/README.md) | The 3D rendered sim and the Gazebo environment. We test the drone code here before we fly. | The Mac only. |
| [`course_map_tools/`](course_map_tools/README.md) | Tools that make the course maps from our phone scan and from the organizers' drawing. | The Mac only. |
| [`docs/`](docs/) | Our plans and guides. `docs/from_the_organizers/` holds the organizers' guides. | Read only. |
| [`archive/`](archive/README.md) | Code that we do not use. We keep it only for reference. | Nothing runs it. |

## Where to start

1. Read [`drone_code/TEST_SEQUENCE.md`](drone_code/TEST_SEQUENCE.md). It gives each command to test the drone, in order.
2. Read [`drone_code/README.md`](drone_code/README.md). It tells what runs on the drone.
3. Read [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md). It gives the plan for the competition.
4. Read [`docs/READABLE_CODE.md`](docs/READABLE_CODE.md) before you write code.

## The rule for changes

1. Change the code on the Mac, in this repository.
2. Test it in the simulator.
3. Copy it to the drone with `drone_code/install_on_drone_16.sh`.

Do not edit code on the drone. Then the repository and the drone always agree.

## Safety rules

- Log in only to drone 16 (`skybeta-16`). A login to a different drone causes disqualification.
- Do not put passwords or keys in a file, a script or Git. This repository is public.
- Get the approval of the team before the drone arms.
