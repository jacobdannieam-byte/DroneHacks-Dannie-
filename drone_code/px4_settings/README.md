# PX4 settings we changed on drone 16

`change_px4_settings.py` reads and changes them over the flight controller's USB cable.
It needs `pymavlink` (`pip install pymavlink`). Read first, then change:

```bash
python3 drone_code/px4_settings/change_px4_settings.py
python3 drone_code/px4_settings/change_px4_settings.py --change
```

Then restart the flight controller. Write the old values printed by the tool in the table below.

| Setting | Old value | New value | Why |
|---|---|---|---|
| `EKF2_MAG_TYPE` | 0 (automatic) | 5 (never use the compass) | Indoors the compass points 135 to 180° off. PX4 switched to it whenever camera reports came late, and its heading swung round. |
| `EKF2_EV_DELAY` | 60 ms | 0 ms | The camera report's time stamp is already the picture time; PX4 counted the delay twice. |
| `EKF2_DELAY_MAX` | 200 ms (as read on the drone: fill in) | 350 ms | PX4 now uses camera reports up to about 445 ms late. They reached 412 ms in bursts. |

To undo: put the old values into `SETTINGS_TO_CHANGE` and run the tool with `--change` again.

How we found this (2026-11-08): `docs/DEBUGGING_LOOP.md`, the drift example. PX4 dropped the
camera after 400 ms with no usable report (a fixed limit in PX4's `ev_control.cpp`). That limit
cannot be changed by a setting. Bursts longer than about 400 ms past the new window still make PX4
drop the camera, and our safety rule 5 (`run_safety.py`) then lands the drone.
