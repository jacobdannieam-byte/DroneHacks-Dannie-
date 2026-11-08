# Telemetry from drone 16

This folder holds the flight records of drone 16, in a form that Git can keep. Do not edit these
files by hand. The script `drone_code/sync_telemetry.sh` writes them.

## How to update this folder

On the Mac, at the top of the repository, type:

```bash
drone_code/sync_telemetry.sh --push
```

The script copies the records from the drone, packs them into this folder, commits only this
folder and pushes. Without `--push`, it commits nothing.

## What is in this folder

- `skybeta-16/flight_records/` holds one folder for each run of our programs. The folder name
  starts with the date and time on the drone. After a power cycle the drone clock can be wrong,
  so some names start with `1970`.
- `skybeta-16/flight_logs/` holds the records of the hover-and-land flight.
- Each file is compressed with gzip and ends in `.gz`. To read one, type:

```bash
gzip -dc telemetry/skybeta-16/flight_records/<folder>/events.jsonl.gz | head
```

## What is not in this folder

- Camera pictures. They are too large for Git. They stay in `flight_records_from_drone/`, which
  Git ignores.
- A file that is larger than 45 MB after compression. The script names each one.
- The flight controller's own log. `drone_code/COLLECTING_RECORDS.md` tells how to get it.
