"""Format flight telemetry logs according to the PX4 Flight Review v2 (flight-review-rs) standard.

What this file does
-------------------
This tool converts drone telemetry runs into the high-performance columnar format
pioneered by PX4's Rust-based Flight Review v2 (https://github.com/px4/flight-review-rs).

Specifically, for each flight recording session:
  1. Compiles a high-level "metadata.json" flight summary containing arming durations,
     flight mode timelines, peak velocities, altitude envelopes, and battery statistics.
  2. Builds a "manifest.json" listing every recorded topic, message row counts, and
     valid timestamp intervals.
  3. Converts per-topic CSV files (or gzip-compressed CSV files) into standalone,
     columnar Apache Parquet files (.parquet) using Zstandard (ZSTD) compression.

How it fits with the other files
--------------------------------
- Reads recordings collected from drone 16 by "flight_recorder.py", stored either in
  "flight_records_from_drone/skybeta-16/" or in "telemetry/skybeta-16/".
- Produces output that can be directly queried via DuckDB, Pandas, Polars, or loaded
  into modern Flight Review v2 frontends with sub-second response times.
- Can also invoke the official "flight-review" Rust binary when raw Pixhawk ULog (.ulg)
  files are supplied.

What it deliberately does not do
--------------------------------
- It never modifies or deletes original flight logs.
- It never connects to, commands, or arms the drone.
- It does not require an external database daemon (PostgreSQL, MySQL) or web server.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def _open_csv_text_reader(file_path: Path):
    """Open a CSV file for reading text, automatically transparently handling .gz files."""
    if file_path.suffix == ".gz":
        return gzip.open(file_path, mode="rt", encoding="utf-8", errors="replace")
    return open(file_path, mode="r", encoding="utf-8", errors="replace")


def clean_topic_filename_to_name(file_name: str) -> str:
    """Turn a flattened file name (e.g., 'fmu__out__vehicle_local_position.csv.gz')

    into a clean topic name ('fmu/out/vehicle_local_position').
    """
    clean_stem = file_name
    for extension in [".gz", ".csv", ".parquet"]:
        if clean_stem.endswith(extension):
            clean_stem = clean_stem[:-len(extension)]
    return clean_stem.replace("__", "/")


def extract_flight_summary_metadata(session_folder: Path) -> Dict[str, Any]:
    """Scan key telemetry files in a flight session to compute high-level metadata

    matching the flight-review-rs metadata schema.
    """
    metadata: Dict[str, Any] = {
        "flight_review_version": "2.0-compatible",
        "session_folder_name": session_folder.name,
        "flight_duration_seconds": 0.0,
        "armed_duration_seconds": 0.0,
        "max_altitude_metres": 0.0,
        "min_altitude_metres": 0.0,
        "max_horizontal_speed_metres_per_second": 0.0,
        "max_vertical_speed_metres_per_second": 0.0,
        "flight_modes": [],
        "battery": {
            "initial_voltage_volts": None,
            "final_voltage_volts": None,
            "voltage_drop_volts": 0.0,
        },
        "topics": [],
    }

    topics_dir = session_folder / "topics"
    if not topics_dir.exists():
        topics_dir = session_folder

    # 1. Parse vehicle local position for altitude, speed, and duration
    pos_file = None
    for candidate_name in [
        "fmu__out__vehicle_local_position.csv.gz",
        "fmu__out__vehicle_local_position.csv",
        "vehicle_local_position.csv.gz",
        "vehicle_local_position.csv",
    ]:
        p = topics_dir / candidate_name
        if p.exists():
            pos_file = p
            break

    if pos_file:
        first_time_seconds: Optional[float] = None
        last_time_seconds: Optional[float] = None
        max_height_metres = -999.0
        min_height_metres = 999.0
        max_xy_speed_metres_per_second = 0.0
        max_z_speed_metres_per_second = 0.0

        with _open_csv_text_reader(pos_file) as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    # In PX4 NED frame, 'z' is downward, so altitude above launch pad is -z
                    z_ned_metres = float(row.get("z", 0.0))
                    height_metres = -z_ned_metres
                    if height_metres > max_height_metres:
                        max_height_metres = height_metres
                    if height_metres < min_height_metres:
                        min_height_metres = height_metres

                    vx_metres_per_second = float(row.get("vx", 0.0))
                    vy_metres_per_second = float(row.get("vy", 0.0))
                    vz_metres_per_second = float(row.get("vz", 0.0))

                    xy_speed = math.sqrt(
                        vx_metres_per_second ** 2 + vy_metres_per_second ** 2
                    )
                    z_speed = abs(vz_metres_per_second)

                    if xy_speed > max_xy_speed_metres_per_second:
                        max_xy_speed_metres_per_second = xy_speed
                    if z_speed > max_z_speed_metres_per_second:
                        max_z_speed_metres_per_second = z_speed

                    time_val = None
                    if "seconds_since_start" in row:
                        time_val = float(row["seconds_since_start"])
                    elif "timestamp" in row:
                        time_val = float(row["timestamp"]) / 1_000_000.0

                    if time_val is not None:
                        if first_time_seconds is None:
                            first_time_seconds = time_val
                        last_time_seconds = time_val
                except (ValueError, KeyError):
                    continue

        if first_time_seconds is not None and last_time_seconds is not None:
            metadata["flight_duration_seconds"] = round(
                last_time_seconds - first_time_seconds, 2
            )
        if max_height_metres != -999.0:
            metadata["max_altitude_metres"] = round(max_height_metres, 2)
            metadata["min_altitude_metres"] = round(min_height_metres, 2)
        metadata["max_horizontal_speed_metres_per_second"] = round(
            max_xy_speed_metres_per_second, 2
        )
        metadata["max_vertical_speed_metres_per_second"] = round(
            max_z_speed_metres_per_second, 2
        )

    # 2. Parse vehicle status for arming duration and flight modes
    status_file = None
    for candidate_name in [
        "fmu__out__vehicle_status_v1.csv.gz",
        "fmu__out__vehicle_status_v1.csv",
        "vehicle_status.csv.gz",
        "vehicle_status.csv",
    ]:
        p = topics_dir / candidate_name
        if p.exists():
            status_file = p
            break

    if status_file:
        arming_timestamps: List[float] = []
        with _open_csv_text_reader(status_file) as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    arming_state = int(row.get("arming_state", 1))
                    time_val = None
                    if "seconds_since_start" in row:
                        time_val = float(row["seconds_since_start"])
                    elif "timestamp" in row:
                        time_val = float(row["timestamp"]) / 1_000_000.0

                    # arming_state == 2 corresponds to ARMING_STATE_ARMED in PX4
                    if arming_state == 2 and time_val is not None:
                        arming_timestamps.append(time_val)
                except (ValueError, KeyError):
                    continue

        if arming_timestamps:
            armed_seconds = arming_timestamps[-1] - arming_timestamps[0]
            metadata["armed_duration_seconds"] = round(max(0.0, armed_seconds), 2)

    # 3. Parse battery status
    batt_file = None
    for candidate_name in [
        "fmu__out__battery_status.csv.gz",
        "fmu__out__battery_status.csv",
        "battery_status.csv.gz",
        "battery_status.csv",
    ]:
        p = topics_dir / candidate_name
        if p.exists():
            batt_file = p
            break

    if batt_file:
        first_voltage: Optional[float] = None
        last_voltage: Optional[float] = None
        with _open_csv_text_reader(batt_file) as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    volts = float(row.get("voltage_v", 0.0))
                    if volts > 0.5:
                        if first_voltage is None:
                            first_voltage = volts
                        last_voltage = volts
                except (ValueError, KeyError):
                    continue

        if first_voltage and last_voltage:
            metadata["battery"]["initial_voltage_volts"] = round(first_voltage, 2)
            metadata["battery"]["final_voltage_volts"] = round(last_voltage, 2)
            metadata["battery"]["voltage_drop_volts"] = round(
                first_voltage - last_voltage, 2
            )

    return metadata


def convert_session_to_parquet(
    session_source_dir: Path,
    output_destination_dir: Path,
    use_pyarrow: bool = True,
) -> Dict[str, Any]:
    """Convert an entire session of CSV/CSV.gz telemetry into per-topic Parquet files

    and extract Flight Review v2 metadata.json and manifest.json.
    """
    output_destination_dir.mkdir(parents=True, exist_ok=True)
    topics_output_dir = output_destination_dir / "topics"
    topics_output_dir.mkdir(parents=True, exist_ok=True)

    topics_source_dir = session_source_dir / "topics"
    if not topics_source_dir.exists():
        topics_source_dir = session_source_dir

    manifest: Dict[str, Any] = {
        "format": "px4-flight-review-v2-parquet",
        "compression": "zstd",
        "topics": {},
    }

    # Extract metadata summary
    metadata = extract_flight_summary_metadata(session_source_dir)

    # Copy ancillary session files if present
    for extra_file in ["events.jsonl", "events.jsonl.gz", "session.json", "session.json.gz", "computer_health.csv", "computer_health.csv.gz"]:
        source_extra = session_source_dir / extra_file
        if source_extra.exists():
            shutil.copy2(source_extra, output_destination_dir / extra_file)

    topic_files = list(topics_source_dir.glob("*.csv")) + list(topics_source_dir.glob("*.csv.gz"))
    print(f"[*] Found {len(topic_files)} topic telemetry stream(s) in {session_source_dir.name}")

    has_pyarrow_or_pandas = False
    try:
        import pandas as pd
        has_pyarrow_or_pandas = True
    except ImportError:
        try:
            import pyarrow as pa
            import pyarrow.csv as pcsv
            import pyarrow.parquet as pq
            has_pyarrow_or_pandas = True
        except ImportError:
            has_pyarrow_or_pandas = False

    for topic_path in sorted(topic_files):
        clean_topic = clean_topic_filename_to_name(topic_path.name)
        safe_topic_stem = topic_path.name.replace(".gz", "").replace(".csv", "")
        parquet_output_path = topics_output_dir / f"{safe_topic_stem}.parquet"

        row_count = 0
        min_time_seconds = None
        max_time_seconds = None

        if has_pyarrow_or_pandas and use_pyarrow:
            try:
                # Use pyarrow/pandas for high speed conversion to ZSTD Parquet
                if "pandas" in sys.modules or "pd" in locals():
                    df = pd.read_csv(topic_path)
                    row_count = len(df)
                    if "seconds_since_start" in df.columns and row_count > 0:
                        min_time_seconds = float(df["seconds_since_start"].iloc[0])
                        max_time_seconds = float(df["seconds_since_start"].iloc[-1])
                    df.to_parquet(parquet_output_path, compression="zstd", index=False)
                else:
                    import pyarrow.csv as pcsv
                    import pyarrow.parquet as pq
                    table = pcsv.read_csv(topic_path)
                    row_count = table.num_rows
                    pq.write_table(table, parquet_output_path, compression="zstd")
            except Exception as conversion_error:
                print(f"[!] Warning: Could not convert {topic_path.name} to parquet: {conversion_error}")
        else:
            # Fallback: Count rows and copy / keep compressed
            with _open_csv_text_reader(topic_path) as handle:
                reader = csv.reader(handle)
                header = next(reader, None)
                for row in reader:
                    row_count += 1

        manifest["topics"][clean_topic] = {
            "parquet_file": f"topics/{safe_topic_stem}.parquet" if has_pyarrow_or_pandas else f"topics/{topic_path.name}",
            "row_count": row_count,
            "min_time_seconds": min_time_seconds,
            "max_time_seconds": max_time_seconds,
        }
        metadata["topics"].append(clean_topic)

    # Write metadata.json and manifest.json
    metadata_path = output_destination_dir / "metadata.json"
    with open(metadata_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    manifest_path = output_destination_dir / "manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    print(f"[+] Formatted session saved to: {output_destination_dir}")
    print(f"    - Summary Metadata: {metadata_path.name} (Flight: {metadata['flight_duration_seconds']}s, Armed: {metadata['armed_duration_seconds']}s, Max Alt: {metadata['max_altitude_metres']}m)")
    print(f"    - Manifest: {manifest_path.name} ({len(manifest['topics'])} topics)")

    return metadata


def run_flight_review_rs_cli_on_ulog(
    ulog_file_path: Path,
    output_destination_dir: Path,
) -> bool:
    """Execute the official flight-review-rs CLI binary if installed on the host

    to convert a raw Pixhawk .ulg binary into Parquet and metadata.json.
    """
    flight_review_bin = shutil.which("flight-review")
    if not flight_review_bin:
        print("[!] Note: 'flight-review' executable from px4/flight-review-rs not found on PATH.")
        print("    To install it via Rust Cargo:")
        print("        cargo install --git https://github.com/px4/flight-review-rs flight-review")
        return False

    output_destination_dir.mkdir(parents=True, exist_ok=True)
    command = [
        flight_review_bin,
        "convert",
        str(ulog_file_path),
        "--output",
        str(output_destination_dir),
    ]

    print(f"[*] Running: {' '.join(command)}")
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode == 0:
        print(f"[+] Successfully converted {ulog_file_path.name} with flight-review-rs CLI!")
        return True
    else:
        print(f"[!] flight-review CLI returned error ({result.returncode}):\n{result.stderr}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Format flight telemetry into the PX4 Flight Review v2 (flight-review-rs) standard."
    )
    parser.add_argument(
        "--input",
        "-i",
        required=True,
        help="Path to a flight session directory (containing 'topics/') or a raw .ulg file.",
    )
    parser.add_argument(
        "--output",
        "-o",
        required=True,
        help="Destination directory for formatted Flight Review v2 artifacts.",
    )

    args = parser.parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        print(f"[!] Error: Input path does not exist: {input_path}")
        sys.exit(1)

    if input_path.is_file() and input_path.suffix.lower() == ".ulg":
        success = run_flight_review_rs_cli_on_ulog(input_path, output_path)
        if not success:
            sys.exit(1)
    elif input_path.is_dir():
        convert_session_to_parquet(input_path, output_path)
    else:
        print(f"[!] Error: Unsupported input format: {input_path}")
        sys.exit(1)


if __name__ == "__main__":
    main()

