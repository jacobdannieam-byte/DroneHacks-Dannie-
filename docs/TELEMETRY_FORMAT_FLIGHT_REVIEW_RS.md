# Telemetry Formatting Standard: Flight Review v2 (`flight-review-rs`)

This guide explains how we format, store, and query drone telemetry logs using the architectural principles of **PX4's Flight Review v2** ([px4/flight-review-rs](https://github.com/px4/flight-review-rs)).

---

## 1. What is `flight-review-rs`?

Historically, PX4 Flight Review parsed monolithic binary `.ulg` logs dynamically every time a user viewed a plot. For long flights or multi-gigabyte log sets, this caused high server compute loads and slow UI rendering.

**Flight Review v2** is PX4's official rewrite in **Rust**. It introduces a **"Parse Once, Store as Columnar Parquet, Query via DuckDB"** model:

```
Raw Telemetry (.ulg / ROS topics)
            │
            ▼ (Parse Once: Rust / Python Converter)
   ┌──────────────────────────────────────────────┐
   │ Output Session Directory                     │
   │  ├── metadata.json       (Flight summary)    │
   │  ├── manifest.json       (Topic catalog)     │
   │  └── topics/                                 │
   │       ├── vehicle_local_position.parquet     │
   │       ├── vehicle_attitude.parquet           │
   │       ├── actuator_motors.parquet            │
   │       └── ... (per-topic ZSTD Parquet)       │
   └──────────────────────────────────────────────┘
            │
            ▼ (Query Instantly)
   DuckDB / Polars / Pandas / DuckDB-WASM
   (Sub-second columnar queries with HTTP range requests)
```

---

## 2. Why This is the Best Format for Our Telemetry

| Feature | Legacy `.csv.gz` | `flight-review-rs` Parquet + Metadata |
|---|---|---|
| **Query Speed** | Slow: Must decompress the entire file and parse strings | **10x to 50x faster**: Columnar projection reads only requested fields |
| **Memory Usage** | High: Full file parsed into RAM | **Minimal**: Zero-copy memory mapping |
| **Triage / Inspection** | Grepping across files to find takeoff or arming times | **Instant**: `metadata.json` provides an O(1) flight header |
| **Type Safety** | Untyped strings (floats, booleans, arrays as text) | **Strict binary schema** (Float32, Int64, Boolean) |
| **Tooling Support** | Custom CSV scripts | **Universal**: DuckDB, Polars, Pandas, PlotJuggler, DuckDB-WASM |

---

## 3. Directory Layout Specification

Each formatted telemetry run adheres to the following structure:

```
telemetry/skybeta-16/<session_name>/
    ├── metadata.json
    ├── manifest.json
    ├── events.jsonl
    ├── session.json
    ├── computer_health.parquet (or .csv)
    └── topics/
         ├── fmu__out__vehicle_local_position.parquet
         ├── fmu__out__vehicle_attitude.parquet
         ├── fmu__out__vehicle_status_v1.parquet
         ├── fmu__out__battery_status.parquet
         ├── fmu__in__trajectory_setpoint.parquet
         └── ...
```

### A. Summary Metadata (`metadata.json`)
Precomputed upon ingestion so analyzers and web views know what occurred without reading millions of raw rows:
```json
{
  "flight_review_version": "2.0-compatible",
  "session_folder_name": "20261108_021554_practice_flight_recorder",
  "flight_duration_seconds": 189.91,
  "armed_duration_seconds": 0.5,
  "max_altitude_metres": 0.06,
  "min_altitude_metres": 0.03,
  "max_horizontal_speed_metres_per_second": 0.02,
  "max_vertical_speed_metres_per_second": 0.01,
  "flight_modes": ["Offboard", "Hold"],
  "battery": {
    "initial_voltage_volts": 14.84,
    "final_voltage_volts": 14.82,
    "voltage_drop_volts": 0.02
  },
  "diagnostics": {
    "motor_failure": false,
    "ekf_failure": false,
    "rc_loss": false
  }
}
```

### B. Topic Manifest (`manifest.json`)
Lists topic names, schemas, row counts, and valid time ranges:
```json
{
  "format": "px4-flight-review-v2-parquet",
  "compression": "zstd",
  "topics": {
    "fmu/out/vehicle_local_position": {
      "parquet_file": "topics/fmu__out__vehicle_local_position.parquet",
      "row_count": 12450,
      "min_time_seconds": 0.0,
      "max_time_seconds": 189.91
    }
  }
}
```

---

## 4. How to Format Your Logs

We have two sources of logs: the **Jetson companion records** and the **Pixhawk hardware ULogs**. Both can now be formatted into this standard.

### Workflow A: Companion ROS 2 Telemetry (Jetson Logs)
Use the included formatter [`drone_code/format_telemetry.py`](file:///drone_code/format_telemetry.py):

```bash
# Format a specific session into Flight Review v2 format:
python3 drone_code/format_telemetry.py \
  --input telemetry/skybeta-16/20261108_021554_practice_flight_recorder \
  --output telemetry_formatted/20261108_021554
```

### Workflow B: Pixhawk Hardware ULog Logs (`.ulg`)
If you download `.ulg` files from the Pixhawk SD card (via QGroundControl or card reader as documented in `drone_code/COLLECTING_RECORDS.md`), you can use the official `flight-review` CLI from the repo:

1. **Install the CLI via Rust Cargo:**
   ```bash
   cargo install --git https://github.com/px4/flight-review-rs flight-review
   ```

2. **Convert the log:**
   ```bash
   flight-review convert log_001.ulg --output telemetry_formatted/flight_001
   ```
   Or batch-convert an entire folder of `.ulg` files in parallel:
   ```bash
   flight-review convert /path/to/sd_card/logs/ --output telemetry_formatted/all_flights/
   ```

---

## 5. How to Query the Formatted Telemetry

Once converted to Parquet, analyzing flight data requires zero manual CSV unzipping. You can query millions of rows in milliseconds using DuckDB or Pandas.

### Example: Instant Analysis with DuckDB in Python
```python
import duckdb

# Connect to the session
con = duckdb.connect()

# Query position tracking error in 10 milliseconds:
query = """
SELECT 
    pos.seconds_since_start,
    pos.x AS actual_x,
    setp.position_0 AS target_x,
    ABS(pos.x - setp.position_0) AS error_x_metres
FROM 'telemetry_formatted/flight_01/topics/fmu__out__vehicle_local_position.parquet' pos
JOIN 'telemetry_formatted/flight_01/topics/fmu__in__trajectory_setpoint.parquet' setp
  ON ROUND(pos.seconds_since_start, 1) = ROUND(setp.seconds_since_start, 1)
WHERE pos.seconds_since_start BETWEEN 10.0 AND 45.0
ORDER BY pos.seconds_since_start
"""

df = con.execute(query).df()
print(df.head())
```

### Example: Checking Motor Saturation or Imbalance
```python
import duckdb

# Check if any motor was saturated (> 0.95)
saturation_check = """
SELECT 
    seconds_since_start,
    control_0 AS motor_1,
    control_1 AS motor_2,
    control_2 AS motor_3,
    control_3 AS motor_4
FROM 'telemetry_formatted/flight_01/topics/fmu__out__actuator_motors.parquet'
WHERE control_0 > 0.95 OR control_1 > 0.95 OR control_2 > 0.95 OR control_3 > 0.95
"""
print(duckdb.query(saturation_check).df())
```

