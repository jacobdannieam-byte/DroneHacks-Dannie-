# Formal Technical Evaluation & Architectural Modification Proposal
**Author:** Gemini 4 Argon (Google Frontier Reasoning Model)  
**Role:** Principal Autonomous Flight Systems Architect & Senior Roboticist  
**Subject:** DroneHacks Autonomous Navigation Stack ("Published-Layout Route Map + Live Perception")  
**Target Platform:** Jetson Orin / Linux Companion Container (ROS 2 Jazzy, Micro XRCE-DDS, PX4 Autopilot, cuVSLAM)  
**Baseline Codebase:** [`refnav/`](../refnav/core.py) & [`onboard/`](../onboard/README.md)

---

## Executive Architectural Summary

The team’s core thesis—**rejecting heavy global 3D mapping (`nvblox`, `OctoMap`, 3D A*) in favor of a topological/metric prior (`config/course.yaml`) coupled with live, ego-centric perception and bounded local trajectory corrections**—is **architecturally sound** and directly mitigates the real-time factor (RTF 0.53) compute bottleneck documented in [`onboard/README.md`](../onboard/README.md#L3-L10).

However, when evaluated against the actual implementation in [`refnav/core.py`](../refnav/core.py), [`refnav/depth.py`](../refnav/depth.py), [`onboard/slam/cuvslam_lib_node.py`](../onboard/slam/cuvslam_lib_node.py), and [`onboard/vio/ev_bridge.py`](../onboard/vio/ev_bridge.py), the 13-step proposal contains **five critical failure modes** that will ground or crash the vehicle if not addressed:

1. **Unobserved-Corridor Deadlock in the Dark Tunnel & Featureless Maze:** [`Mission._clearance`](../refnav/core.py#L453-L477) and [`Mission.on_depth`](../refnav/core.py#L229-L233) enforce `min_valid_frac >= 0.30` and `min_corridor_coverage >= 0.60`, with a hard AUTO_LAND trigger after `depth_lost_land_s = 3.0` s ([`refnav/core.py`](../refnav/core.py#L334-L335)). Passive stereo matching in a textureless dark tunnel returns `NaN` across $>75\%$ of the frame, guaranteeing an unintended mid-tunnel landing unless stage-conditioned perception contracts are introduced.
2. **Silent Inertial Drift Masking in `cuvslam_lib_node.py`:** [`CuvslamLibNode.__init__`](../onboard/slam/cuvslam_lib_node.py#L94-L97) defaults to `imu_only_policy: "publish"` ([`onboard/config/vio.yaml`](../onboard/config/vio.yaml#L16)). When visual observations drop below `min_visual_obs = 10` in the dark tunnel, cuVSLAM continues publishing IMU dead-reckoning poses. Because [`EvBridge._on_odom`](../onboard/vio/ev_bridge.py#L79-L108) only subscribes to `/visual_slam/tracking/odometry` and not `/visual_slam/status` or `/visual_slam/vis/observations_count`, `/estimator/healthy` remains `True` while double-integrated accelerometer bias silently drifts the vehicle into tunnel walls.
3. **PX4 EKF2 Reset Counter Hazards:** Applying maze opening corrections by mutating the external vision pose or triggering [`EvBridge`](../onboard/vio/ev_bridge.py#L98-L99) `jump_reset_m` (`1.0 m`) increments `reset_counter`, causing EKF2 state resets and velocity/attitude transients during tight aperture traversal. Corrections **must** occur in the guidance frame, never by spoofing or jumping `/fmu/in/vehicle_visual_odometry`.
4. **Stop-and-Turn / Single-Axis Carrot Limitation:** [`Mission._go`](../refnav/core.py#L479-L502) only advances a scalar arc-length `self.s` along a fixed 1D ray `leg_dir`, and any confirmed obstacle closer than `stop_m = 0.9` m unconditionally calls [`self._land(...)`](../refnav/core.py#L493). Navigating a maze opening and chicane requires a 2D/3D lateral deviation profile `(s, d_lat)` and stage-aware obstacle response (hold/re-center vs. abort).
5. **Coding Standard Compliance (`docs/READABLE_CODE.md`):** [`CLAUDE.md`](../CLAUDE.md#L3-L4) mandates strict adherence to [`docs/READABLE_CODE.md`](./READABLE_CODE.md) (explicit unit suffixes `_metres`, `_seconds`, `_radians`, provenance-labeled constants, single-screen functions, and plain-English explanations for non-robotics engineers).

---

## 1. Depth Processing Architecture

### 1.1 Critical Evaluation: OpenCV `StereoSGBM` vs. GPU-Accelerated Stereo (VPI / CUDA / Isaac ROS)

| Dimension | CPU OpenCV `cv2.StereoSGBM` | NVIDIA VPI Stereo Disparity (CUDA/PVA) | Isaac ROS `isaac_ros_ess` / `StereoDisparityNode` |
| :--- | :--- | :--- | :--- |
| **Compute Backend** | ARM Cortex-A78AE CPU (NEON) / x86-64 Sim CPU | CUDA GPU + Programmable Vision Accelerator (PVA) | TensorRT / CUDA (DNN or SGM) |
| **Latency @ 640×400, $D=64$** | **6.5 – 11.5 ms** (3/5-path SGBM, single/dual thread) | **1.4 – 2.2 ms** (CUDA backend) | **8 – 18 ms** (ESS DNN); **2.5 ms** (SGM) |
| **GPU Contention with cuVSLAM** | **Zero.** Leaves GPU 100% dedicated to `cuvslam.Tracker.track()` | **Moderate.** Shares GPU SMs & L2 cache with cuVSLAM | **High.** TensorRT context + Nitros memory pool overhead |
| **Portability (x86 Sim $\leftrightarrow$ Jetson)** | **100% Identical** (`opencv-python` / `cv2` standard) | **Breaks portability** if VPI Python bindings differ | **High friction**; requires Nitros graph & matching TensorRT engine builds per SM |
| **Failure Mode on Textureless Walls** | Predictable invalidation (`uniquenessRatio`, `disp12MaxDiff`) $\rightarrow$ `NaN` | Requires explicit confidence threshold tuning; over-interpolates across low texture | DNN hallucination risk on unseen dark-tunnel geometry (violates fail-closed invariant) |

#### Architectural Verdict
**Adopt a deterministic CPU `cv2.StereoSGBM` pipeline as the primary stereo engine in `autonomy/stereo.py`, preserving a clean NumPy interface.**
- In simulation and on Jetson Orin, `cuvslam_lib_node.py` already consumes GPU resources. Adding a second heavy GPU process creates stream contention that risks exceeding the $300\text{ ms}$ sample age gate in `ev_bridge.py`.
- At $640 \times 400$ resolution ($256,000$ pixels) @ 15 Hz ($66.7\text{ ms}$ budget), CPU `StereoSGBM` with `numDisparities=64, blockSize=5` executes in **$< 9\text{ ms}$** on one CPU core, consumes **0 MB VRAM**, and behaves identically in simulation, unit tests, and hardware.

---

### 1.2 Memory & Compute Budget (640×400 @ 15 Hz + IMU @ 480 Hz)

Frame period at 15 Hz is **$66.67\text{ ms}$**. The allocation:

| Subsystem / Node | Hardware Unit | Time Budget (ms / frame) | Memory Footprint | Contention Mitigation |
| :--- | :--- | :--- | :--- | :--- |
| **`cuvslam_lib_node.py`** | GPU (CUDA) + 1 CPU core | $8.0\text{ ms}$ p50 / $18.0\text{ ms}$ p99 | ~380 MB VRAM, 140 MB RAM | Exclusive GPU consumer; IMU queue capped at 5000 samples. |
| **`ev_bridge.py`** | CPU | $0.4\text{ ms}$ @ 15 Hz | 25 MB RAM | Pure Python/math 6×6 covariance rotation. |
| **`autonomy/stereo.py`** (CLAHE + SGBM + Speckle) | CPU (OpenCV thread) | $11.5\text{ ms}$ max | 45 MB RAM (pre-allocated) | Pre-allocate buffers. **Exclude WLS filter** (saves 14 ms CPU). |
| **`refnav/depth.py` + `autonomy/clearance.py`** | CPU (NumPy vectorized) | $1.8\text{ ms}$ @ `stride=8` | 18 MB RAM | Stride-8 subsampling reduces ray count to 3,264 rays. |
| **`refnav/core.py` + Guidance** | CPU (20 Hz timer) | $0.3\text{ ms}$ / tick | < 10 MB RAM | Zero dynamic allocations in the 20 Hz control loop. |
| **Total Per-Frame Compute** | **1 GPU + 2.5 CPU cores** | **$22.0\text{ ms} \ll 66.7\text{ ms}$ (33% duty cycle)** | **~380 MB VRAM / ~240 MB RAM** | **Leaves 67% frame headroom.** |

---

### 1.3 Concrete Stereo & Depth Processing Pipeline (`autonomy/stereo.py`)

1. **Rectification & Epipolar Validation:** Inspect `CameraInfo` at startup: verify $|f_{x,L} - f_{x,R}| < 0.5\text{ px}$, $|c_{y,L} - c_{y,R}| < 0.5\text{ px}$, and extract baseline $B = -T_x / f_x$ ($B \approx 0.08 - 0.12\text{ m}$).
2. **Pre-Conditioning (CLAHE):** Apply `cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))` to both eyes *only inside `autonomy/stereo.py`* (never mutate images sent to cuVSLAM to avoid optical flow flicker).
3. **Disparity Computation (`StereoSGBM`):**
   - `minDisparity = 0`, `numDisparities = 64` (covers near range down to $0.62\text{ m}$; use 80 for $0.45\text{ m}$).
   - `blockSize = 5`, `P1 = 200`, `P2 = 800`.
   - `uniquenessRatio = 12` (strictly rejects ambiguous matches into `NaN`).
   - `disp12MaxDiff = 1` (half-pixel left-right consistency check, eliminating occlusion ghosts at doorway edges).
   - `mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY` (runs in ~5.2 ms vs 9.5 ms for 5-way).
4. **Post-Filtering (Why WLS Must Be Excluded in Flight):**
   - **Hazard:** WLS requires computing a second right-to-left matcher (doubling CPU time) and **hallucinates planar surfaces across unmatched `NaN` holes**, risking turning an open doorway into a false wall or vice versa.
   - Use **deterministic invalidation filters instead**:
     - `speckleWindowSize = 100`, `speckleRange = 16` ($1.0\text{ px}$).
     - Disparity step edge erosion ($2\text{ px}$ radius around $|\Delta d| > 3.0\text{ px}$).
     - Blank the left 64 columns where the left eye has no stereo overlap.
5. **Metric Depth Conversion:**
   $$z = \frac{f_x \cdot B}{d_{\text{raw}} / 16.0}$$
   Clamped outside $[0.30\text{ m}, 6.0\text{ m}]$ to `np.nan` so missing returns are strictly **unobserved**, never free space.

---

### 1.4 Handling Pathological Indoor Conditions

- **Featureless Walls in the Maze:** In [`Mission._clearance`](../refnav/core.py#L463-L471), `clearance()` was previously only evaluated if `cov >= min_corridor_coverage`. When facing a close, textureless wall, coverage drops below 0.60, causing `_clearance` to ignore the obstacle! **Correction:** Always evaluate `clearance()` on every frame. If an obstacle is detected within `stop_m` across `confirm_frames`, stop immediately regardless of coverage. Additionally, use floor-wall crease contrast to maintain horizontal depth support across uniform walls.
- **Low-Light Tunnel Surfaces:** Compute a per-frame `PhotometricMetrics` struct `(mean_luma, p90_luma, spatial_gradient)`. Gate CLAHE based on noise floor to prevent locking onto sensor dark noise, and transition to stage-conditioned tunnel envelopes.

---

## 2. SLAM Integration & Trajectory Anchoring

### 2.1 Frame Architecture: Anchoring cuVSLAM to `config/course.yaml`

- **Course Frame ($\mathcal{F}_C$):** Right-handed **FLU** metric frame anchored at the center of the Takeoff Pad ($+X_C$ Forward along takeoff corridor, $+Y_C$ Left, $+Z_C$ Up).
- **PX4 Local Control Frame ($\mathcal{F}_E$):** PX4 `/fmu/out/vehicle_local_position_v1` **NED** frame.
- **Rigid Anchor Transform ($T_{E \leftarrow C}$):** Locked at the `wait` $\rightarrow$ `arm` transition using `self.home = (n_0, e_0, d_0, \psi_0)`:
  $$\begin{bmatrix} n \\ e \end{bmatrix} = \begin{bmatrix} n_0 \\ e_0 \end{bmatrix} + \begin{bmatrix} \cos\psi_0 & \sin\psi_0 \\ \sin\psi_0 & -\cos\psi_0 \end{bmatrix} \begin{bmatrix} x_C \\ y_C \end{bmatrix}, \qquad d = d_0 - z_C, \qquad \psi_E = \text{wrap}(\psi_0 - \psi_C)$$
  Every coordinate in `config/course.yaml` maps bijectively into PX4 local NED without modifying `ev_bridge.py` or triggering EKF2 resets.

---

### 2.2 Managing Cumulative VIO Drift via Landmark Anchors (No Global Grid)

1. **Maze Aperture Planes:** Wall at known $x_C = x_{\text{gate}}$ with expected opening $y_C = y_{\text{gate}}$. Observing opening center at range $z_{\text{obs}}$ and bearing $\beta_{\text{obs}}$ directly measures longitudinal drift $\delta x$ and lateral drift $\delta y$.
2. **Tunnel Entrance & Exit Portals:** Acts as a 2D bounding anchor prior to entering the feature-poor tunnel and immediately upon exiting.
3. **Rangefinder Altitude Decoupling:** Rangefinder `dist_bottom` cross-checks vertical position $d$, preventing double-integrated vertical VIO drift from causing floor/ceiling collisions.

---

### 2.3 Mathematical Formulation for Bounded Opening Corrections (`autonomy/corrections.py`)

Given a nominal leg from $\mathbf{p}_{\text{start}}$ to $\mathbf{p}_{\text{end}}$ with unit tangent $\mathbf{u}_\parallel = (\cos\psi_{\text{leg}}, \sin\psi_{\text{leg}})^\top$ and normal $\mathbf{u}_\perp = (-\sin\psi_{\text{leg}}, \cos\psi_{\text{leg}})^\top$:

1. **Raw Measurement Model:**
   $$\mathbf{g}_k = \mathbf{p}_{\text{veh}}(t_k) + R_z(\psi(t_k)) \begin{bmatrix} x_{\text{wall}} \\ y_{\text{gap}} \end{bmatrix}, \qquad e_{\perp, k} = (\mathbf{g}_k - \mathbf{p}_{\text{start}})^\top \mathbf{u}_\perp$$
2. **Gating & EWMA Smoothing:**
   $$\hat{\delta}_\perp^{(k)} = \text{clip}\!\left((1 - \alpha)\hat{\delta}_\perp^{(k-1)} + \alpha \, e_{\perp, k}, \; -\Delta_{\max}, \; +\Delta_{\max}\right), \qquad \alpha = 0.35, \quad \Delta_{\max} \le 0.25\text{ m}$$
   - Require $N_{\text{confirm}} \ge 2$ consecutive frames within $0.5\text{ s}$ before applying.
   - When approaching within camera minimum range ($s_{\text{wall}} - s < 0.65\text{ m}$), **freeze (latch)** $\hat{\delta}_\perp$ until $0.5\text{ m}$ past the wall.
3. **$C^2$-Continuous Carrot Blending:**
   Rate-limit the applied lateral carrot offset $\delta_\perp(t)$ at the 20 Hz control loop:
   $$\dot{\delta}_{\perp,\text{cmd}}(t) = \text{clip}\!\left(\frac{\hat{\delta}_\perp - \delta_\perp(t)}{\tau_p}, \; -v_{\perp,\max}, \; +v_{\perp,\max}\right), \qquad v_{\perp,\max} = 0.25\text{ m/s}$$
   $$\mathbf{p}_{\text{sp}}(t) = \mathbf{p}_{\text{start}} + s(t)\,\mathbf{u}_\parallel + \delta_\perp(t)\,\mathbf{u}_\perp$$
   - Maximum position step per 20 Hz tick is $0.25 \times 0.05 = 1.25\text{ cm}$ (zero control discontinuity).
   - Once through the opening, the confirmed offset is absorbed into the stage anchor offset so subsequent waypoints align seamlessly.

---

## 3. Dark Tunnel Qualification & Fallback Strategy

### Exact Transition & Gating Criteria

| Criterion | Safe Passage (`TUNNEL_GUARDED_TRANSIT`) | Hold (`CORRIDOR_HOLD`) | Immediate Abort / Land (`ABORT_AUTO_LAND`) |
| :--- | :--- | :--- | :--- |
| **Pre-Entry Qualification** | Within $\pm 0.15\text{ m}$ lateral, $\pm 5^\circ$ yaw of tunnel axis; clearance confirmed $\ge 2.5\text{ m}$; `n_obs >= 25`. | Entry depth coverage $< 0.60$ outside portal (`t_hold <= 3.0 s`). | Confirmed obstacle at portal ($\text{clr} < 0.9\text{ m}$ over 2 frames). |
| **cuVSLAM Features (`n_obs`)** | `n_obs >= 10` (visual), **OR** $t_{\text{inertial}} \le 1.2\text{ s}$ (max drift $< 0.02\text{ m}$). | $1.2\text{ s} < t_{\text{inertial}} \le 1.8\text{ s}$: freeze carrot advancement while holding hover setpoint. | $t_{\text{inertial}} > 1.8\text{ s}$ **OR** `/estimator/healthy` false/stale for $> 1.0\text{ s}$. |
| **Depth Coverage** | Per-stage tunnel envelope (`valid_frac >= 0.18`, `cov >= 0.35`); no obstacle within $0.9\text{ m}$. | `valid_frac < 0.18` or `cov < 0.35` for $\le 3.0\text{ s}$ (carrot pauses, aircraft holds). | No usable depth frame for $> 3.0\text{ s}$, **OR** confirmed obstacle $\text{clr} < 0.9\text{ m}$. |
| **Attitude & Pilot** | $|\text{roll}|, |\text{pitch}| \le 12^\circ$; cross-track offset clamped to $\pm 0.15\text{ m}$. | Single-frame stereo spike ($\text{clr} < 0.9\text{ m}$ on 1 frame). | PX4 leaves `NAV_OFFBOARD` (`14`) (immediately yield). |

---

## 4. Concrete Next Engineering Steps (Roadmap)

### Step 1: Course Configuration & Stage Envelopes (`config/course.yaml` & `refnav/course.py`)
- Define `config/course.yaml` with stage envelopes (`corridor_half_width_metres`, `stop_distance_metres`, `min_valid_depth_fraction`, `min_corridor_coverage_fraction`, `max_lateral_correction_metres`, `max_inertial_coast_seconds`).
- Implement `refnav/course.py` (pure Python, loading `course.yaml` or legacy `route.json`).
- Write unit tests in `tests/test_course_config.py`.

### Step 2: Stereo Disparity & Multi-Bin Ray Scanning (`autonomy/stereo.py` & `refnav/depth.py`)
- Implement pure CPU SGBM with CLAHE in `autonomy/stereo.py`.
- Extend `DepthFrame` in `refnav/depth.py` with multi-bin lateral scanning (`corridor_bins`), preserving all existing functions.
- Write unit tests in `tests/test_stereo_and_depth_bins.py`.

### Step 3: Multi-Bin Clearance & Bounded Opening Detector (`autonomy/clearance.py` & `autonomy/openings.py`)
- Create `autonomy/clearance.py` to evaluate structured corridor slices.
- Create `autonomy/openings.py` to detect opening offsets with bounded EWMA smoothing ($\alpha = 0.35$, $\Delta_{\max} \le 0.25\text{ m}$).
- Write unit tests in `tests/test_openings_and_clearance.py`.

### Step 4: Core Integration Without Regressions (`refnav/core.py`)
- In `refnav/core.py`, add optional `stage_envelopes`, `max_lateral_nudge_m`, and `on_vio_status` on `Mission`.
- In `Mission._go`, apply the bounded EWMA offset perpendicular to `leg_dir`.
- Ensure all 18 existing test functions (21 pytest items) in `tests/test_refnav_core.py` pass without modification.
- Add new stage-aware tests in `tests/test_refnav_stage_and_nudge.py`.

### Step 5: VIO Tracking Status & Safety Gating (`cuvslam_lib_node.py` & `ev_bridge.py`)
- In `cuvslam_lib_node.py`, ensure observation count and tracking mode are published in `_processing_details`.
- In `ev_bridge.py`, declare `max_inertial_coast_s = 1.8` s and transition `/estimator/healthy` to `False` if inertial coasting exceeds this window.

---

## 5. The Resulting Agent Prompt

```markdown
# ROLE & OBJECTIVE
You are an autonomous robotics software engineer working on the DroneHacks companion repository. Your objective is to execute **Phase 1 (Steps 1 through 5)** of the architectural upgrade:
1. Course configuration schema & loader (`config/course.yaml`, `refnav/course.py`)
2. Pure stereo SGBM helper & multi-bin depth scan profile (`autonomy/stereo.py`, `refnav/depth.py`)
3. Multi-bin corridor clearance & bounded EWMA opening detector (`autonomy/clearance.py`, `autonomy/openings.py`)
4. Stage-aware & lateral-nudge integration into `refnav/core.py` (with 100% backward compatibility)
5. Visual-feature starvation gating in `onboard/slam/cuvslam_lib_node.py` and `onboard/vio/ev_bridge.py`

---

# MANDATORY REPOSITORY RULES & INVARIANTS
Before writing or editing any file, read and obey:
1. `AGENTS.md`, `AI_CONTEXT.md`, `CLAUDE.md`, and `docs/READABLE_CODE.md`.
2. **NO FLIGHT EXECUTION**: Never run flight scripts (`main.py`, `onboard/examples/takeoff_land.py`, or `refnav/node.py`) against a live simulator or hardware without explicit team flight plan approval. Run ONLY unit tests (`python3 -m pytest`).
3. **READABLE CODE STANDARD (`docs/READABLE_CODE.md`)**:
   - Every new folder with code must have a `README.md` explaining what the folder is for, which file to read first, and in what order to read the rest.
   - Every file must open with a plain-English docstring stating: (a) what the file does, (b) how it fits with the other files, and (c) what it deliberately does not do.
   - Include physical units in all new variable, parameter, and function names (`_metres`, `_seconds`, `_radians`, `_degrees`, `_mps`).
   - Every numerical constant must be named at the top of the file with its unit and a comment stating its source (measured, estimated, from PX4 source code, from rulebook, or our own choice).
   - Define drone terms (e.g., "NED", "Offboard", "disparity", "EWMA") in a one-sentence comment the first time they appear.
   - Keep functions short (fitting on one screen) with shallow nesting and early returns.
4. **PURE CORE & SAFETY INVARIANTS**:
   - `refnav/core.py`, `refnav/depth.py`, `refnav/course.py`, `autonomy/stereo.py`, `autonomy/clearance.py`, and `autonomy/openings.py` must be **100% PURE**: no `rclpy` imports, no system clock reads (`time.time()`), no simulator truth, and no host/lab API calls.
   - Never weaken estimator health checks, depth freshness checks, or corridor coverage checks. Unobserved space is **unknown space, never free space**.
   - All 18 existing test functions (21 pytest items) in `tests/test_refnav_core.py` must pass unmodified at every step.

---

# STEP-BY-STEP IMPLEMENTATION SPECIFICATION

## Step 0: Baseline Verification
1. Inspect `refnav/core.py`, `refnav/depth.py`, `tests/test_refnav_core.py`, `onboard/slam/cuvslam_lib_node.py`, and `onboard/vio/ev_bridge.py`.
2. Run `python3 -m pytest tests/ -q` and confirm all 21 existing test items pass before making any changes.

## Step 1: `config/course.yaml` & `refnav/course.py`
1. Create `config/course.yaml` defining course stages in the takeoff-pad horizontal Forward-Left-Up (`FLU_pad`) frame:
   - Include `course_name`, `nominal_flight_height_metres`, and a `stages` list where each stage specifies:
     `stage_id`, `stage_type` (`"corridor" | "aperture" | "tunnel" | "slalom"`), `target_forward_metres`, `target_left_metres`, `speed_metres_per_second`, `corridor_half_width_metres`, `stop_distance_metres`, `min_valid_depth_fraction`, `min_corridor_coverage_fraction`, `max_lateral_correction_metres`, and `max_inertial_coast_seconds`.
2. Create `refnav/course.py` (pure Python):
   - Define `@dataclass(frozen=True) class StageEnvelope` with explicit unit-suffixed fields matching `config/course.yaml`.
   - Implement `load_course_definition(file_path)` loading either `config/course.yaml` or legacy `route.json` and returning `(route_legs_metres, stage_envelopes)`.
   - Validate all numerical fields and raise descriptive full-sentence `ValueError` messages.
3. Add unit tests in `tests/test_course_config.py` covering valid YAML loading, legacy `route.json` compatibility, and error cases.

## Step 2: `autonomy/stereo.py` & `refnav/depth.py`
1. Create `autonomy/README.md` and `autonomy/__init__.py`.
2. Create `autonomy/stereo.py` (pure NumPy/OpenCV):
   - Implement CLAHE contrast normalization and StereoSGBM disparity-to-depth conversion:
     `compute_depth_from_rectified_stereo_metres(left_mono8, right_mono8, focal_length_pixels, baseline_metres, min_depth_metres=0.3, max_depth_metres=10.0) -> np.ndarray` returning `float32` metres with `np.nan` for invalid pixels.
3. In `refnav/depth.py`:
   - Preserve all existing functions and classes with identical signatures and behavior.
   - Add `corridor_bins(...)` on `DepthFrame` partitioning the horizontal corridor `[-half_width_m, +half_width_m]` into `num_bins` lateral sub-corridors and returning per-bin `(bin_center_offset_metres, bin_clearance_metres, bin_coverage_fraction)`.
4. Add unit tests in `tests/test_stereo_and_depth_bins.py` verifying synthetic disparity conversion, NaN masking, and multi-bin clearance accuracy.

## Step 3: `autonomy/clearance.py` & `autonomy/openings.py`
1. Create `autonomy/clearance.py`:
   - Implement `evaluate_corridor_slices(...)` returning structured `CorridorSlice` objects (`lateral_offset_metres`, `clearance_metres`, `coverage_fraction`, `is_observed`).
2. Create `autonomy/openings.py`:
   - Implement `find_best_opening_offset_metres(...)` returning target lateral offset in metres (or `None` if unobserved).
   - Implement `BoundedLateralFilter` using EWMA ($\alpha = 0.35$) clamped to `[-max_lateral_correction_metres, +max_lateral_correction_metres]`. Decay output toward `0.0` when observations are unconfirmed.
3. Add unit tests in `tests/test_openings_and_clearance.py` testing centered corridors, blocked corridors with clear openings, unobserved ray handling, and EWMA step responses.

## Step 4: Integrate into `refnav/core.py` Without Breaking Existing Tests
1. In `refnav/core.py`:
   - Preserve all existing fields and defaults in `Params`. Add optional fields:
     `stage_envelopes: list = field(default_factory=list)`, `max_lateral_nudge_m: float = 0.0`, `max_inertial_coast_s: float = 1.8`.
   - Add `on_vio_status(self, t, visually_tracked_ok, n_observations=None)` on `Mission`. Trigger `self._land(...)` if inertial coasting exceeds `max_inertial_coast_s` while airborne.
   - Support per-leg stage overrides from `p.stage_envelopes[self.leg]`.
   - In `Mission._go`, apply the bounded EWMA offset perpendicular to `self.leg_dir` only while clearance is confirmed (`>= stop_m`) and observed (`>= min_corridor_coverage`).
2. Add unit tests in `tests/test_refnav_stage_and_nudge.py` testing opening avoidance, tunnel stage envelopes, and clock invariance.

## Step 5: Safety Gating in `onboard/slam/cuvslam_lib_node.py` & `onboard/vio/ev_bridge.py`
1. In `onboard/slam/cuvslam_lib_node.py`:
   - Ensure `"n_observations"` and `"visually_tracked"` are published in `_processing_details` on `/visual_slam/processing_status`.
2. In `onboard/vio/ev_bridge.py`:
   - Declare parameter `max_inertial_coast_s` (default `1.8` s). Transition `/estimator/healthy` to `False` if visual features remain starved (`n_obs < min_visual_obs`) for longer than this threshold.
3. Run `python3 -m pytest tests/ -q` and verify that all original 21 tests plus all new unit tests pass cleanly.
```

