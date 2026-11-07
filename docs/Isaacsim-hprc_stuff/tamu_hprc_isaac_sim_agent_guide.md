# Agent Operating Manual: NVIDIA Isaac Sim 5.1.0 on TAMU HPRC (Grace Cluster)

## 1. Mission & System Context

You are an AI agent operating inside an SSH terminal session on the **Texas A&M University High Performance Research Computing (TAMU HPRC) Grace Cluster** (`grace.hprc.tamu.edu`), working alongside an active **HPRC Open OnDemand VNC Desktop** session on a GPU node.

Your goal is to:

1. Safely probe the current cluster node, storage mounts, Singularity container setup, and attach to the active **Open OnDemand VNC Desktop** display (`$DISPLAY`).
2. Get **NVIDIA Isaac Sim 5.1.0** running reliably via Singularity—using headless Python (`/isaac-sim/python.sh`) for fast data processing and launching the full GUI into the user's VNC Desktop when visual inspection is needed.
3. Locate our **Scaniverse `.ply` point cloud** of the real drone course (**596,349 points with RGB color**), convert it into an OpenUSD (`.usd`) stage with proper coordinate orientation ($Z$-up), vertex color rendering, and static voxel/mesh collision proxies so a drone can fly inside it.

### Known User, Asset & Path Context

* **User:** `jacob.dannie.am`
* **Allocated VNC GPU Node:** `r001` (or current `r***`/`c***` node allocated by Open OnDemand)
* **Working Directory:** `$SCRATCH` (`/scratch/user/jacob.dannie.am`)
* **Container Image:** `/sw/hprc/sw/bio/containers/isaac-sim/isaac-sim_5.1.0.sif`
* **Environment Script:** `$SCRATCH/env_isaac-sim.sh`
* **Local Wrapper/Overlay:** `$SCRATCH/my-isaac-sim/isaac-sim/isaac-sim.sh`
* **Course Asset:** A Scaniverse `.ply` point cloud (`596,349` colored points) of the real drone course.

---

## 2. Critical Rules of the TAMU Grace HPRC + VNC Environment

Before running any commands, you must respect these cluster constraints:

1. **NEVER Write Caches or Data to `$HOME`:**
   * On TAMU Grace, `$HOME` (`/home/jacob.dannie.am`) has a strict **10 GB storage limit and 10,000 file quota**.
   * Isaac Sim generates thousands of shader cache and log files. If Isaac Sim writes to `~/.nvidia-omniverse` in `$HOME`, you will immediately hit a `Disk quota exceeded` error and crash the VNC session.
   * Always work in `$SCRATCH` (`/scratch/user/jacob.dannie.am`), which has a 1 TB / 250,000 file limit.

2. **Make Sure Your SSH Shell is on the Same Node as the VNC Desktop (`r001`):**
   * When you first SSH into Grace from Antigravity, you may land on a login node (`grace1`–`grace5`).
   * If `hostname` is not `r001` (or the active VNC GPU node), find your active SLURM VNC job with `squeue -u $USER` and SSH directly into that node (`ssh r001`).

3. **Bridging Your SSH Terminal to the Open OnDemand VNC Desktop (`$DISPLAY`):**
   * The user has an **HPRC Open OnDemand VNC Desktop** running on `r001`.
   * However, a separate SSH terminal session from Antigravity does **not** automatically inherit the VNC `$DISPLAY` environment variable.
   * Before launching the Isaac Sim GUI from your SSH terminal so the user can see it inside their VNC browser tab, you must export the active VNC display number (e.g., `export DISPLAY=:1` or whatever `:N` belongs to the user's `Xvnc` process).

4. **Compute Nodes Have No Direct Internet by Default:**
   * Compute nodes like `r001` are isolated from the public internet. Always load the proxy module before running Isaac Sim:
     ```bash
     module load WebProxy
     ```

5. **Scaniverse `.ply` Point Clouds vs. Triangle Meshes:**
   * Our course file is a **Scaniverse `.ply` point cloud** (`596,349` colored points), **not** a standard `.obj` mesh.
   * Standard `omni.kit.asset_converter` expects polygonal faces and will often fail or produce an empty stage if given a pure point cloud `.ply` with `0` faces.
   * Instead, use a Python script via `/isaac-sim/python.sh` to parse the binary/ASCII `.ply` directly with `numpy` and build an OpenUSD `UsdGeom.Points` prim (for crisp colored visualization) plus a voxelized collision mesh proxy (for drone physics).

---

## 3. Dissecting the Baseline Commands

Here is what the user's baseline commands do and how you should use them:

```bash
# 1. Moves into high-capacity scratch storage
cd $SCRATCH

# 2. Sets up Singularity bind paths (SINGULARITY_BIND / APPTAINER_BIND) 
#    and redirects Omniverse caches/logs to $SCRATCH
source env_isaac-sim.sh

# 3. Uses HPRC's local wrapper script to clear/reset corrupted user configs
my-isaac-sim/isaac-sim/isaac-sim.sh --reset-user

# 4. Executes Isaac Sim inside the Singularity image with NVIDIA GPU passthrough (--nv)
singularity exec --nv /sw/hprc/sw/bio/containers/isaac-sim/isaac-sim_5.1.0.sif /isaac-sim/isaac-sim.sh
```

---

## 4. Phase 1: Safe Environment Probing (Run These First!)

Run these lightweight diagnostic commands to verify the node, find the VNC display, inspect the scripts, and locate the Scaniverse `.ply` file:

### Step 1.1: Verify Node, SLURM Job, GPU, and Quotas

```bash
hostname
squeue -u $USER
nvidia-smi
showquota
```

* **What to look for:**
  * If `hostname` says `grace1`–`grace5`, look at the `NODELIST` column in `squeue -u $USER` (e.g., `r001`) and run `ssh r001` to jump onto the VNC GPU node.
  * Confirm an NVIDIA GPU is visible in `nvidia-smi` and `$HOME` quota in `showquota` is not maxed out.

### Step 1.2: Detect the Active Open OnDemand VNC `$DISPLAY`

```bash
echo "Current DISPLAY: $DISPLAY"
ps -u $USER -o pid,cmd | grep -E "Xvnc|Xorg"
ls -la /tmp/.X11-unix/
```

* **What to look for:** Look at the `Xvnc` command line (e.g., `Xvnc :1 ...` or `Xvnc :2 ...`). If `echo $DISPLAY` is blank in your SSH terminal, export that display number:
  ```bash
  export DISPLAY=:1   # Replace :1 with the exact :N shown in your Xvnc process!
  ```

### Step 1.3: Inspect the HPRC Setup Scripts

```bash
cd $SCRATCH
ls -la
cat env_isaac-sim.sh
cat my-isaac-sim/isaac-sim/isaac-sim.sh
```

* **What to look for:** Note the exact `--bind` paths exported by `env_isaac-sim.sh` and used inside `my-isaac-sim/isaac-sim/isaac-sim.sh`. You will need these same bind mounts whenever you call `/isaac-sim/python.sh`.

### Step 1.4: Locate and Inspect the Scaniverse `.ply` File

```bash
find $SCRATCH -maxdepth 4 -iname "*.ply"
find $HOME -maxdepth 4 -iname "*.ply" 2>/dev/null
```

Once you find the `.ply` file, inspect its header (first 35 lines) without printing binary data:

```bash
head -n 35 /path/to/course.ply
```

* **What to look for:**
  * Confirm `element vertex 596349` (or similar vertex count).
  * Check the format line (`format binary_little_endian 1.0` or `format ascii 1.0`).
  * Check the exact property names (`property float x`, `y`, `z`, `property uchar red`, `green`, `blue`, etc.) and whether `element face` is `0` or non-existent.
  * If the `.ply` file is stored in `$HOME`, **copy it into `$SCRATCH`** immediately so Singularity can access it cleanly.

---

## 5. Phase 2: Two Ways to Run Isaac Sim (Headless Automation + VNC GUI)

Because we have **both** an Antigravity SSH terminal and an **Open OnDemand VNC Desktop**:

1. **Use Headless Python (`/isaac-sim/python.sh`)** to programmatically convert the `.ply` scan, construct the `.usd` scene, configure lighting/physics, and run automated tests.
2. **Use GUI Mode (`/isaac-sim/isaac-sim.sh`) with `DISPLAY` set** to pop the visual Isaac Sim window onto the user's Open OnDemand VNC Desktop once the `.usd` stage is ready!

### Step 2.1: Run a Minimal Headless Heartbeat Test

Create `$SCRATCH/test_headless.py`:

```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

print("=== SUCCESS: Isaac Sim 5.1.0 Headless Initialized! ===")
stage = omni.usd.get_context().new_stage()
UsdGeom.Xform.Define(stage, "/World")
print("Active Stage Root:", stage.GetRootLayer().identifier)

simulation_app.close()
print("=== SUCCESS: Clean Shutdown ===")
```

Run it using the environment setup:

```bash
cd $SCRATCH
source env_isaac-sim.sh
module load WebProxy
singularity exec --nv --bind $SCRATCH:$SCRATCH /sw/hprc/sw/bio/containers/isaac-sim/isaac-sim_5.1.0.sif /isaac-sim/python.sh $SCRATCH/test_headless.py
```

*(Note: On first launch, RTX shader compilation takes **3 to 8 minutes**. Do NOT kill the process early—let it finish caching shaders to `$SCRATCH`!)*

---

## 6. Phase 3: Converting the Scaniverse `.ply` Point Cloud (`596,349` Points) to OpenUSD

### Why a Custom Converter Script is Required for Scaniverse `.ply`

1. **Pure Point Cloud Geometry:** Scaniverse point cloud exports contain vertices ($x, y, z$) and vertex colors ($R, G, B$), with zero triangle faces. OpenUSD represents point clouds natively using `UsdGeom.Points` with a `primvars:displayColor` array.
2. **Coordinate System ($Y$-Up $\rightarrow$ $Z$-Up):** Scaniverse exports scans in a **$Y$-up** coordinate frame, whereas Isaac Sim robotics environments use **$Z$-up**. Our converter rotates the points by $(x, y, z)_{\text{Scaniverse}} \rightarrow (x, -z, y)_{\text{IsaacSim}}$ and shifts the floor to $z = 0$ so the course sits flat and right-side up.
3. **Drone Collision Proxy:** PhysX cannot collide directly with raw 0-face `UsdGeom.Points`. To prevent our drone from falling through the floor or flying through course obstacles, the script also generates an optional **voxelized static collision mesh** (`UsdGeom.Mesh` with `UsdPhysics.CollisionAPI`) aligned with the point cloud!

### Step 3.1: Create `$SCRATCH/convert_scaniverse_ply.py`

This script uses only `numpy` and OpenUSD (`pxr`), which are built into `/isaac-sim/python.sh` (no internet `pip install` required):

```python
import os
import sys
import numpy as np
from isaacsim import SimulationApp

# Initialize headless Isaac Sim
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Sdf, Vt, Gf


def load_scaniverse_ply(ply_path: str):
    """
    Parses binary_little_endian or ASCII Scaniverse .ply point clouds using pure numpy.
    Returns (points_xyz_float32, colors_rgb_float32_0_to_1).
    """
    PlyTypeMap = {
        "char": "i1", "int8": "i1",
        "uchar": "u1", "uint8": "u1",
        "short": "i2", "int16": "i2",
        "ushort": "u2", "uint16": "u2",
        "int": "i4", "int32": "i4",
        "uint": "u4", "uint32": "u4",
        "float": "f4", "float32": "f4",
        "double": "f8", "float64": "f8",
    }

    with open(ply_path, "rb") as f:
        fmt = None
        num_vertices = 0
        vertex_props = []
        in_vertex_section = False

        while True:
            line = f.readline()
            if not line:
                raise ValueError("Reached EOF before end_header in PLY file.")
            line_str = line.decode("ascii", errors="ignore").strip()

            if line_str.startswith("format "):
                fmt = line_str.split()[1]
            elif line_str.startswith("element vertex "):
                num_vertices = int(line_str.split()[2])
                in_vertex_section = True
            elif line_str.startswith("element ") and not line_str.startswith("element vertex"):
                in_vertex_section = False
            elif line_str.startswith("property ") and in_vertex_section:
                parts = line_str.split()
                if parts[1] != "list":
                    dtype_str = PlyTypeMap.get(parts[1], "f4")
                    prop_name = parts[2]
                    vertex_props.append((prop_name, "<" + dtype_str))
            elif line_str == "end_header":
                break

        print(f"[INFO] PLY Format: {fmt} | Vertices: {num_vertices:,}")
        print(f"[INFO] Vertex Properties: {[p[0] for p in vertex_props]}")

        dtype = np.dtype(vertex_props)
        if fmt == "binary_little_endian":
            data = np.fromfile(f, dtype=dtype, count=num_vertices)
        elif fmt == "ascii":
            data = np.loadtxt(f, dtype=dtype, max_rows=num_vertices)
        else:
            raise ValueError(f"Unsupported PLY format: {fmt}")

    # Extract XYZ coordinates
    x = data["x"].astype(np.float32)
    y = data["y"].astype(np.float32)
    z = data["z"].astype(np.float32)

    # Rotate from Scaniverse Y-up (x, y, z) to Isaac Sim Z-up (x, -z, y)
    pts_z_up = np.stack([x, -z, y], axis=-1)

    # Shift minimum Z ( ~2nd percentile to ignore stray outlier noise ) to z = 0.0
    ground_z = np.percentile(pts_z_up[:, 2], 1.0)
    pts_z_up[:, 2] -= ground_z
    print(f"[INFO] Shifted Z by {-ground_z:.3f}m so course floor sits at Z=0.")

    # Extract RGB colors (normalized to [0.0, 1.0])
    prop_names = data.dtype.names
    if "red" in prop_names and "green" in prop_names and "blue" in prop_names:
        r = data["red"].astype(np.float32)
        g = data["green"].astype(np.float32)
        b = data["blue"].astype(np.float32)
        if r.max() > 1.0 or g.max() > 1.0 or b.max() > 1.0:
            r /= 255.0
            g /= 255.0
            b /= 255.0
        colors = np.stack([r, g, b], axis=-1)
    else:
        print("[WARN] No RGB properties found; defaulting to light gray.")
        colors = np.ones_like(pts_z_up) * 0.8

    return pts_z_up, colors


def build_voxel_collision_mesh(stage, prim_path: str, pts: np.ndarray, voxel_size: float = 0.15):
    """
    Builds a lightweight static triangle mesh out of occupied voxels so drones/robots
    can physically collide with the point cloud floor and obstacles.
    """
    # Quantize 3D points into unique occupied voxel grid coordinates
    voxel_coords = np.floor(pts / voxel_size).astype(np.int32)
    unique_voxels = np.unique(voxel_coords, axis=0)
    print(f"[INFO] Generated {len(unique_voxels):,} occupied collision voxels (size={voxel_size}m).")

    # Unit cube vertices (8) and triangle indices (12 triangles = 36 indices)
    cube_verts = np.array([
        [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]
    ], dtype=np.float32) * voxel_size

    cube_faces = np.array([
        0, 2, 1, 0, 3, 2,  # Bottom
        4, 5, 6, 4, 6, 7,  # Top
        0, 1, 5, 0, 5, 4,  # Front
        2, 3, 7, 2, 7, 6,  # Back
        0, 4, 7, 0, 7, 3,  # Left
        1, 2, 6, 1, 6, 5   # Right
    ], dtype=np.int32)

    n_vox = len(unique_voxels)
    offsets = (unique_voxels.astype(np.float32) * voxel_size)[:, None, :]
    all_verts = (offsets + cube_verts[None, :, :]).reshape(-1, 3)

    vert_offsets = (np.arange(n_vox, dtype=np.int32) * 8)[:, None]
    all_indices = (vert_offsets + cube_faces[None, :]).reshape(-1)
    face_counts = np.full(n_vox * 12, 3, dtype=np.int32)

    mesh_prim = UsdGeom.Mesh.Define(stage, prim_path)
    mesh_prim.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(all_verts))
    mesh_prim.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(face_counts))
    mesh_prim.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(all_indices))

    # Hide the blocky collision proxy visually so only the crisp Scaniverse point cloud is seen
    mesh_prim.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdGeom.Imageable(mesh_prim).MakeInvisible()

    # Apply exact static triangle mesh collision
    UsdPhysics.CollisionAPI.Apply(mesh_prim.GetPrim())
    mesh_col = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim.GetPrim())
    mesh_col.CreateApproximationAttr("none")


def create_course_usd(ply_path: str, output_usd_path: str, point_width: float = 0.015):
    pts, colors = load_scaniverse_ply(ply_path)

    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    stage = usd_context.get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world_prim)

    # Physics Scene
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    # Dome Lighting
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(1000.0)

    # Course Root Xform
    course_xform = UsdGeom.Xform.Define(stage, "/World/DroneCourse")

    # 1. Visual Colored Point Cloud (596,349 points)
    pc_prim = UsdGeom.Points.Define(stage, "/World/DroneCourse/ScaniversePointCloud")
    pc_prim.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    pc_prim.CreateWidthsAttr(Vt.FloatArray([point_width] * len(pts)))

    color_primvar = UsdGeom.PrimvarsAPI(pc_prim).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
    )
    color_primvar.Set(Vt.Vec3fArray.FromNumpy(colors))

    # Set bounding box extent so camera framing (F key) works immediately in VNC GUI
    min_pt = pts.min(axis=0).tolist()
    max_pt = pts.max(axis=0).tolist()
    pc_prim.CreateExtentAttr([Gf.Vec3f(*min_pt), Gf.Vec3f(*max_pt)])
    print(f"[INFO] Course Bounds (meters): Min={np.round(min_pt, 2)}, Max={np.round(max_pt, 2)}")

    # 2. Invisible Static Collision Proxy for Drone Physics
    build_voxel_collision_mesh(stage, "/World/DroneCourse/CollisionProxy", pts, voxel_size=0.15)

    stage.GetRootLayer().Export(output_usd_path)
    print(f"[SUCCESS] Saved simulation-ready Scaniverse course stage to: {output_usd_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python.sh convert_scaniverse_ply.py <path_to_scaniverse.ply> [output.usd]")
        simulation_app.close()
        sys.exit(1)

    input_ply = os.path.abspath(sys.argv[1])
    out_usd = (
        os.path.abspath(sys.argv[2])
        if len(sys.argv) >= 3
        else os.path.join(os.path.dirname(input_ply), "drone_course_sim_ready.usd")
    )

    create_course_usd(input_ply, out_usd)
    simulation_app.close()
```

### Step 3.2: Run the Converter Script

```bash
cd $SCRATCH
source env_isaac-sim.sh
module load WebProxy
singularity exec --nv --bind $SCRATCH:$SCRATCH /sw/hprc/sw/bio/containers/isaac-sim/isaac-sim_5.1.0.sif /isaac-sim/python.sh $SCRATCH/convert_scaniverse_ply.py /path/to/your_course.ply $SCRATCH/drone_course_sim_ready.usd
```

---

## 7. Phase 4: Launching the Course in the Open OnDemand VNC Desktop GUI

Once `$SCRATCH/drone_course_sim_ready.usd` is created, you can launch the full Isaac Sim GUI directly into the user's **Open OnDemand VNC Desktop** so they can fly around and inspect the colored point cloud:

```bash
cd $SCRATCH
source env_isaac-sim.sh
module load WebProxy

# 1. Attach to the active VNC Display discovered in Step 1.2 (e.g., :1)
export DISPLAY=:1

# 2. Launch Isaac Sim GUI onto the VNC Desktop and auto-open the USD stage
my-isaac-sim/isaac-sim/isaac-sim.sh --open $SCRATCH/drone_course_sim_ready.usd
```

*(Tip for the User in VNC: Once the stage loads in the viewport, select `/World/DroneCourse/ScaniversePointCloud` in the Stage tree on the right and press **`F`** on the keyboard to frame the camera onto the course! If the points look too small or too chunky, adjust the `point_width` parameter in `convert_scaniverse_ply.py`—default is `0.015` meters / 1.5 cm).*

---

## 8. Anticipated Errors & Troubleshooting Guide

### Error 1: `Failed to create GLFW window` or `X11: The DISPLAY environment variable is missing`
* **Cause:** Your Antigravity SSH terminal does not have `DISPLAY` set to the Open OnDemand VNC server's display number.
* **Fix:**
  1. Run `ps -u $USER -o pid,cmd | grep Xvnc` on `r001` to find the display number (e.g., `:1` or `:2`).
  2. Run `export DISPLAY=:1` (matching that number) before launching `isaac-sim.sh`.
  3. If you see `Authorization required, but no authorization protocol specified`, also export `export XAUTHORITY=$HOME/.Xauthority`.

### Error 2: Course Appears Sideways or Upside-Down in the Viewport
* **Cause:** Different Scaniverse export settings sometimes swap $Y$ and $Z$ axes differently.
* **Fix:** You don't even need to re-run the converter! In the Isaac Sim GUI (or in Python), select `/World/DroneCourse` and set its `xformOp:rotateXYZ` attribute to `(0, 0, 0)` or `(90, 0, 0)` until the course floor aligns with the ground grid.

### Error 3: `OSError: [Errno 30] Read-only file system`
* **Cause:** Singularity containers are read-only by default, and `/isaac-sim/python.sh` was called without sourcing `env_isaac-sim.sh` or without passing the host bind mounts defined in `my-isaac-sim/isaac-sim/isaac-sim.sh`.
* **Fix:** Inspect `cat $SCRATCH/my-isaac-sim/isaac-sim/isaac-sim.sh`, copy its exact `--bind` flags, and pass them to `singularity exec --nv ...`.

### Error 4: `Disk quota exceeded`
* **Cause:** Something wrote to `/home/jacob.dannie.am` (`$HOME`), such as `~/.nvidia-omniverse` or `~/.cache`.
* **Fix:** Run `showquota`. Move any large hidden directories (`~/.cache`, `~/.nvidia-omniverse`) into `$SCRATCH` and symlink them (`ln -s $SCRATCH/.cache ~/.cache`).

---

## 9. Immediate Action Plan for the Agent

When you begin your terminal session, execute this sequence:

1. **Check Node & VNC Display:** Run `hostname`, `squeue -u $USER`, `nvidia-smi`, `showquota`, and `ps -u $USER -o pid,cmd | grep Xvnc` to confirm you are on `r001` and identify the active VNC `DISPLAY`.
2. **Inspect Scripts & Find the `.ply` Scan:** Run `cat $SCRATCH/env_isaac-sim.sh`, `cat $SCRATCH/my-isaac-sim/isaac-sim/isaac-sim.sh`, and `find $SCRATCH $HOME -maxdepth 4 -iname "*.ply" 2>/dev/null`.
3. **Inspect the `.ply` Header:** Run `head -n 35 <path_to_ply>` to verify the vertex count (`596,349`), format (`binary_little_endian` vs `ascii`), and color property names.
4. **Convert the Course Scan:** Create `$SCRATCH/convert_scaniverse_ply.py` and run it via `/isaac-sim/python.sh` to generate `$SCRATCH/drone_course_sim_ready.usd`.
5. **Open in VNC Desktop:** Export the VNC `DISPLAY` and launch `my-isaac-sim/isaac-sim/isaac-sim.sh --open $SCRATCH/drone_course_sim_ready.usd` so the user can see the 3D colored drone course live in their Open OnDemand VNC Desktop.