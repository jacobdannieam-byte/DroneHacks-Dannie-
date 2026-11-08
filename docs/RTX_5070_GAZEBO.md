# RTX 5070 Gazebo rendering

Verified locally on 2026-11-08: NVIDIA GeForce RTX 5070 Laptop GPU, Windows driver 592.15, Docker Desktop through Ubuntu WSL2, pinned simulator image `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`.

`--gpus all` exposed CUDA but did not alone enable accelerated camera rendering. The working graphics path uses Mesa D3D12, WSLg, and read-only mounts of both `/usr/lib/wsl/lib` and `/usr/lib/wsl/drivers`. GLX reports `Accelerated: yes`; Gazebo's actual OGRE2 log reports `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Laptop GPU)`. A stationary depth camera delivered a 320x240 image. No PX4, vehicle controller, ROS flight bridge or moving model was launched.

## Reproduce

From this checkout in PowerShell:

```powershell
./examples/gpu_render_probe.ps1 -OutputDirectory ../taskforce-bootstrap-005/gpu-new-run
```

Use a new output directory; the script preserves previous evidence. It runs a bounded server-only Gazebo camera world with networking disabled and no flight software, captures renderer/depth evidence, and removes the container. It requires the existing Ubuntu Docker integration and WSLg session. The script contains the tested image, environment and mounts. No Linux NVIDIA driver installation is needed.

## Apply to subsequent simulator runs

Use the same graphics mounts/environment on the isolated simulator container. Invoke the server with `gz sim -s -r` using WSLg GLX; do not retain `--headless-rendering` from the CPU/EGL configuration. Verify the actual OGRE renderer in each run and refuse to call a software fallback GPU-qualified. Preserve the existing source archive, ROS isolation, clock, recorder, approval and command-owner gates. Full PX4 plus all-camera composition and H3 timing remain to be validated on this configuration before flight; this probe does not qualify them.

GPU rendering can remove CPU camera work. It does not automatically accelerate Python/Numpy voxel processing, physics, test orchestration, or remote Codex inference. Previous native runs were already close to real-time factor 1.0, so this does not imply proportional wall-time speedup. No comparative throughput improvement has yet been measured. Keep simulated timing unchanged until a separate benchmark establishes a reason to change it.

## Preserved failures

Initial EGL probes failed or fell back to software; mounting only WSL libraries omitted the graphics driver store. The first stationary Gazebo probe delivered depth but did not exit on TERM; it was explicitly stopped after preserving its log. The repeatable probe now bounds shutdown with KILL after a grace interval. The first PowerShell wrapper lost backslashes in WSL path conversion; a later inline-shell attempt lost its PID variable. The final wrapper normalizes paths and writes a literal shell file rather than passing shell source across Windows/WSL quoting layers. Raw attempts remain under `../taskforce-bootstrap-005/`.

References: [Docker GPU support](https://docs.docker.com/desktop/features/gpu/), [Gazebo EGL headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html), [NVIDIA WSL guide](https://docs.nvidia.com/cuda/wsl-user-guide/). Local renderer evidence, rather than CUDA visibility, establishes this result.
