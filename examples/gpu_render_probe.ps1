# Stationary camera only: no PX4, ROS bridge, arming or flight owner.
param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference = 'Stop'
if (Test-Path -LiteralPath $OutputDirectory) { throw 'Use a new evidence directory.' }
$outputPath = (New-Item -ItemType Directory -Path $OutputDirectory).FullName
$probePath = Join-Path $PSScriptRoot 'gpu_render_probe.sdf'
Copy-Item -LiteralPath $probePath -Destination (Join-Path $outputPath 'probe.sdf')
$linuxPathResult = wsl -d Ubuntu -- wslpath -a ($outputPath.Replace('\','/'))
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve evidence path in Ubuntu WSL.' }
$linuxPath = $linuxPathResult.Trim()
$probeCommand = @'
glxinfo -B > /evidence/glxinfo.txt 2>&1
timeout -k 3s 20s gz sim -s -r -v 4 /evidence/probe.sdf > /evidence/gazebo.log 2>&1 &
probe_pid=$!
sleep 8
timeout -k 2s 5s gz topic -e -t /gpu_probe/depth -n 1 > /evidence/depth.txt 2>&1
wait "$probe_pid"
cp /root/.gz/rendering/ogre2.log /evidence/ogre2.log
grep -q 'GL_RENDERER = D3D12 (NVIDIA' /evidence/ogre2.log && grep -q 'width: 320' /evidence/depth.txt
'@
[System.IO.File]::WriteAllText((Join-Path $outputPath 'probe.sh'), $probeCommand.Replace("`r`n","`n"), [System.Text.UTF8Encoding]::new($false))
$dockerArgs = @('run','--rm','--network','none','--gpus','all','--cap-drop','ALL','--security-opt','no-new-privileges',
 '--mount','type=bind,source=/usr/lib/wsl/lib,target=/usr/lib/wsl/lib,readonly',
 '--mount','type=bind,source=/usr/lib/wsl/drivers,target=/usr/lib/wsl/drivers,readonly',
 '--mount','type=bind,source=/mnt/wslg,target=/mnt/wslg,readonly',
 '--mount','type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly',
 '--mount',"type=bind,source=$linuxPath,target=/evidence",
 '--env','LD_LIBRARY_PATH=/usr/lib/wsl/lib','--env','DISPLAY=:0',
 '--env','XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir','--env','GALLIUM_DRIVER=d3d12',
 '--env','MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA','--entrypoint','bash',
 'sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1','/evidence/probe.sh')
wsl -d Ubuntu -- docker @dockerArgs
if ($LASTEXITCODE -ne 0) { throw "GPU rendering probe failed; inspect $outputPath" }
Write-Output "RTX rendering and depth delivery verified: $outputPath"
