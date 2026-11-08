$Trial = 'c1'
$ErrorActionPreference = 'Stop'
$root = '/mnt/c/Users/Adity/OneDrive/Documents/ChatGPT/DroneHacks/taskforce-c1-020-001'
$image = 'sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1'
$manifest = Get-Content (Join-Path $PSScriptRoot '../taskforce-flight-prep-008/candidate.manifest.json') -Raw | ConvertFrom-Json
$profile = if ($Trial -eq 'c1') { 'c1_stop_020' } else { 'r1_stationary_expiry' }
$domain = if ($Trial -eq 'c1') { '57' } else { '58' }
$output = Join-Path $PSScriptRoot 'evidence'
if (Test-Path -LiteralPath $output) { throw 'Fresh evidence directory required' }
New-Item -ItemType Directory -Path $output | Out-Null
$dockerArgs = @('run','--rm','--name','drone-c1-020-001','--network','none','--gpus','all',
 '--cap-drop','ALL','--security-opt','no-new-privileges','--shm-size','1g')
foreach ($mount in @(
 'type=bind,source=/usr/lib/wsl/lib,target=/usr/lib/wsl/lib,readonly',
 'type=bind,source=/usr/lib/wsl/drivers,target=/usr/lib/wsl/drivers,readonly',
 'type=bind,source=/mnt/wslg,target=/mnt/wslg,readonly',
 'type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly',
 "type=bind,source=$root/../taskforce-flight-prep-008/candidate.tar,target=/source.tar,readonly",
 "type=bind,source=$root/run_trial.sh,target=/runner.sh,readonly",
 "type=bind,source=$root/evidence,target=/evidence",
 "type=bind,source=$root/approval.json,target=/approval.json,readonly")) { $dockerArgs += @('--mount',$mount) }
foreach ($setting in @('LD_LIBRARY_PATH=/usr/lib/wsl/lib','DISPLAY=:0','XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir',
 'GALLIUM_DRIVER=d3d12','MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA','GAZEBO_RENDER_BACKEND=wslg_d3d12',
 "ROS_DOMAIN_ID=$domain",'ROS_LOCALHOST_ONLY=1',"CALIBRATION_IMAGE_SHA256=$image")) { $dockerArgs += @('--env',$setting) }
$dockerArgs += @('--entrypoint','bash',$image,'/runner.sh',$manifest.revision,$profile)
& wsl -d Ubuntu -- docker @dockerArgs
exit $LASTEXITCODE
