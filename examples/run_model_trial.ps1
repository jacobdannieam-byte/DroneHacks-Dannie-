param(
    [Parameter(Mandatory=$true)][string]$CandidateFile,
    [Parameter(Mandatory=$true)][string]$SourceArchive,
    [Parameter(Mandatory=$true)][string]$OutputDirectory,
    [string]$ApprovalFile,
    [switch]$DisarmedCheck
)
$ErrorActionPreference = 'Stop'
$candidate = Get-Content -LiteralPath $CandidateFile -Raw | ConvertFrom-Json
$archive = (Resolve-Path -LiteralPath $SourceArchive).Path
if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $candidate.source_sha256) {
    throw 'Frozen source archive does not match candidate'
}
if (Test-Path -LiteralPath $OutputDirectory) { throw 'Evidence output must be new' }
$mode = 'model_check'
$mounts = @()
$approvalArgument = ''
if (-not $DisarmedCheck) {
    if (-not $ApprovalFile) { throw 'Explicit reviewed model-flight approval required' }
    $approvalPath = (Resolve-Path -LiteralPath $ApprovalFile).Path
    $approval = Get-Content -LiteralPath $approvalPath -Raw | ConvertFrom-Json
    if ($approval.mode -ne 'hybrid_model_trial' -or $approval.revision -ne $candidate.revision -or
        $approval.image_sha256 -ne $candidate.image -or $approval.ros_domain_id -ne 62 -or
        $approval.registration_enabled -ne $false -or $approval.fallback_allowed -ne $false -or
        $approval.experimental_model_control_reviewed -ne $true -or
        [DateTimeOffset]::Parse($approval.expires_utc) -le [DateTimeOffset]::UtcNow) {
        throw 'A current exact-candidate model-control approval is required'
    }
    $mode = 'model_trial'
    $mounts = @('--mount', "type=bind,src=$approvalPath,dst=/approval.json,readonly")
    $approvalArgument = '/approval.json'
}
$out = New-Item -ItemType Directory -Path $OutputDirectory
$containerName = 'drone-model-' + [Guid]::NewGuid().ToString('N').Substring(0,8)
@{revision=$candidate.revision; source_sha256=$candidate.source_sha256; image=$candidate.image;
  mode=$mode; registration_enabled=$false; container=$containerName} |
    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out.FullName 'host-launch.json')
docker run --rm --init --network none --cap-drop ALL --security-opt no-new-privileges `
    --name $containerName --env ROS_DOMAIN_ID=62 --env ROS_LOCALHOST_ONLY=1 --env GZ_IP=127.0.0.1 `
    --env "CALIBRATION_IMAGE_SHA256=$($candidate.image)" `
    --mount "type=bind,src=$archive,dst=/source.tar,readonly" `
    --mount "type=bind,src=$($out.FullName),dst=/evidence" @mounts `
    $candidate.image bash -c 'cd /repository && tar --no-same-owner -xmf /source.tar && exec bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh "$@"' _ `
    $mode drone_code/team_nav/course_maps/phone_scan_2026-11-06.json 10 $candidate.revision `
    scenarios/model_flight/h1_pinn_prefix.json $approvalArgument 2>&1 |
    Tee-Object -FilePath (Join-Path $out.FullName 'console.txt')
$runResult = $LASTEXITCODE
if (-not $DisarmedCheck -and (Test-Path -LiteralPath (Join-Path $out.FullName 'observations'))) {
    # Raw command correlation runs read-only after bag finalization; its report is new evidence.
    docker run --rm --network none --cap-drop ALL --security-opt no-new-privileges `
        --env OPENBLAS_NUM_THREADS=1 `
        --mount "type=bind,src=$archive,dst=/source.tar,readonly" `
        --mount "type=bind,src=$($out.FullName),dst=/evidence" `
        $candidate.image bash -c 'cd /repository && tar --no-same-owner -xmf /source.tar && source /opt/ros/humble/setup.bash && source /opt/px4_messages/install/setup.bash && python3 -m examples.audit_model_flight --evidence /evidence' 2>&1 |
        Tee-Object -FilePath (Join-Path $out.FullName 'audit-console.txt')
    if ($LASTEXITCODE -ne 0) { $runResult = 1 }
}
exit $runResult
