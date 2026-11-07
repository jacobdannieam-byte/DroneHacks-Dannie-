#!/usr/bin/env bash
# Host entrypoint. The container has loopback only, no hardware devices, no host ROS network.
set -euo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
mode=observe
domain=37
seconds=30
world=drone_code/team_nav/course_maps/phone_scan_2026-11-06.json
profile=
approval=
check=0
usage() {
    echo "Usage: bash simulator/gazebo_trial/run_hybrid.sh [--mode observe|classical|hybrid_shadow|hybrid_sitl] [--profile FILE] [--approval FILE] [--domain 1..99] [--world TRACKED_MAP] [--seconds 4..120] [--check]"
}
while (($#)); do
    case "$1" in
        --check) check=1; shift ;;
        --help|-h) usage; exit 0 ;;
        --mode|--domain|--world|--seconds|--profile|--approval)
            [ "$#" -ge 2 ] || { usage >&2; exit 2; }
            case "$1" in
                --mode) mode=$2 ;; --domain) domain=$2 ;; --world) world=$2 ;;
                --seconds) seconds=$2 ;; --profile) profile=$2 ;; --approval) approval=$2 ;;
            esac
            shift 2 ;;
        *) usage >&2; exit 2 ;;
    esac
done
case "$mode" in observe|classical|hybrid_shadow|hybrid_sitl) ;; *) usage >&2; exit 2 ;; esac
[[ "$domain" =~ ^[1-9][0-9]?$ && "$seconds" =~ ^[1-9][0-9]{0,2}$ && "$seconds" -ge 4 && "$seconds" -le 120 ]] ||
    { echo "Invalid domain or observation duration" >&2; exit 2; }
[[ "$world" =~ ^drone_code/team_nav/course_maps/[A-Za-z0-9_.-]+\.json$ ]] &&
    git -C "$repo" ls-files --error-unmatch -- "$world" >/dev/null 2>&1 ||
    { echo "World must be a tracked course map in this repository" >&2; exit 2; }
if [[ "$mode" == hybrid_shadow || "$mode" == hybrid_sitl ]]; then
    [ -f "$profile" ] || { echo "A reviewed Gazebo profile is required" >&2; exit 2; }
fi
if [[ "$mode" == hybrid_sitl || "$mode" == classical ]]; then
    [ -f "$approval" ] || { echo "This flight mode needs the exact team-approved trial record" >&2; exit 2; }
elif [ -n "$approval" ]; then
    echo "An approval record is used only in flight modes" >&2
    exit 2
fi
revision="$(git -C "$repo" rev-parse HEAD)"
if ((check)); then
    printf 'mode=%s domain=%s world=%s revision=%s launch=false\n' "$mode" "$domain" "$world" "$revision"
    exit 0
fi
command -v docker >/dev/null || { echo "Docker is required; no simulator was started" >&2; exit 127; }
docker info >/dev/null || { echo "Docker daemon unavailable; no simulator was started" >&2; exit 1; }
docker image inspect drone-simulator-gazebo-hybrid >/dev/null 2>&1 ||
    { echo "Build the Hybrid Gazebo image first (see docs/hybrid_gazebo_runtime.md)" >&2; exit 2; }
if [ -n "$(git -C "$repo" status --porcelain -- simulator drone_code examples refnav onboard scenarios)" ]; then
    echo "Commit simulator/runtime changes before recording a run" >&2
    exit 2
fi
runs="$repo/simulator/gazebo_trial/generated/hybrid_runs"
mkdir -p "$runs"
out="$(mktemp -d "$runs/run-XXXXXXXX")"
name="drone-gazebo-${out##*/}"
profile_container=
approval_container=
mounts=()
if [ -n "$profile" ]; then
    profile_dir="$(cd "$(dirname "$profile")" && pwd)"
    mounts+=(--mount "type=bind,src=$profile_dir,dst=/profiles,readonly")
    profile_container="/profiles/$(basename "$profile")"
fi
if [ -n "$approval" ]; then
    approval_dir="$(cd "$(dirname "$approval")" && pwd)"
    mounts+=(--mount "type=bind,src=$approval_dir/$(basename "$approval"),dst=/approval.json,readonly")
    approval_container=/approval.json
fi
printf '%s\n' "$revision" > "$out/revision.txt"
printf 'mode=%s\ndomain=%s\nworld=%s\ncontainer=%s\n' "$mode" "$domain" "$world" "$name" > "$out/run.txt"
echo "Container: $name; evidence: $out"
docker run --rm --init --network none --cap-drop ALL --security-opt no-new-privileges \
    --name "$name" --env "ROS_DOMAIN_ID=$domain" --env ROS_LOCALHOST_ONLY=1 --env GZ_IP=127.0.0.1 \
    --mount "type=bind,src=$repo,dst=/source,readonly" --mount "type=bind,src=$out,dst=/evidence" \
    "${mounts[@]}" drone-simulator-gazebo-hybrid \
    bash /source/simulator/gazebo_trial/inside_the_box/run_hybrid.sh \
    "$mode" "$world" "$seconds" "$revision" "$profile_container" "$approval_container" 2>&1 | tee "$out/console.txt"
