#!/bin/bash
# The checks themselves. This runs ON drone 16: check_drone_16.sh sends it over ssh after
# confirming the drone's name. It only reads; it never changes anything on the drone, and it never
# reads or sets a PX4 parameter. Prints one plain line per check, then READY TO FLY or NOT READY
# with the reasons. Exit code 0 means ready.
#
# Arguments: the start limits, metres sideways and metres up or down from where the stack started
# (the mission's origin_check_horizontal_metres and origin_check_vertical_metres).

horizontal_limit_metres="${1:-1.0}"
vertical_limit_metres="${2:-0.3}"
# Where things are on the drone (from the organizers' guide). Tests point these elsewhere.
skybeta_folder="${SKYBETA_FOLDER:-$HOME/skybeta}"
drone_env_file="${DRONE_ENV_FILE:-/etc/skybeta/drone.env}"
team_workspace="${TEAM_WORKSPACE:-$HOME/team_ws}"
stack_container="skybeta-stack"

# From the organizers' guide: takeover needs skybeta 0.3.3 or later.
LOWEST_WORKING_SKYBETA="0.3.3"
# Our own choice: camera tracking normally reports 30 times a second.
LOWEST_ODOMETRY_RATE_HZ=20
# From the team: the in-flight watchdog lands after 0.5 s without a report (rule a).
LONGEST_ODOMETRY_GAP_SECONDS=0.5
# The mission's prestart_blind_below_corners: fewer corners than this, the left camera is blind.
BLIND_BELOW_CORNERS=30
ODOMETRY_TOPIC="/visual_slam/tracking/odometry"

problems=()
warnings=()

line() { printf '%-22s %s\n' "$1:" "$2"; }

in_stack() {
    # A shell inside the running stack, with ROS and both workspaces loaded.
    docker exec "$stack_container" bash -c "
        for setup in /opt/ros/humble/setup.bash /root/skybeta_ws/install/setup.bash /root/team_ws/install/setup.bash; do
            [ -f \$setup ] && source \$setup
        done
        $1" 2>/dev/null
}

check_skybeta_version() {
    local version
    version="$(cat "$skybeta_folder/VERSION" 2>/dev/null | tr -d '[:space:]')"
    if [ -z "$version" ]; then
        line "skybeta version" "unknown: $skybeta_folder/VERSION not found"
        problems+=("skybeta version unknown")
        return
    fi
    if [ "$(printf '%s\n' "$LOWEST_WORKING_SKYBETA" "$version" | sort -V | head -n 1)" = "$LOWEST_WORKING_SKYBETA" ]; then
        line "skybeta version" "$version (at least $LOWEST_WORKING_SKYBETA: yes)"
    else
        line "skybeta version" "$version (at least $LOWEST_WORKING_SKYBETA: NO, PX4 ignores the Offboard request)"
        problems+=("skybeta $version is older than $LOWEST_WORKING_SKYBETA; ask the organizers to update it")
    fi
}

check_competitor_settings() {
    local package launch
    package="$(grep -E '^COMPETITOR_PKG=' "$drone_env_file" 2>/dev/null | tail -n 1 | cut -d= -f2- | tr -d '"')"
    launch="$(grep -E '^COMPETITOR_LAUNCH=' "$drone_env_file" 2>/dev/null | tail -n 1 | cut -d= -f2- | tr -d '"')"
    line "competitor settings" "COMPETITOR_PKG=${package:-unset} COMPETITOR_LAUNCH=${launch:-unset}"
    if [ "$package" != "team_nav" ]; then
        problems+=("COMPETITOR_PKG is '${package:-unset}', not team_nav (drone_code/README.md, install step 2)")
    fi
    case "$launch" in
        team_nav.launch.py|team_nav_route.launch.py) ;;
        *) problems+=("COMPETITOR_LAUNCH is '${launch:-unset}', not team_nav.launch.py or team_nav_route.launch.py") ;;
    esac
}

stack_is_running() {
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$stack_container"
}

check_programs() {
    local programs interfaces missions squares corners
    # "ps" lists every program's command line inside the stack. Counting here, not with pgrep in
    # the stack, so the search words never match the search itself.
    programs="$(docker exec "$stack_container" ps -eo args 2>/dev/null)"
    interfaces="$(grep -c 'skybeta_flight_interface/flight_interface' <<< "$programs")"
    missions="$(grep -c 'team_nav/mission' <<< "$programs")"
    corners="$(grep -c 'team_nav/corner_count' <<< "$programs")"
    squares="$(grep -c 'square_mission' <<< "$programs")"
    line "programs" "team_nav mission $missions, flight_interface $interfaces, square_mission $squares, corner_count $corners"
    [ "$missions" -ge 1 ] || problems+=("the team_nav mission is not running")
    [ "$interfaces" -eq 1 ] || problems+=("$interfaces flight_interface programs are running; exactly 1 must be")
    [ "$squares" -eq 0 ] || problems+=("the organizers' square_mission demo is running")
    [ "$corners" -ge 1 ] || warnings+=("corner_count is not running, so the pre-start check cannot tell a blind camera")
}

check_last_build() {
    # The stack builds ~/team_ws with colcon each time it starts; colcon keeps a log of the last build.
    local log="$team_workspace/log/latest_build/logger_all.log"
    if [ ! -f "$log" ]; then
        line "last team_ws build" "unknown: no build log at $log"
        warnings+=("could not find the last team_ws build log")
        return
    fi
    local when failed
    when="$(date -r "$log" '+%Y-%m-%d %H:%M' 2>/dev/null)"
    failed="$(grep -Eo '[0-9]+ packages? (failed|aborted)[^]]*' "$log" | tail -n 1)"
    if [ -n "$failed" ]; then
        line "last team_ws build" "FAILED at $when: $failed"
        problems+=("the last team_ws build failed ($failed), so nothing of ours started")
    elif grep -q 'Summary:.*finished' "$log"; then
        line "last team_ws build" "succeeded at $when"
    else
        line "last team_ws build" "unfinished or unknown at $when"
        problems+=("the last team_ws build did not finish")
    fi
}

check_odometry_position() {
    local position x y z verdict
    position="$(in_stack "timeout 5 ros2 topic echo --once $ODOMETRY_TOPIC --field pose.pose.position")"
    x="$(awk '/^x:/ {print $2}' <<< "$position")"
    y="$(awk '/^y:/ {print $2}' <<< "$position")"
    z="$(awk '/^z:/ {print $2}' <<< "$position")"
    if [ -z "$x" ] || [ -z "$y" ] || [ -z "$z" ]; then
        line "odometry position" "none received in 5 s"
        problems+=("no camera tracking position on $ODOMETRY_TOPIC")
        return
    fi
    verdict="$(awk -v x="$x" -v y="$y" -v z="$z" -v h="$horizontal_limit_metres" -v v="$vertical_limit_metres" 'BEGIN {
        sideways = sqrt(x * x + y * y); updown = (z < 0) ? -z : z
        printf "%s %.2f", (sideways <= h && updown <= v) ? "inside" : "OUTSIDE", sideways }')"
    line "odometry position" "($(printf '%.2f, %.2f, %.2f' "$x" "$y" "$z")) m, ${verdict#* } m sideways: ${verdict%% *} the start limits (${horizontal_limit_metres} m, ${vertical_limit_metres} m)"
    if [ "${verdict%% *}" != "inside" ]; then
        problems+=("the drone was moved after the stack started; put it on the takeoff spot and restart the stack")
    fi
}

check_odometry_rate() {
    local report rate gap
    report="$(in_stack "timeout 6 ros2 topic hz $ODOMETRY_TOPIC")"
    rate="$(awk '/average rate:/ {rate = $3} END {print rate}' <<< "$report")"
    gap="$(awk '/max:/ {for (i = 1; i <= NF; i++) if ($i == "max:") gap = $(i + 1)} END {sub(/s$/, "", gap); print gap}' <<< "$report")"
    if [ -z "$rate" ] || [ -z "$gap" ]; then
        line "odometry rate" "no reports in 5 s"
        problems+=("camera tracking is not reporting")
        return
    fi
    line "odometry rate" "$rate Hz, longest gap $gap s (over 5 s)"
    if awk -v r="$rate" -v g="$gap" -v lr="$LOWEST_ODOMETRY_RATE_HZ" -v lg="$LONGEST_ODOMETRY_GAP_SECONDS" 'BEGIN {exit !(r < lr || g > lg)}'; then
        problems+=("camera tracking is too slow or gappy ($rate Hz, gap $gap s; needs $LOWEST_ODOMETRY_RATE_HZ Hz and gaps under $LONGEST_ODOMETRY_GAP_SECONDS s)")
    fi
}

check_corner_count() {
    local count
    count="$(in_stack "timeout 4 ros2 topic echo --once /team/left_corner_count --field data" | head -n 1 | tr -d '[:space:]')"
    if ! [[ "$count" =~ ^[0-9]+$ ]]; then
        line "left camera corners" "no count received (corner_count not running?)"
        warnings+=("no corner count from the left camera")
        return
    fi
    if [ "$count" -lt "$BLIND_BELOW_CORNERS" ]; then
        line "left camera corners" "$count (BLIND: fewer than $BLIND_BELOW_CORNERS)"
        problems+=("the left camera sees almost no corners ($count); is it covered, dark, or facing a blank wall?")
    else
        line "left camera corners" "$count (enough to track)"
    fi
}

check_px4_link() {
    local state
    state="$(in_stack "timeout 4 ros2 topic echo --once /skybeta/state --field data")"
    if grep -q '"px4_link": true' <<< "$state"; then
        line "PX4 link" "up"
    elif [ -z "$state" ]; then
        line "PX4 link" "unknown: no /skybeta/state received"
        problems+=("no /skybeta/state from flight_interface")
    else
        line "PX4 link" "DOWN"
        problems+=("flight_interface has no link to PX4")
    fi
}

check_mission_status() {
    local recent status
    # The stack's "competitor" window shows our launch file's output, so our mission's log.
    recent="$(docker exec "$stack_container" tmux capture-pane -p -J -t skybeta:competitor -S -3000 2>/dev/null \
              | grep 'team_nav_mission\]' | sed 's/.*team_nav_mission\]: //')"
    if [ -z "$recent" ]; then
        line "mission status" "no lines from the mission in the competitor window"
        problems+=("no log lines from the team_nav mission")
        return
    fi
    echo "mission status (last 3 lines):"
    tail -n 3 <<< "$recent" | sed 's/^/    /'
    status="$(grep -E '^waiting_for_start: (ready|not ready):' <<< "$recent" | tail -n 1)"
    case "$status" in
        "waiting_for_start: not ready:"*) problems+=("the mission says ${status#waiting_for_start: }") ;;
        "waiting_for_start: ready:"*) ;;
        *) warnings+=("the mission has not said ready yet") ;;
    esac
}

check_skybeta_version
check_competitor_settings
if stack_is_running; then
    check_programs
    check_last_build
    check_px4_link
    check_odometry_position
    check_odometry_rate
    check_corner_count
    check_mission_status
else
    line "stack" "NOT RUNNING (no $stack_container container)"
    problems+=("the stack is not running; it restarts by itself about 10 s after 'docker rm -f skybeta-stack'")
fi

echo
for warning in "${warnings[@]}"; do
    echo "warning: $warning"
done
echo "Reminder: unplug the Pixhawk USB cable; PX4 will not arm while it is plugged in."
if [ "${#problems[@]}" -eq 0 ]; then
    echo "READY TO FLY"
    exit 0
fi
reasons="$(printf '%s; ' "${problems[@]}")"
echo "NOT READY: ${reasons%; }"
exit 1
