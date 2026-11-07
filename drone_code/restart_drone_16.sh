#!/bin/bash
# Restarts drone 16's software (the "stack"), waits for our mission to come up, then runs the check.
# Run this on the Mac, with the drone ALREADY ON THE TAKEOFF SPOT and still:
#
#   drone_code/restart_drone_16.sh skybeta-16.local
#
# Why the takeoff spot: camera tracking's position starts at (0, 0, 0) wherever the drone is when
# the stack starts, and every target is held at least 0.3 m above that point. Do not move the drone
# after this. A restart also builds ~/team_ws again, so it picks up a fresh install.
#
# It restarts the stack the way the organizers' guide says: remove its container, and the service
# brings it back by itself. It never touches PX4 or its parameters.
set -e
folder_of_this_script="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=drone_16_common.sh
source "$folder_of_this_script/drone_16_common.sh"

# Our own choice: the stack is back in about 10 s, then it builds team_ws before starting team_nav.
LONGEST_WAIT_SECONDS=240
# Our own choice: after team_nav starts, give camera tracking time to start reporting.
SETTLE_SECONDS=15

confirm_it_is_drone_16
echo "Restarting the stack. Keep the drone still on the takeoff spot."
on_drone "docker rm -f skybeta-stack > /dev/null"
waited=0
until on_drone "docker exec skybeta-stack ps -eo args 2>/dev/null | grep -q 'team_nav/mission'"; do
    if [ "$waited" -ge "$LONGEST_WAIT_SECONDS" ]; then
        echo "team_nav has not started after $LONGEST_WAIT_SECONDS s. The build may have failed; the check below says more."
        break
    fi
    sleep 5
    waited=$((waited + 5))
    echo "  waiting for team_nav to start ($waited s)"
done
echo "Letting camera tracking settle for $SETTLE_SECONDS s."
sleep "$SETTLE_SECONDS"
exec "$folder_of_this_script/check_drone_16.sh" "$drone_host"
