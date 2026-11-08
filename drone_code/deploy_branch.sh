#!/bin/bash
# drone_code/deploy_branch.sh <branch-name>
# Deploys a specified git branch to drone 16 with audit logging.
set -euo pipefail

BRANCH="${1:-}"
DRONE_ADDRESS="${DRONE_ADDRESS:-192.168.55.1}"
DRONE_LOGIN="dronehacks@$DRONE_ADDRESS"
DEPLOY_LOG="telemetry/deployments.log"
mkdir -p "$(dirname "$DEPLOY_LOG")"

if [ -z "$BRANCH" ]; then
    echo "Usage: $0 <branch-name>"
    echo "Example: $0 classical-dannie-attemptone"
    exit 1
fi

COMMIT_SHA="$(git rev-parse --short HEAD)"
TIMESTAMP="$(date '+%Y-%m-%d %H:%M:%S')"
echo "[$TIMESTAMP] DEPLOY START: Branch $BRANCH ($COMMIT_SHA) -> $DRONE_LOGIN" | tee -a "$DEPLOY_LOG"

echo "Switching laptop working tree to: $BRANCH..."
git checkout "$BRANCH"

echo "Deploying $BRANCH to $DRONE_LOGIN..."
drone_code/install_on_drone_16.sh

echo "[$TIMESTAMP] DEPLOY SUCCESS: $BRANCH ($COMMIT_SHA) compiled and ready on drone 16." | tee -a "$DEPLOY_LOG"
