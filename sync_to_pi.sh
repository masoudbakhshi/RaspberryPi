#!/usr/bin/env bash
# Sync all GitHub repos to Raspberry Pi

PI_USER="masoudbakhshi_plan22"
PI_HOST="192.168.0.107"
LOCAL_BASE="/c/Python Codes/GitHub"
REMOTE_BASE="~/GitHub"

REPOS=(
    Applications
    ControllerTuning
    MotorControl
    PrivateTest
    PythonCodes
    RaspberryPi
    Website_Plan22
)

export PATH="$HOME/bin:$PATH"

# Verify rsync is available
if ! command -v rsync &>/dev/null; then
    echo "ERROR: rsync not found. Expected at ~/bin/rsync.exe"
    exit 1
fi

# Verify Pi is reachable
if ! ssh -o ConnectTimeout=5 "$PI_USER@$PI_HOST" "exit" 2>/dev/null; then
    echo "ERROR: Cannot reach Pi at $PI_HOST"
    exit 1
fi

echo "Syncing to $PI_USER@$PI_HOST"
echo "----------------------------------------"

TOTAL_SENT=0
FAILED=()

for repo in "${REPOS[@]}"; do
    echo ""
    echo ">> $repo"
    result=$(rsync -a --delete --stats \
        "$LOCAL_BASE/$repo/" \
        "$PI_USER@$PI_HOST:$REMOTE_BASE/$repo/" 2>&1)

    if [ $? -eq 0 ]; then
        sent=$(echo "$result" | grep "Total transferred file size" | grep -o '[0-9,]*' | tr -d ',')
        files=$(echo "$result" | grep "Number of regular files transferred" | grep -o '[0-9]*' | head -1)
        echo "   files transferred: ${files:-0}  |  bytes sent: ${sent:-0}"
    else
        echo "   FAILED"
        FAILED+=("$repo")
    fi
done

echo ""
echo "----------------------------------------"
if [ ${#FAILED[@]} -eq 0 ]; then
    echo "All repos synced successfully."
else
    echo "Failed: ${FAILED[*]}"
    exit 1
fi
