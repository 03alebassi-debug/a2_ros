#!/bin/bash
set -euo pipefail

# ----- Mission type -----
MISSION=""
while [[ -z "$MISSION" ]]; do
    read -rp "Enter the mission (exp, nav, ...): " MISSION
done

EXP_TIME=60
if [[ "$MISSION" == "exp" ]]; then
    MISSION="exploration"
    read -rp "Enter the exploration time in seconds [60]: " input
    EXP_TIME="${input:-60}"
    if [[ "$EXP_TIME" -lt 0 ]]; then
        echo "ERROR: exploration time cannot be negative"
        exit 1
    fi
fi

# ----- Simulation or real -----
SIM=""
while [[ "$SIM" != "sim" && "$SIM" != "real" ]]; do
    read -rp "Simulation or real? (sim/real): " SIM
done

MAP=""
if [[ "$SIM" == "sim" ]]; then
    read -rp "Enter the scene file (leave empty for default): " MAP
fi

# ----- tmux session -----
SESSION="bringup"

command -v tmux >/dev/null 2>&1 || { echo "tmux not installed"; exit 1; }

if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "sim/nuc"
if [[ "$SIM" == "real" ]]; then
    tmux send-keys -t "$SESSION:sim/nuc" "ros2 launch a2_ros nuc.launch.py" Enter
else
    if [[ -z "$MAP" ]]; then
        tmux send-keys -t "$SESSION:sim/nuc" "a2 sim " Enter
    else
        tmux send-keys -t "$SESSION:sim/nuc" "a2 sim --scene $MAP " Enter
    fi
fi

tmux new-window -t "$SESSION" -n "prep"
tmux send-keys -t "$SESSION:prep" "sleep 3 && a2 stand && a2 unlock && a2 walk" Enter

tmux new-window -t "$SESSION" -n "dlio"
tmux send-keys -t "$SESSION:dlio" "sleep 6 && a2 dlio --rviz" Enter

#tmux new-window -t "$SESSION" -n "detect"
#tmux send-keys -t "$SESSION:detect" "sleep 6 && a2 detect" Enter

tmux new-window -t "$SESSION" -n "mission"
if [[ "$MISSION" == "exploration" ]]; then
    tmux send-keys -t "$SESSION:mission" "sleep 8 && ros2 launch a2_ros exploration.launch.py exploration_duration:=$EXP_TIME" Enter
else
    tmux send-keys -t "$SESSION:mission" "sleep 8 && a2 $MISSION" Enter
fi

tmux select-window -t "$SESSION:mission"
if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux -2 attach-session -t "$SESSION"
fi
