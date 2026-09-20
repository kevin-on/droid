#!/usr/bin/env bash
set -e
source /home/iliad/Utilities/miniconda3/etc/profile.d/conda.sh
conda activate expoft
# Args: $1 = stable gripper serial path, $2 = Polymetis gripper port.
device="${1:-/dev/ttyUSB0}"
chmod a+rw "$device"
exec launch_gripper.py gripper=robotiq_2f "gripper.comport=$device" "port=${2:-50052}"
