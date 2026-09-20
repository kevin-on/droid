#!/usr/bin/env bash
set -e
source /home/iliad/Utilities/miniconda3/etc/profile.d/conda.sh
conda activate expoft
# Args: $1 = Franka IP, $2 = Polymetis arm port.
# The Python owner handles its own process groups. An occupied port must be
# resolved explicitly or reused with launch_controller=false.
exec launch_robot.py robot_client=franka_hardware \
    "robot_client.executable_cfg.robot_ip=${1:-172.16.0.2}" "port=${2:-50051}"
