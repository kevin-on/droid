source /home/iliad/Utilities/miniconda3/etc/profile.d/conda.sh
conda activate expoft
# Args: $1 = robot_ip (Franka control box), $2 = polymetis controller port.
ROBOT_IP="${1:-172.16.0.2}"
PORT="${2:-50051}"

# Port-scoped cleanup: stop ONLY this arm's prior controller (matched by its
# unique controller port + robot IP) so a restart/orphan doesn't block the new
# launch. The other arm (different port + IP) is left untouched.
pkill -9 -f "run_server .*-p ${PORT}( |\$)" 2>/dev/null
for pid in $(pgrep -f franka_panda_client 2>/dev/null); do
    if ss -tnp 2>/dev/null | grep -E "pid=${pid}(,|\))" | grep -q "${ROBOT_IP}:"; then
        kill -9 "$pid" 2>/dev/null
    fi
done
sleep 1

launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip="$ROBOT_IP" port="$PORT"
