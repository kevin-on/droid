source /home/iliad/Utilities/miniconda3/etc/profile.d/conda.sh
conda activate expoft
# Args: $1 = gripper serial comport, $2 = polymetis gripper server port.
COMPORT="${1:-/dev/ttyUSB0}"
PORT="${2:-50052}"

# Port/device-scoped cleanup: stop ONLY this gripper's prior server (matched by
# its unique server port + serial device) so a restart/orphan doesn't block the
# new launch. The other arm's gripper (different port + device) is left untouched.
pkill -9 -f "launch_gripper.py.*port=${PORT}( |\$)" 2>/dev/null
pkill -9 -f "run_server .*-p ${PORT}( |\$)" 2>/dev/null
fuser -k "$(readlink -f "$COMPORT")" 2>/dev/null
sleep 1

chmod a+rw "$COMPORT"
launch_gripper.py gripper=robotiq_2f gripper.comport="$COMPORT" port="$PORT"
