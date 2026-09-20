import argparse

import zerorpc

from droid.franka.robot import FrankaRobot

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Launch a DROID zerorpc robot server for one arm.")
    parser.add_argument("--zerorpc-port", type=int, default=4242, help="Port the zerorpc clients connect to.")
    parser.add_argument("--robot-ip", default="172.16.0.2", help="Franka control-box IP of this arm.")
    parser.add_argument("--robot-port", type=int, default=50051, help="Local polymetis controller port for this arm.")
    parser.add_argument("--gripper-comport", default="/dev/ttyUSB0", help="Robotiq gripper serial device for this arm.")
    parser.add_argument("--gripper-port", type=int, default=50052, help="Local polymetis gripper server port for this arm.")
    args = parser.parse_args()

    robot_client = FrankaRobot(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        gripper_comport=args.gripper_comport,
        gripper_port=args.gripper_port,
    )
    # heartbeat must match the client (server_interface.py) and be generous enough that a
    # slow controller bring-up (launch_controller + launch_robot retries) doesn't trip
    # "Lost remote" mid-call. Default is 5s (lost after 10s) -> far too short here.
    s = zerorpc.Server(robot_client, heartbeat=30)
    s.bind(f"tcp://0.0.0.0:{args.zerorpc_port}")
    print(
        f"server -> tcp://0.0.0.0:{args.zerorpc_port} | robot_ip={args.robot_ip} "
        f"ctrl_port={args.robot_port} gripper={args.gripper_comport}:{args.gripper_port}",
        flush=True,
    )
    s.run()
