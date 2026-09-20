# ROBOT SPECIFIC IMPORTS
import os
import subprocess
import time

import gevent
import grpc
import numpy as np
import torch
from polymetis import GripperInterface, RobotInterface

from droid.misc.parameters import sudo_password
from droid.misc.subprocess_utils import run_threaded_command

# UTILITY SPECIFIC IMPORTS
from droid.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler
from droid.robot_ik.robot_ik_solver import RobotIKSolver


class FrankaRobot:
    def __init__(
        self,
        robot_ip="172.16.0.2",
        robot_port=50051,
        gripper_comport="/dev/ttyUSB0",
        gripper_port=50052,
        *,
        gripper_device=None,
    ):
        # Preserve the NUC's positional API and accept EXPO-FT's keyword alias.
        if gripper_device is not None:
            if gripper_comport != "/dev/ttyUSB0" and gripper_comport != gripper_device:
                raise ValueError("Conflicting gripper_comport and gripper_device")
            gripper_comport = gripper_device
        # Per-arm settings so one package can drive multiple robots (e.g. left/right).
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.gripper_comport = gripper_comport
        self.gripper_port = gripper_port
        self._controller_processes = []
        self._server_launched = False

    def launch_controller(self):
        # Only stop processes started by this object. Existing controllers can be
        # reused through ServerInterface(launch=False); never kill by port/name.
        self.kill_controller()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        launches = (
            ("launch_robot.sh", [self.robot_ip, str(self.robot_port)], f"/tmp/droid_robot_{self.robot_port}.log"),
            ("launch_gripper.sh", [self.gripper_comport, str(self.gripper_port)],
             f"/tmp/droid_gripper_{self.gripper_port}.log"),
        )
        try:
            for script, args, log_path in launches:
                # Inherit a real log file, not an unread PIPE that can stall the
                # controller. Keep authentication out of the command line.
                with open(log_path, "ab") as log:
                    process = subprocess.Popen(
                        ["sudo", "-S", "bash", os.path.join(dir_path, script), *args],
                        stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
                        text=True, start_new_session=True,
                    )
                self._controller_processes.append(process)
                try:
                    process.stdin.write(sudo_password + "\n")
                finally:
                    process.stdin.close()
            gevent.sleep(5)  # cooperative: preserve the ZeroRPC heartbeat
            for process, (_, _, log_path) in zip(self._controller_processes, launches):
                if process.poll() is not None:
                    raise RuntimeError(f"Controller launcher exited; check {log_path}")
            self._server_launched = True
        except BaseException:
            self.kill_controller()
            raise

    def launch_robot(self):
        # The polymetis controller + gripper servers (started by launch_controller)
        # can take several seconds to come up. Retry connecting instead of failing
        # immediately with gRPC "failed to connect to all addresses".
        self._robot = self._connect_with_retry(
            lambda: RobotInterface(ip_address="localhost", port=self.robot_port)
        )
        def _gripper_factory():
            g = GripperInterface(ip_address="localhost", port=self.gripper_port)
            # GripperInterface SWALLOWS the "server not ready" gRPC error: it logs
            # "Metadata unavailable from server" and returns without setting .metadata.
            # Raise so _connect_with_retry waits for the gripper server to be ready,
            # instead of crashing on `.metadata.max_width` on the next line.
            if not hasattr(g, "metadata"):
                raise RuntimeError("gripper server not ready (metadata unavailable)")
            return g

        self._gripper = self._connect_with_retry(_gripper_factory)
        self._max_gripper_width = self._gripper.metadata.max_width
        self._ik_solver = RobotIKSolver()
        self._controller_not_loaded = False

    @staticmethod
    def _connect_with_retry(factory, timeout=30, interval=1.0):
        deadline = time.time() + timeout
        while True:
            try:
                return factory()
            except Exception:
                if time.time() > deadline:
                    raise
                gevent.sleep(interval)  # cooperative: keep the zerorpc heartbeat alive while retrying

    def kill_controller(self):
        # Keep a failed stop in the list so a caller can retry it. No unrelated
        # arm, gripper, or pre-existing controller is selected for termination.
        while self._controller_processes:
            process = self._controller_processes[-1]
            if process.poll() is None:
                subprocess.run(
                    ["sudo", "-S", "kill", "-TERM", "--", f"-{process.pid}"],
                    input=sudo_password + "\n", text=True, check=True, timeout=10,
                )
                deadline = time.monotonic() + 10
                while process.poll() is None:
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"Controller process group {process.pid} did not stop")
                    gevent.sleep(0.05)
            self._controller_processes.pop()
        self._server_launched = False

    def update_command(self, command, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        action_dict = self.create_action_dict(command, action_space=action_space, gripper_action_space=gripper_action_space)

        self.update_joints(action_dict["joint_position"], velocity=False, blocking=blocking)
        self.update_gripper(action_dict["gripper_position"], velocity=False, blocking=blocking)

        return action_dict

    def update_pose(self, command, velocity=False, blocking=False):
        if blocking:
            if velocity:
                curr_pose = self.get_ee_pose()
                cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(command)
                command = add_poses(cartesian_delta, curr_pose)

            pos = torch.Tensor(command[:3])
            quat = torch.Tensor(euler_to_quat(command[3:6]))
            curr_joints = self._robot.get_joint_positions()
            desired_joints = self._robot.solve_inverse_kinematics(pos, quat, curr_joints)
            self.update_joints(desired_joints, velocity=False, blocking=True)
        else:
            if not velocity:
                curr_pose = self.get_ee_pose()
                cartesian_delta = pose_diff(command, curr_pose)
                command = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)

            robot_state = self.get_robot_state()[0]
            joint_velocity = self._ik_solver.cartesian_velocity_to_joint_velocity(command, robot_state=robot_state)

            self.update_joints(joint_velocity, velocity=True, blocking=False)

    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        if cartesian_noise is not None:
            command = self.add_noise_to_joints(command, cartesian_noise)
        command = torch.Tensor(command)

        if velocity:
            joint_delta = self._ik_solver.joint_velocity_to_delta(command)
            command = joint_delta + self._robot.get_joint_positions()

        def helper_non_blocking():
            if not self._robot.is_running_policy():
                self._controller_not_loaded = True
                self._robot.start_cartesian_impedance()
                timeout = time.time() + 5
                while not self._robot.is_running_policy():
                    time.sleep(0.01)
                    if time.time() > timeout:
                        self._robot.start_cartesian_impedance()
                        timeout = time.time() + 5

                self._controller_not_loaded = False
            try:
                self._robot.update_desired_joint_positions(command)
            except grpc.RpcError:
                pass

        if blocking:
            if self._robot.is_running_policy():
                self._robot.terminate_current_policy()
            try:
                time_to_go = self.adaptive_time_to_go(command)
                self._robot.move_to_joint_positions(command, time_to_go=time_to_go)
            except grpc.RpcError:
                pass

            self._robot.start_cartesian_impedance()
        else:
            if not self._controller_not_loaded:
                run_threaded_command(helper_non_blocking)

    def update_gripper(self, command, velocity=True, blocking=False):
        if velocity:
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(command)
            command = gripper_delta + self.get_gripper_position()

        command = float(np.clip(command, 0, 1))
        self._gripper.goto(width=self._max_gripper_width * (1 - command), speed=0.1, force=70.0, blocking=blocking)

    def add_noise_to_joints(self, original_joints, cartesian_noise):
        original_joints = torch.Tensor(original_joints)

        pos, quat = self._robot.robot_model.forward_kinematics(original_joints)
        curr_pose = pos.tolist() + quat_to_euler(quat).tolist()
        new_pose = add_poses(cartesian_noise, curr_pose)

        new_pos = torch.Tensor(new_pose[:3])
        new_quat = torch.Tensor(euler_to_quat(new_pose[3:]))

        noisy_joints, success = self._robot.solve_inverse_kinematics(new_pos, new_quat, original_joints)

        if success:
            desired_joints = noisy_joints
        else:
            desired_joints = original_joints

        return desired_joints.tolist()

    def get_joint_positions(self):
        return self._robot.get_joint_positions().tolist()

    def get_joint_velocities(self):
        return self._robot.get_joint_velocities().tolist()

    def get_gripper_position(self):
        return self._gripper.get_state().width / self._max_gripper_width

    def get_ee_pose(self):
        pos, quat = self._robot.get_ee_pose()
        angle = quat_to_euler(quat.numpy())
        return np.concatenate([pos, angle]).tolist()

    def get_robot_state(self):
        robot_state = self._robot.get_robot_state()
        gripper_position = self.get_gripper_position()
        pos, quat = self._robot.robot_model.forward_kinematics(torch.Tensor(robot_state.joint_positions))
        cartesian_position = pos.tolist() + quat_to_euler(quat.numpy()).tolist()

        state_dict = {
            "cartesian_position": cartesian_position,
            "gripper_position": gripper_position,
            "joint_positions": list(robot_state.joint_positions),
            "joint_velocities": list(robot_state.joint_velocities),
            "joint_torques_computed": list(robot_state.joint_torques_computed),
            "prev_joint_torques_computed": list(robot_state.prev_joint_torques_computed),
            "prev_joint_torques_computed_safened": list(robot_state.prev_joint_torques_computed_safened),
            "motor_torques_measured": list(robot_state.motor_torques_measured),
            "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
            "prev_command_successful": robot_state.prev_command_successful,
        }

        timestamp_dict = {
            "robot_timestamp_seconds": robot_state.timestamp.seconds,
            "robot_timestamp_nanos": robot_state.timestamp.nanos,
        }

        return state_dict, timestamp_dict

    def adaptive_time_to_go(self, desired_joint_position, t_min=0, t_max=10):
        curr_joint_position = self._robot.get_joint_positions()
        displacement = desired_joint_position - curr_joint_position
        time_to_go = self._robot._adaptive_time_to_go(displacement)
        clamped_time_to_go = min(t_max, max(time_to_go, t_min))
        return clamped_time_to_go

    def create_action_dict(self, action, action_space, gripper_action_space=None, robot_state=None):
        assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
        if robot_state is None:
            robot_state = self.get_robot_state()[0]
        action_dict = {"robot_state": robot_state}
        velocity = "velocity" in action_space

        if gripper_action_space is None:
            gripper_action_space = "velocity" if velocity else "position"
        assert gripper_action_space in ["velocity", "position"]
            

        if gripper_action_space == "velocity":
            action_dict["gripper_velocity"] = action[-1]
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(action[-1])
            gripper_position = (1 - robot_state["gripper_position"]) + gripper_delta
            action_dict["gripper_position"] = float(np.clip(gripper_position, 0, 1))
        else:
            action_dict["gripper_position"] = float(np.clip(action[-1], 0, 1))
            gripper_delta = action_dict["gripper_position"] - robot_state["gripper_position"]
            gripper_velocity = self._ik_solver.gripper_delta_to_velocity(gripper_delta)
            action_dict["gripper_delta"] = gripper_velocity

        if "cartesian" in action_space:
            if velocity:
                action_dict["cartesian_velocity"] = action[:-1]
                cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(action[:-1])
                action_dict["cartesian_position"] = add_poses(
                    cartesian_delta, robot_state["cartesian_position"]
                ).tolist()
            else:
                action_dict["cartesian_position"] = action[:-1]
                cartesian_delta = pose_diff(action[:-1], robot_state["cartesian_position"])
                cartesian_velocity = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
                action_dict["cartesian_velocity"] = cartesian_velocity.tolist()

            action_dict["joint_velocity"] = self._ik_solver.cartesian_velocity_to_joint_velocity(
                action_dict["cartesian_velocity"], robot_state=robot_state
            ).tolist()
            joint_delta = self._ik_solver.joint_velocity_to_delta(action_dict["joint_velocity"])
            action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()

        if "joint" in action_space:
            # NOTE: Joint to Cartesian has undefined dynamics due to IK
            if velocity:
                action_dict["joint_velocity"] = action[:-1]
                joint_delta = self._ik_solver.joint_velocity_to_delta(action[:-1])
                action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
            else:
                action_dict["joint_position"] = action[:-1]
                joint_delta = np.array(action[:-1]) - np.array(robot_state["joint_positions"])
                joint_velocity = self._ik_solver.joint_delta_to_velocity(joint_delta)
                action_dict["joint_velocity"] = joint_velocity.tolist()

        return action_dict
