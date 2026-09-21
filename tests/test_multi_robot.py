"""Offline regression tests: no SDK import, device access, sudo or controller launch."""
import ast
import fcntl
import tempfile
from copy import deepcopy
import io
import os
from pathlib import Path
import runpy
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_definition(relative, name, namespace):
    path = ROOT / relative
    node = next(node for node in ast.parse(path.read_text()).body if getattr(node, "name", None) == name)
    namespace["__file__"] = str(path)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), namespace)
    return namespace[name]


@pytest.fixture
def hardware():
    processes, kills, sleeps, interfaces, logs = [], [], [], [], []
    clock = [0.0]

    def sleep(seconds):
        sleeps.append(seconds)
        clock[0] += seconds

    class Process:
        def __init__(self, command, **kwargs):
            assert kwargs["start_new_session"]
            assert kwargs["stderr"] == subprocess.STDOUT
            self.pid = 100 + len(processes)
            self.command, self.options = command, kwargs
            self.stdin = io.StringIO()
            self.returncode = None
            processes.append(self)

        def poll(self):
            return self.returncode

    def kill(command, **kwargs):
        assert command[:5] == ["sudo", "-S", "kill", "-TERM", "--"]
        assert kwargs["check"]
        kills.append(command[-1])
        pid = -int(command[-1])
        next(process for process in processes if process.pid == pid).returncode = 0

    def open_log(path, mode):
        assert mode == "ab"
        logs.append(path)
        return io.BytesIO()

    def interface(**kwargs):
        interfaces.append(kwargs)
        return SimpleNamespace(metadata=SimpleNamespace(max_width=0.085))

    namespace = dict(
        os=os, subprocess=SimpleNamespace(Popen=Process, PIPE=subprocess.PIPE, STDOUT=subprocess.STDOUT, run=kill),
        time=SimpleNamespace(time=lambda: clock[0], monotonic=lambda: clock[0]),
        gevent=SimpleNamespace(sleep=sleep), open=open_log, sudo_password="test-credential",
        RobotInterface=interface, GripperInterface=interface, RobotIKSolver=lambda: "ik", np=np,
    )
    cls = load_definition("droid/franka/robot.py", "FrankaRobot", namespace)
    return SimpleNamespace(cls=cls, namespace=namespace, processes=processes, kills=kills,
                           sleeps=sleeps, interfaces=interfaces, logs=logs)


def test_two_arms_keep_independent_processes_ports_and_logs(hardware):
    h = hardware
    # Preserve the NUC's original positional order; also accept the fork keyword.
    first = h.cls("172.16.0.2", 50053, "/dev/serial/by-id/gripper-A", 50054)
    second = h.cls("172.16.0.3", 50051, gripper_port=50052, gripper_device="/dev/serial/by-id/gripper-B")
    first.launch_controller()
    second.launch_controller()
    first.launch_robot()
    second.launch_robot()
    assert [call["port"] for call in h.interfaces] == [50053, 50054, 50051, 50052]
    assert h.processes[0].command[-2:] == ["172.16.0.2", "50053"]
    assert h.processes[3].command[-2:] == ["/dev/serial/by-id/gripper-B", "50052"]
    assert len(set(h.logs)) == 4
    assert all("test-credential" not in " ".join(p.command) for p in h.processes)
    assert h.sleeps == [5, 5]
    second.kill_controller()
    second.kill_controller()
    assert h.kills == ["-103", "-102"]
    assert all(p.poll() is None for p in h.processes[:2])
    assert len(first._controller_processes) == 2
    assert not second._server_launched


def test_partial_start_failure_cleans_up_only_created_process(hardware):
    h = hardware
    original = h.namespace["subprocess"].Popen

    def fail_second(*args, **kwargs):
        if h.processes:
            raise OSError("gripper launcher failed")
        return original(*args, **kwargs)

    h.namespace["subprocess"].Popen = fail_second
    robot = h.cls()
    with pytest.raises(OSError, match="gripper"):
        robot.launch_controller()
    assert h.kills == ["-100"]
    assert not robot._controller_processes
    assert not robot._server_launched


def test_early_launcher_exit_reports_log_and_stops_sibling(hardware):
    h = hardware
    original = h.namespace["subprocess"].Popen

    def exited_first(*args, **kwargs):
        process = original(*args, **kwargs)
        if len(h.processes) == 1:
            process.returncode = 1
        return process

    h.namespace["subprocess"].Popen = exited_first
    robot = h.cls()
    with pytest.raises(RuntimeError, match="/tmp/droid_robot_50051.log"):
        robot.launch_controller()
    assert h.kills == ["-101"]
    assert not robot._controller_processes


def test_stop_failure_keeps_ownership_for_retry(hardware):
    h = hardware
    robot = h.cls()
    robot.launch_controller()
    original = h.namespace["subprocess"].run

    def fail(*args, **kwargs):
        raise subprocess.CalledProcessError(1, args[0])

    h.namespace["subprocess"].run = fail
    with pytest.raises(subprocess.CalledProcessError):
        robot.kill_controller()
    assert len(robot._controller_processes) == 2
    h.namespace["subprocess"].run = original
    robot.kill_controller()
    assert h.kills == ["-101", "-100"]


def test_retry_and_missing_gripper_metadata_keep_heartbeat_alive(hardware):
    h = hardware
    calls = []

    def gripper(**kwargs):
        calls.append(kwargs)
        if len(calls) < 3:
            return SimpleNamespace()
        return SimpleNamespace(metadata=SimpleNamespace(max_width=0.085))

    h.namespace["GripperInterface"] = gripper
    robot = h.cls(gripper_port=50054)
    robot.launch_robot()
    assert len(calls) == 3 and all(call["port"] == 50054 for call in calls)
    assert h.sleeps == [1.0, 1.0]
    assert robot._max_gripper_width == 0.085

    def unavailable():
        raise RuntimeError("unavailable")

    with pytest.raises(RuntimeError, match="unavailable"):
        robot._connect_with_retry(unavailable, timeout=2)


def test_existing_gripper_tuning_is_preserved(hardware):
    robot = hardware.cls()
    commands = []
    robot._max_gripper_width = 0.085
    robot._gripper = SimpleNamespace(goto=lambda **kwargs: commands.append(kwargs))
    robot.update_gripper(0.5, velocity=False)
    assert commands == [dict(width=0.0425, speed=0.1, force=70.0, blocking=False)]
    with pytest.raises(ValueError, match="Conflicting"):
        hardware.cls(gripper_comport="/dev/A", gripper_device="/dev/B")


@pytest.mark.parametrize("launch,expected", [(False, ["attach"]), (True, ["start", "attach"])])
def test_rpc_routing_and_attach_mode(launch, expected):
    events, endpoints, heartbeats = [], [], []
    rpc = SimpleNamespace(connect=endpoints.append, launch_controller=lambda: events.append("start"),
                          launch_robot=lambda: events.append("attach"))

    def client(**kwargs):
        heartbeats.append(kwargs["heartbeat"])
        return rpc

    namespace = dict(zerorpc=SimpleNamespace(Client=client), time=SimpleNamespace(sleep=lambda _: None))
    namespace["attempt_n_times"] = load_definition("droid/misc/server_interface.py", "attempt_n_times", namespace)
    cls = load_definition("droid/misc/server_interface.py", "ServerInterface", namespace)
    cls("172.16.0.1", 4243, launch=launch)
    assert endpoints == ["tcp://172.16.0.1:4243"]
    assert heartbeats == [30] and events == expected


def test_camera_selection_checks_missing_serials_before_wrapping_any_camera():
    opened = []
    namespace = dict(
        sl=SimpleNamespace(Camera=SimpleNamespace(
            get_device_list=lambda: [SimpleNamespace(serial_number=i) for i in (11, 12, 21, 22)])),
        ZedCamera=lambda camera, wrist_camera_serial=None: opened.append((camera.serial_number, wrist_camera_serial)),
    )
    gather = load_definition("droid/camera_utils/camera_readers/zed_camera.py", "gather_zed_cameras", namespace)
    gather(["21", 22], "22")
    assert opened == [(21, "22"), (22, "22")]
    opened.clear()
    with pytest.raises(ValueError, match="99"):
        gather(["21", "99"], "99")
    assert opened == []


@pytest.mark.parametrize("side_key", ["varied_camera", "static_camera"])
def test_robot_camera_roles_override_global_mapping(side_key):
    cameras = [SimpleNamespace(serial_number=str(i), is_hand_camera=(i == 22),
                set_trajectory_mode=lambda: None, high_res_calibration=False) for i in (21, 22)]
    settings = {}
    for cam in cameras:
        cam.set_reading_parameters = lambda serial=cam.serial_number, **kw: settings.update({serial: kw})
    namespace = dict(os=os, fcntl=fcntl, tempfile=tempfile,
                     gather_zed_cameras=lambda *args: cameras, get_camera_type=lambda _: "wrong-global-role")
    wrapper = load_definition("droid/camera_utils/wrappers/multi_camera_wrapper.py", "MultiCameraWrapper", namespace)
    wrapper({"hand_camera": {"left_only": True}, side_key: {"depth": False}}, ["21", "22"], "22")
    assert settings == {"21": {"depth": False}, "22": {"left_only": True}}


def environment_class():
    connections, readers = [], []
    calibration = {"12_left": [12], "22_left": [22]}
    namespace = dict(
        gym=SimpleNamespace(Env=object), np=np, nuc_ip="172.16.0.1", hand_camera_id="12",
        ServerInterface=lambda **kwargs: connections.append(kwargs),
        MultiCameraWrapper=lambda *args: readers.append(args),
        load_calibration_info=lambda: calibration, camera_type_dict={"12": 0}, deepcopy=deepcopy,
        change_pose_frame=lambda extrinsics, pose: ["updated", extrinsics, pose],
    )
    cls = load_definition("droid/robot_env.py", "RobotEnv", namespace)
    return cls, connections, readers


def test_environment_routes_selected_robot_and_wrist_calibration():
    cls, connections, readers = environment_class()
    env = cls(robot_server_port=4243, launch_controller=False,
              camera_serials=[21, 22], wrist_camera_serial="22", camera_kwargs={"static_camera": {"depth": False}})
    assert connections == [dict(ip_address="172.16.0.1", port=4243, launch=False)]
    assert readers == [({"static_camera": {"depth": False}}, ["21", "22"], "22")]
    assert env.camera_type_dict == {"21": 1, "22": 0}
    assert env.control_hz == 30
    extrinsics = env.get_camera_extrinsics({"cartesian_position": [0]})
    assert extrinsics["12_left"] == [12]
    assert extrinsics["22_left"][0] == "updated"
    assert extrinsics["22_left_gripper_offset"] == [22]


def test_legacy_environment_port_and_invalid_overrides():
    cls, connections, _ = environment_class()
    cls(server_port=4243)
    assert connections[-1]["port"] == 4243
    connections.clear()
    with pytest.raises(ValueError, match="Conflicting"):
        cls(server_port=4243, robot_server_port=4244)
    with pytest.raises(ValueError, match="wrist"):
        cls(camera_serials=["21"], wrist_camera_serial="22")
    assert connections == []


@pytest.mark.parametrize("port_flag,device_flag", [
    ("--zerorpc-port", "--gripper-comport"), ("--port", "--gripper-device"),
])
def test_server_cli_aliases_and_cleanup(monkeypatch, port_flag, device_flag):
    events = []

    class Robot:
        def __init__(self, **kwargs):
            events.append(("robot", kwargs))

        def kill_controller(self):
            events.append(("stop",))

    class Server:
        def __init__(self, robot, **kwargs):
            assert kwargs == {"heartbeat": 30}

        def bind(self, endpoint):
            events.append(("bind", endpoint))

        def run(self):
            raise RuntimeError("end fake server")

    monkeypatch.setitem(sys.modules, "zerorpc", SimpleNamespace(Server=Server))
    monkeypatch.setitem(sys.modules, "droid.franka.robot", SimpleNamespace(FrankaRobot=Robot))
    monkeypatch.setattr(sys, "argv", ["run_server.py", port_flag, "4243", "--robot-ip", "172.16.0.3",
                                    "--robot-port", "50051", "--gripper-port", "50052",
                                    device_flag, "/dev/serial/by-id/gripper-B"])
    with pytest.raises(RuntimeError, match="end fake server"):
        runpy.run_path(str(ROOT / "scripts/server/run_server.py"), run_name="__main__")
    assert events[0] == ("robot", dict(robot_ip="172.16.0.3", robot_port=50051,
                                     gripper_port=50052, gripper_comport="/dev/serial/by-id/gripper-B"))
    assert events[1:] == [("bind", "tcp://0.0.0.0:4243"), ("stop",)]


@pytest.mark.parametrize("blanks,expected", [(["22"], ["21"]), (["21", "22"], [])])
def test_blank_cameras_are_excluded_from_device_opening(blanks, expected):
    cls, connections, readers = environment_class()
    env = cls(camera_serials=["21", "22"], wrist_camera_serial="22", blank_camera_serials=blanks)
    assert readers == [({}, expected, "22")]
    assert env.camera_type_dict == {"21": 1, "22": 0}
    assert env.blank_camera_serials == tuple(blanks)


@pytest.mark.parametrize("serials,blanks", [(None, ["22"]), (["21", "22"], ["99"]),
                                          (["21", "22"], "22")])
def test_invalid_blank_selection_fails_before_hardware_access(serials, blanks):
    cls, connections, readers = environment_class()
    with pytest.raises(ValueError, match="blank_camera_serials"):
        cls(camera_serials=serials, wrist_camera_serial="22", blank_camera_serials=blanks)
    assert not connections and not readers
