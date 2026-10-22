#!/usr/bin/env python3
"""Launch an isolated headless Gazebo demo, validate it, and shut it down.

Source the Jazzy workspace before running. Logs are retained on success/failure.
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import uuid
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bare", action="store_true")
    parser.add_argument("--scene", action="store_true", help="Validate table/cube and execute the approach")
    parser.add_argument("--grasp", action="store_true", help="With --scene, validate a physical lift/release")
    stages = parser.add_mutually_exclusive_group()
    for stage in ("transfer", "stack", "push"):
        stages.add_argument(f"--{stage}", action="store_true", help=f"Validate the {stage} physics checkpoint")
    parser.add_argument("--domain", type=int, default=86)
    parser.add_argument("--output", default="/tmp/opencode/arctos-gazebo-runtime")
    args = parser.parse_args()
    stage = next((name for name in ("transfer", "stack", "push") if getattr(args, name)), None)
    if stage:
        args.scene = True
    if args.grasp and not args.scene:
        parser.error("--grasp requires --scene")
    if args.scene and args.bare:
        parser.error("The approach checkpoint requires the gripper")
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=True)
    name = "bare" if args.bare else "gripper"
    scene_args = []
    if args.scene:
        name = stage or ("grasp" if args.grasp else "manipulation")
        scene_args = ["scene_config:=" + str(Path(get_package_share_directory("arctos_gazebo")) / "config" /
                                           ("stacking.yaml" if stage else "manipulation.yaml"))]
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
               GZ_PARTITION=f"arctos-check-{uuid.uuid4().hex}")
    with (directory / f"{name}-launch.log").open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "arctos_gazebo",
            "manipulation.launch.py" if args.scene else "demo.launch.py", "gui:=false",
            "rviz:=false", f"include_gripper:={str(not args.bare).lower()}",
        ] + scene_args, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            command = [
                "python3", str(Path(__file__).with_name("test_mock_moveit.py")),
                "--sim-time", "--hold-seconds", "5",
            ] + (["--bare"] if args.bare else [])
            if args.scene:
                command = ["python3", str(Path(__file__).with_name("test_manipulation.py")), "--execute"]
                if args.grasp:
                    command.append("--grasp")
                if stage:
                    command.append(f"--{stage}")
            try:
                result = subprocess.run(command, env=env, timeout=420 if stage else 240,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            except subprocess.TimeoutExpired as error:
                output = error.stdout or b""
                if isinstance(output, bytes):
                    output = output.decode(errors="replace")
                (directory / f"{name}-checks.log").write_text(output + "\nValidation timed out\n")
                raise
            (directory / f"{name}-checks.log").write_text(result.stdout)
            print(result.stdout, end="", flush=True)
            result.check_returncode()
        finally:
            # Launch may exit early while its Gazebo/bridge children remain.
            # Always signal our isolated process group, not just a live parent.
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            if process.poll() is None:
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    print(f"PASS: {name} headless validation; evidence: {directory}")


if __name__ == "__main__":
    main()
