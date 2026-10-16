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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bare", action="store_true")
    parser.add_argument("--domain", type=int, default=86)
    parser.add_argument("--output", default="/tmp/opencode/arctos-gazebo-runtime")
    args = parser.parse_args()
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=True)
    name = "bare" if args.bare else "gripper"
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
               GZ_PARTITION=f"arctos-check-{uuid.uuid4().hex}")
    with (directory / f"{name}-launch.log").open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "arctos_gazebo", "demo.launch.py", "gui:=false",
            "rviz:=false", f"include_gripper:={str(not args.bare).lower()}",
        ], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            result = subprocess.run([
                "python3", str(Path(__file__).with_name("test_mock_moveit.py")),
                "--sim-time", "--hold-seconds", "5",
            ] + (["--bare"] if args.bare else []), env=env, timeout=240,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            (directory / f"{name}-checks.log").write_text(result.stdout)
            print(result.stdout, end="", flush=True)
            result.check_returncode()
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    print(f"PASS: {name} headless validation; evidence: {directory}")


if __name__ == "__main__":
    main()
