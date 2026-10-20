"""Compile and execute real master/steering firmware against hardware mocks."""
import argparse
import os
import pathlib
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", default="g++")
parser.add_argument("--build-dir", required=True)
args = parser.parse_args()
root = pathlib.Path(__file__).resolve().parent
build = pathlib.Path(args.build_dir).resolve()
build.mkdir(parents=True, exist_ok=True)
env = os.environ.copy()
env["PATH"] = str(pathlib.Path(args.compiler).resolve().parent) + os.pathsep + env.get("PATH", "")
cases = [("master_regression", "master_regression", []),
         ("steering_profile_regression", "steering_profile_regression", []),
         ("steer_limit_test_regression", "steer_limit_test_regression", []),
         ("steer_sensor_test_regression", "steer_sensor_test_regression", []),
         ("watchdog_tick_regression", "watchdog_tick_regression", []),
         ("can_bus_regression", "can_bus_regression", []),
         ("can_pair_regression", "can_pair_regression", []),
         ("vehicle_pair_regression", "vehicle_pair_regression", []),
         ("jog_regression", "jog_regression", []),
         ("steer_regression", "steer_regression", []),
         ("can_link_sender", "can_link_regression", ["-DCAN_LINK_SENDER=1"]),
         ("can_link_responder", "can_link_regression", ["-DCAN_LINK_SENDER=0"])]
for name, source, defines in cases:
    exe = build / (name + ".exe")
    subprocess.run([args.compiler, "-std=c++11", "-Wall", "-Wextra", *defines,
                    "-I", str(root / "firmware_mocks"), str(root / (source + ".cpp")),
                    "-static", "-o", str(exe)], check=True, env=env)
    subprocess.run([str(exe)], check=True, timeout=10, env=env)
