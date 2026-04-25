#!/usr/bin/env python3
import yaml, subprocess, copy, os
from itertools import product
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import argparse
import time

def create_grid(joints, offsets, base_values):
    grid = []

    for combo in product(offsets, repeat=len(joints)):
        cfg = {}
        for joint, offset in zip(joints, combo):
            cfg[joint] = base_values[joint] + offset
        grid.append(cfg)

    return grid

parser = argparse.ArgumentParser()
parser.add_argument('--start_value', dest='start_value', type=int, help='Step where gridsearch shall be started. Default is at beginning (0).')
args = parser.parse_args()
if (args.start_value):
    startvalue = args.start_value
else:
    startvalue = 0

this_package = get_package_share_directory("ik_processing")
config_file_path = os.path.join(this_package, "config", "config.yaml")

with open(config_file_path) as f:
    base = yaml.safe_load(f)

joints = [
    "glenohumeral_yaw_joint",
    "glenohumeral_roll_joint",
    "sternoclavicular_yaw_joint",
    "sternoclavicular_pitch_joint"
]

base_joint_weights = base["Fullsim_Node"]["ros__parameters"]["joint_weights"]
values = [-0.5, -0.25, 0, 0.25, 0.5]
grid = create_grid(joints, values, base_joint_weights)

home = os.environ.get("HOME")
if not home:
    raise RuntimeError("HOME environment variable not set")
dump_dir_path = Path(home) / "Projects/Masterarbeit/simdata"
dump_path = dump_dir_path / "grid_search_data.json"

if dump_path.exists() and startvalue != 0:
    dump_path.unlink()
    print("\033[32mDeleted old grid seach file.\033[0m")

num_combinations = len(grid)

subprocess.Popen([
    "ros2", "launch", "ik_processing", "gridsearch.launch.py",
    "use_rviz:=false",
    "startpos1:=3",
    "goalpos1:=7",
    "num_requests:=" + str(num_combinations),
])

time.sleep(5)

print("Info: grid length:", num_combinations, "starting gridsearch")

tmp_path = "/tmp/grid_config.yaml" 

for i, params in enumerate(grid):
    if i < startvalue:
        print('skipping ', i)
        continue

    cfg = copy.deepcopy(base)

    for joint in joints:
        cfg["Fullsim_Node"]["ros__parameters"]["joint_weights"][joint] = params[joint]

    cfg["Fullsim_Node"]["ros__parameters"]["experiment_id"] = i

    with open(tmp_path, "w") as f:
        yaml.dump(cfg, f)

    print("\033[31mRunning Experiment ID: ", i, "/", num_combinations-1, "\033[0m")

    subprocess.run([
        "ros2", "launch", "ik_processing", "sim.launch.py",
        f"config_file:={tmp_path}",
        "use_rviz:=false",
        "grid_search:=true"
    ])