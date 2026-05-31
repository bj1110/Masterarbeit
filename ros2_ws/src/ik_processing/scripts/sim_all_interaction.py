#!/usr/bin/env python3

import subprocess
import os
import signal
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import time

package_share = Path(get_package_share_directory('ik_processing'))

data_dir = package_share / "data" / "dset"
interaction_folder = data_dir / "interaction"

moveit_proc = None


def cleanup():
    global moveit_proc

    if moveit_proc is not None:
        print("Stopping moveit launch...")

        try:
            os.killpg(os.getpgid(moveit_proc.pid), signal.SIGTERM)
            moveit_proc.wait(timeout=5)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            print("Force killing moveit launch...")
            os.killpg(os.getpgid(moveit_proc.pid), signal.SIGKILL)


def signal_handler(sig, frame):
    cleanup()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

try:
    # launch moveit
    moveit_proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ik_processing",
            "moveit.launch.py",
            "use_rviz:=false",
        ],
        preexec_fn=os.setsid,
    )

    time.sleep(10)

    MAX_ITER= 100
    iter=0

    for paths in interaction_folder.iterdir():
        agent1, agent2 = paths.name.split("_")

        a1_start, a1_goal = agent1.split("-")
        a2_start, a2_goal = agent2.split("-")

        for exdir in paths.iterdir():
            exnum= exdir.name
            dat_files = sorted(exdir.glob("*.dat"))
            for dat in dat_files:
                datafile = dat.stem
                for ag in ["agent1", "agent2"]:

                    cmd = [
                        "ros2",
                        "launch",
                        "ik_processing",
                        "parse_and_sim.launch.py",

                        f"is_baseline:=false",
                        f"is_agent1:={'true' if ag == 'agent1' else 'false'}",

                        f"startpos1:={a1_start}",
                        f"goalpos1:={a1_goal}",
                        f"startpos2:={a2_start}",
                        f"goalpos2:={a2_goal}",

                        f"datafile:={datafile}",
                        f"exNum:={int(exnum)}",
                        f"use_rviz:=false",
                    ]

                    print("RUNNING:", " ".join(cmd))
                    
                    iter +=1

                    try:
                        subprocess.run(cmd, check=True)
                    except subprocess.CalledProcessError as e:
                        print(f"FAILED: {dat}")
                        print(e)
                        continue

                    if iter%MAX_ITER ==0 :
                        print("relaunching moveit")
                        cleanup()
                        
                        moveit_proc = subprocess.Popen(
                            [
                                "ros2",
                                "launch",
                                "ik_processing",
                                "moveit.launch.py",
                                "use_rviz:=false",
                            ],
                            preexec_fn=os.setsid,
                        )

                        time.sleep(10)


finally:
    cleanup()