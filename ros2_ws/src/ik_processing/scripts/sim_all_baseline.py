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
baseline_folder = data_dir / "baselines"

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

    time.sleep(2)

    for agentdir in baseline_folder.iterdir():
        agent = agentdir.name

        for pathdir in agentdir.iterdir():
            start, goal = pathdir.name.split("-")

            for exdir in pathdir.iterdir():
                exnum = exdir.name

                dat_files = sorted(exdir.glob("*.dat"))

                for dat in dat_files:
                    datafile = dat.stem

                    cmd = [
                        "ros2",
                        "launch",
                        "ik_processing",
                        "parse_and_sim.launch.py",

                        f"is_baseline:=true",
                        f"is_agent1:={'true' if agent == 'agent1' else 'false'}",

                        f"{'startpos1' if agent == 'agent1' else 'startpos2'}:={start}",
                        f"{'goalpos1' if agent == 'agent1'  else 'goalpos2'}:={goal}",

                        f"datafile:={datafile}",
                        f"exNum:={int(exnum)}",
                        f"use_rviz:=false",
                    ]

                    print("RUNNING:", " ".join(cmd))

                    subprocess.run(cmd, check=True)

finally:
    cleanup()