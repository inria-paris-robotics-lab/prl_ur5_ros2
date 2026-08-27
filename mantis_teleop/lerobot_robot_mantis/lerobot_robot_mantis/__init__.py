"""LeRobot third-party robot plugin for the PRL mantis (dual UR5, left arm).

The distribution name starts with `lerobot_robot_`, which lerobot auto-imports, so
importing this package registers `mantis_follower` as a --robot.type choice and the
stock CLI drives the real robot:

    lerobot-replay --robot.type=mantis_follower --dataset.repo_id=mantis/teleop
                   --dataset.root=<dataset dir> --dataset.episode=0

Needs the ROS2 environment sourced, and must not run while teleop_mantis.py does:
both publish the arm command topic.
"""
from .config_mantis_follower import MantisFollowerConfig
from .mantis_follower import MantisFollower

__all__ = ["MantisFollowerConfig", "MantisFollower"]
