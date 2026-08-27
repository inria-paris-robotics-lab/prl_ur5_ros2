from dataclasses import dataclass

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("mantis_follower")
@dataclass
class MantisFollowerConfig(RobotConfig):
    """Robot specifics come from the teleop's own YAML, so the two stacks cannot disagree."""
    teleop_config: str = "/home/ros/share/config_teleop_mantis.yaml"
    approach_vel: float = 0.3
    approach_tol: float = 0.05
    connect_timeout: float = 10.0
    gripper_tol: float = 0.005
