#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
import numpy as np

# A simple function to plan and execute a motion

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        # multiple pipelines are used to plan the trajectory
        # the planning component will use the first pipeline that returns a valid trajectory
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        # single pipeline is used to plan the trajectory
        # the planning component will use the pipeline specified in the parameters
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        # no pipeline is specified, the planning component will use the default pipeline set in the srdf
        plan_result = planning_component.plan()

    # execute the plan if it is valid trajectory
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.right_pose_goal")

    # instantiate MoveItPy instance and get planning component
    ur = MoveItPy(node_name="moveit_py")
    logger.info("Waiting for MoveIt to receive robot state...")
    time.sleep(5.0) # Pause de 5 secondes, à ajuster si nécessaire
    left_arm = ur.get_planning_component("left_arm")
    right_arm = ur.get_planning_component("right_arm")
    left_gripper = ur.get_planning_component("left_gripper")
    right_gripper = ur.get_planning_component("right_gripper")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    # left__arm.set_start_state(configuration_name="current") this allow to set the start state to a predefined state
    left_arm.set_start_state_to_current_state()
    # right_arm.set_start_state_to_current_state()

    # set pose goal using predefined state (prefined in the moveit package)
    left_arm.set_goal_state(configuration_name="work")
    # right_arm.set_goal_state(configuration_name="work")

    # plan to goal
    plan_and_execute(ur, left_arm, logger, sleep_time=3.0)
    # plan_and_execute(ur, right_arm, logger, sleep_time=3.0)

    left_gripper.set_start_state_to_current_state()
    # right_gripper.set_start_state_to_current_state()

    left_gripper.set_goal_state(configuration_name="close")
    # right_gripper.set_goal_state(configuration_name="open")

    plan_and_execute(ur, left_gripper, logger, sleep_time=3.0)
    # plan_and_execute(ur, right_gripper, logger, sleep_time=3.0)


    left_gripper.set_start_state_to_current_state()
    # right_gripper.set_start_state_to_current_state()
    left_gripper.set_goal_state(configuration_name="open")
    # right_gripper.set_goal_state(configuration_name="closed")

    plan_and_execute(ur, left_gripper, logger, sleep_time=3.0)
    # plan_and_execute(ur, right_gripper, logger, sleep_time=3.0)

    left_arm.set_start_state_to_current_state()
    # right_arm.set_start_state_to_current_state()
    left_arm.set_goal_state(configuration_name="home")
    # right_arm.set_goal_state(configuration_name="home")

    plan_and_execute(ur, left_arm, logger, sleep_time=3.0)
    # plan_and_execute(ur, right_arm, logger, sleep_time=3.0)


if __name__ == "__main__":
    main()
