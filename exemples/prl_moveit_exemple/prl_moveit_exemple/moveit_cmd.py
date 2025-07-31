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
    PlanRequestParameters,
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
    left_arm = ur.get_planning_component("left_arm")
    right_arm = ur.get_planning_component("right_arm")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    # left__arm.set_start_state(configuration_name="current") this allow to set the start state to a predefined state
    left_arm.set_start_state_to_current_state()
    right_arm.set_start_state_to_current_state()

    # set pose goal using predefined state (prefined in the moveit package)
    left_arm.set_goal_state(configuration_name="work")
    right_arm.set_goal_state(configuration_name="work")

    plan_params = PlanRequestParameters(ur, "left_arm")
    plan_params.planning_pipeline = "ompl"
    plan_params.planner_id = "RRTConnect"
    plan_params.max_velocity_scaling_factor = 0.25
    plan_params.max_acceleration_scaling_factor = 0.25

    # plan to goal
    plan_and_execute(ur, left_arm, logger,single_plan_parameters=plan_params, sleep_time=3.0)
    plan_and_execute(ur, right_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    # instantiate a RobotState instance using the current robot model
    # robotstate is a class that allows to set the state of the robot
    robot_model = ur.get_robot_model()
    robot_state = RobotState(robot_model)

    # Define the joint values for the robot state (in the same order as the joint names in the robot model)
    joint_values = np.array([
        0.0,        # shoulder_pan_joint
        -1.57,      # shoulder_lift_joint
        0.0,       # elbow_joint
        -1.57,      # wrist_1_joint
        0.0,      # wrist_2_joint
        0.0         # wrist_3_joint
    ])

    # Set the joint values for the left and right arms
    robot_state.set_joint_group_positions("left_arm", joint_values)
    robot_state.set_joint_group_positions("right_arm", joint_values)

    # Set goal state using RobotState object
    left_arm.set_goal_state(robot_state=robot_state)
    right_arm.set_goal_state(robot_state=robot_state)

    # pan to goal
    plan_and_execute(ur, left_arm, logger, sleep_time=3.0)
    plan_and_execute(ur, right_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    right_arm.set_start_state_to_current_state()
    left_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    right_pose_goal = PoseStamped()
    # set the reference frame for the pose
    right_pose_goal.header.frame_id = "base_link" 
    # set the pose orientation and position
    # the orientation is set using quaternions
    right_pose_goal.pose.orientation.w = 0.0
    right_pose_goal.pose.orientation.x = 1.0
    right_pose_goal.pose.orientation.y = 0.0
    right_pose_goal.pose.orientation.z = 0.0
    # the position is set using x, y, z coordinates
    # the position is set relative to the reference frame
    right_pose_goal.pose.position.x = 0.0
    right_pose_goal.pose.position.y = -0.191
    right_pose_goal.pose.position.z = 1.001
    # set the goal state using the pose goal
    # the pose_link is the link to which the pose is relative to
    right_arm.set_goal_state(pose_stamped_msg=right_pose_goal, pose_link="right_tool0")

    # Same for the left arm
    left_pose_goal = PoseStamped()
    left_pose_goal.header.frame_id = "base_link"
    left_pose_goal.pose.orientation.w = 0.0
    left_pose_goal.pose.orientation.x = 0.5
    left_pose_goal.pose.orientation.y = 0.0
    left_pose_goal.pose.orientation.z = 0.0
    left_pose_goal.pose.position.x = 0.0    
    left_pose_goal.pose.position.y = 0.191
    left_pose_goal.pose.position.z = 1.001
    left_arm.set_goal_state(pose_stamped_msg=left_pose_goal, pose_link="left_tool0")

    # plan to goal
    plan_and_execute(ur, right_arm, logger, sleep_time=3.0)
    plan_and_execute(ur, left_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    # set plan start state to current state
    left_arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    # set the joint values which you want to constrain for the goal state
    # you just need to set the values of the joints you want to constrain however the joint need to be in the left or right group
    joint_values = { 
        "left_shoulder_pan_joint": 0.0,
        "left_shoulder_lift_joint": -1.57,
        "left_elbow_joint": 0.0,
        "left_wrist_1_joint": -1.57,
        "left_wrist_2_joint": 0.0,
        "left_wrist_3_joint": 0.0,
        "right_shoulder_pan_joint": 0.0,
        "right_shoulder_lift_joint": -1.57,
        "right_elbow_joint": 0.0,
        "right_wrist_1_joint": -1.57,
        "right_wrist_2_joint": 0.0,
        "right_wrist_3_joint": 0.0,
    }
    # set the joint values for the robot state
    robot_state.joint_positions = joint_values
    # construct the joint constraint for the left and right arms by using the joint model group stored in the robot model
    # the joint model group is the group of joints that are used to control the left and right arms so when you get the joint value for a define joint model group
    # you need to have set the joint values of each joint in the group
    left_joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=ur.get_robot_model().get_joint_model_group("left_arm"),
    )
    right_joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=ur.get_robot_model().get_joint_model_group("right_arm"),
    )
    # set the goal state using the joint constraint
    left_arm.set_goal_state(motion_plan_constraints=[left_joint_constraint])
    right_arm.set_goal_state(motion_plan_constraints=[right_joint_constraint])

    # plan to goal
    plan_and_execute(ur, left_arm, logger, sleep_time=3.0)
    plan_and_execute(ur, right_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 5 - Planning with Multiple Pipelines simultaneously
    ###########################################################################

    # set plan start state to current state
    left_arm.set_start_state_to_current_state()
    right_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    left_arm.set_goal_state(configuration_name="work")
    right_arm.set_goal_state(configuration_name="work")

    # initialise multi-pipeline plan request parameters
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        ur, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    )

    # plan to goal with multiple pipelines
    # the planning component will use the first pipeline that returns a valid trajectory you can also use another criteria to select the pipeline
    plan_and_execute(
        ur,
        left_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
    )
    plan_and_execute(
        ur,
        right_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
    )

if __name__ == "__main__":
    main()
