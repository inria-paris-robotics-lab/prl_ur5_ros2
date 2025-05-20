# prl_moveit_exemple

The `prl_moveit_exemple` package provides example files for interfacing your project with the PRL UR5 setup via MoveIt by using MoveItPy.

**`MoveItPy`** is a Python-based interface for MoveIt, enabling easier scripting and integration for robotic applications. It provides a high-level API to interact with MoveIt, making it simpler to plan and execute motions, configure robotic setups, and integrate custom behaviors. 

With `MoveItPy`, users can:
- Create and execute motion plans programmatically.
- Access and modify robot states and configurations.

This package is part of the PRL (Paris Robotics Lab) ecosystem and is designed to facilitate the use of the UR5 robot in simulation and real-world scenarios.

## Content

### Launch Files
- **`moveit_exemple.launch.py`**  
    This launch file demonstrates how to set up and run MoveIt configurations for the UR5 robot. Refer to the built-in documentation in the launch file headers for more details.

### Configuration Files
- **`planner.yaml`**  
    This file allows you to configure the planner for arm path planning using MoveIt. It includes parameters for optimizing motion planning and execution.

## Usage

To use this package, ensure that your environment is set up with the required dependencies, including MoveIt and MoveItPy. Follow these steps:

Launch the example using the provided launch file:
```bash
ros2 launch prl_moveit_exemple moveit_exemple.launch.py
```

Modify the `planner.yaml` file to customize the motion planning parameters as needed.
## Single-Pipeline vs Multi-Pipeline Motion Planning

### Single-Pipeline Motion Planning
---
In single-pipeline motion planning, a single algorithm computes a trajectory for the robot. This approach is straightforward and suitable for tasks with well-defined constraints and environments.

#### Example Pipeline: OMPL RRTConnect
The **OMPL RRTConnect** pipeline is a commonly used planner for single-pipeline motion planning. It is efficient for high-dimensional spaces and provides fast solutions for general-purpose tasks.

#### Workflow:
1. **Set Start and Goal States**:
    - Define the start state of the robotic arm as its current state.
    - Specify the goal state using a predefined pose or configuration.

2. **Configure the Planner**:
    - Set the planner to `ompl_rrtc` in the `planner.yaml` file or programmatically via MoveItPy.

3. **Plan and Execute**:
    - Use the `plan_and_execute` function to compute a trajectory and execute it.
    - If planning fails, log the error and adjust the parameters or goal state as needed.

#### Example Scenario:
- **Scenario**: The robotic arm needs to move from its home position to a predefined "pick" position in an open workspace.
  - Use the `ompl_rrtc` pipeline for its speed and efficiency.

This approach is ideal for tasks where a single planning algorithm is sufficient to meet the requirements. For more complex scenarios, consider using multi-pipeline motion planning as described below.

### Multi-Pipeline Motion Planning
---
In robotic systems, motion planning involves finding a valid trajectory for a robot to move from its current state to a desired goal state. Different planning algorithms (or "pipelines") have unique strengths and weaknesses depending on the task, environment, and constraints.

#### Example Pipelines:
- **OMPL RRTConnect (`ompl_rrtc`)**: Good for fast, general-purpose planning in high-dimensional spaces.
- **Pilz Linear Planner (`pilz_lin`)**: Ideal for simple, linear motions.
- **CHOMP Planner (`chomp_planner`)**: Optimized for smooth trajectories in cluttered environments.

#### How Multi-Pipeline Planning Works:
This approach leverages multiple planning pipelines simultaneously to improve the chances of finding a valid trajectory quickly. The first pipeline that produces a valid trajectory is selected for execution. This is particularly useful in scenarios where:
- **Time is critical**: Minimize planning time by trying multiple algorithms in parallel.
- **Task complexity varies**: Different pipelines excel in different types of motion (e.g., linear vs. complex paths).
- **Fallback mechanisms are needed**: If one pipeline fails, others can still succeed.

#### Workflow:
1. **Set Start and Goal States**:
    - The start state of the robotic arm is set to its current state.
    - The goal state is defined using a configuration name (e.g., `"work"`), which corresponds to a predefined pose.

2. **Initialize Multi-Pipeline Parameters**:
    - Specify the robot (e.g., `ur`) and the list of planning pipelines to use (e.g., `["ompl_rrtc", "pilz_lin", "chomp_planner"]`).

3. **Plan and Execute**:
    - For each arm, the `plan_and_execute` function:
        - Tries all specified pipelines and selects the first valid trajectory.
        - Executes the trajectory using the robot's controllers.
        - Logs an error if planning fails.

#### Example Scenarios:
- **Scenario 1**: The arm needs to move quickly between two points in an open space. Use `ompl_rrtc`.
- **Scenario 2**: The arm needs to perform a precise, linear motion to insert a part. Use `pilz_lin`.
- **Scenario 3**: The arm needs to navigate around obstacles in a cluttered workspace. Use `chomp_planner`.

By using multiple pipelines, the system dynamically adapts to these scenarios without requiring manual intervention or reconfiguration.
---
Refer to the official MoveItPy documentation and the built-in documentation for more advanced usage and integration tips.
