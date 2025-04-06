# ManyMove Project Knowledge Stack (ROS2 Humble)

This document serves as a knowledge base for the `manymove` ROS2 project deployment, based on the repository's README.

## Project Overview

* The `manymove` project aims to simplify the transition to ROS2 for roboticists familiar with classic frameworks.
* It provides a generalized framework for building robotic manipulator control logic using ROS 2 and MoveIt 2.
* The project was initially developed around Ufactory Lite6 and UF850 cobots but can be extended to other robots like Franka Emika Panda.

## Dependencies

* ROS2 Humble
* MoveIt2
* xarm_ros2 (Humble branch)
* BehaviorTree.CPP v3.8
* py_trees_ros

## Installation Instructions

1. **Define workspace directory:** Choose a workspace directory within your Codespace, e.g., `/workspaces/dev_ws`. **Note:** Your Codespace home directory is `/workspaces`. The original README suggests `~/dev_ws`, but we'll use `/workspaces/dev_ws` for consistency with your environment.
2. **Clone the repository:** From `/workspaces/dev_ws/src`:

    ```bash
    # Ensure you are in the correct directory first: cd /workspaces/dev_ws/src
    git clone --branch=humble https://github.com/pastoriomarco/manymove.git
    ```

3. **Install dependencies:** From `/workspaces/dev_ws`:

    ```bash
    # Ensure you are in the correct directory first: cd /workspaces/dev_ws
    rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

4. **Copy auxiliary files:** (Run these commands from `/workspaces/dev_ws`)
    * Create the `other` folder in `xarm_description/meshes`:

        ```bash
        # Ensure you are in the correct directory first: cd /workspaces/dev_ws
        mkdir -p ./src/xarm_ros2/xarm_description/meshes/other
        ```

    * Copy pneumatic gripper's mesh:

        ```bash
        # Ensure you are in the correct directory first: cd /workspaces/dev_ws
        cp ./src/manymove/manymove_object_manager/meshes/custom_end_tools/* ./src/xarm_ros2/xarm_description/meshes/other/
        ```

    * Copy the user param file in `xarm_api/config`:

        ```bash
        # Ensure you are in the correct directory first: cd /workspaces/dev_ws
        cp ./src/manymove/manymove_planner/config/xarm_user_params.yaml ./src/xarm_ros2/xarm_api/config/
        ```

5. **Build the packages:** From `/workspaces/dev_ws`:

    ```bash
    # Ensure you are in the correct directory first: cd /workspaces/dev_ws
    colcon build
    ```

6. **Source the environment:** From `/workspaces/dev_ws`:

    ```bash
    source ./install/setup.bash
    ```

## Project Structure

1. **`manymove_msgs`:** Custom action definitions and messages.
2. **`manymove_planner`:** Motion planning logic using MoveIt 2 and ROS 2 action servers.
3. **`manymove_object_manager`:** Manages collision objects in the planning scene.
4. **`manymove_signals`:** Handles digital I/O signals and checks the robot’s state.
5. **`manymove_cpp_trees`:** C++ BehaviorTree.CPP framework for composing robotic behaviors.
6. **`manymove_py_trees`:** Python-based alternative using py_trees for building control flows.
7. **`manymove_hmi`:** Human-Machine Interface (HMI) for issuing commands and monitoring status.

## Examples

* **Lite 6 manipulator:**
  * MoveItCPP and BehaviorTree.CPP:

```bash
        ros2 launch manymove_planner lite_moveitcpp_fake_cpp_trees.launch.py
        ```

  * MoveGroupInterface and BehaviorTree.CPP:

```bash
        ros2 launch manymove_planner lite_movegroup_fake_cpp_trees.launch.py
        ```

  * MoveGroupInterface and py_trees (minimal):

        ```bash
        ros2 launch manymove_planner lite_movegroup_fake_py_trees.launch.py
        ```

* **Dual robot (Lite 6 + UF850):**
    ```bash
    ros2 launch manymove_planner dual_moveitcpp_fake_cpp_trees.launch.py
    ```
  - Note: to run the app example with the robots in custom positions you'll have to use the fork of xarm_ros2 modified to handle this kind of scenario. Follow the instructions in the above link from the original repository, but change the instruction on point 4.2 with:
    ```bash
    git clone https://github.com/pastoriomarco/xarm_ros2.git --recursive -b $ROS_DISTRO
    ```

* **Panda Manipulator:** (requires the installation of [moveit2_tutorials](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html) )
  * MoveItCPP with BehaviorTree.CPP:

        ```bash
        ros2 launch manymove_planner panda_moveitcpp_fake_cpp_trees.launch.py
        ```

  * MoveGroupInterface with BehaviorTree.CPP:

        ```bash
        ros2 launch manymove_planner panda_movegroup_fake_cpp_trees.launch.py
        ```

  * MoveGroupInterface with py_trees:

        ```bash
        ros2 launch manymove_planner panda_movegroup_fake_py_trees.launch.py
        ```

## Architecture Flow

1. `manymove_planner` and `manymove_object_manager` run action servers.
2. `manymove_signals` handles the digital I/O action server.
3. A behavior tree orchestrates these actions:
    * Check robot state.
    * Add/remove objects.
    * Plan and execute motions.
    * Send signals to grippers or sensors.
4. `manymove_hmi` provides a GUI or service-based interface for control and visualization.

## Credits

* BehaviorTree.CPP v3.8
* py_trees_ros
* MoveIt 2 community
* xarm_ros2 on GitHub

## Notes & Disclaimer

* Experimental project under development.
* No safety features included. Safety **MUST** be implemented using the internal safety system of the robot's controller and/or an appropriate safety controller.
* Feedback is welcome.
