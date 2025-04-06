# ManyMove Project Examples (ROS2 Humble)

This document outlines the example launch files provided in the `manymove` project, based on the repository's README.

**Prerequisites:**

*Ensure you have completed the installation steps outlined in `knowledge_stack.md` and `dependancies_plan.md`.
*Source your workspace environment before running any examples:

```bash
    source /workspaces/dev_ws/install/setup.bash
    ```

## Lite 6 Manipulator Examples

These examples demonstrate control of the Ufactory Lite 6 manipulator in a simulated environment (`fake`).

1.  **MoveItCPP with BehaviorTree.CPP:**
    *   Uses the MoveItCpp interface for planning and execution.
    *   Uses BehaviorTree.CPP for task orchestration.
    *   **Command:**
        ```bash
        ros2 launch manymove_planner lite_moveitcpp_fake_cpp_trees.launch.py
        ```

2.  **MoveGroupInterface with BehaviorTree.CPP:**
    *   Uses the MoveGroupInterface for planning and execution.
    *   Uses BehaviorTree.CPP for task orchestration.
    *   **Command:**
        ```bash
        ros2 launch manymove_planner lite_movegroup_fake_cpp_trees.launch.py
        ```

3.  **MoveGroupInterface with py_trees (minimal):**
    *   Uses the MoveGroupInterface for planning and execution.
    *   Uses py_trees (Python-based behavior trees) for task orchestration.
    *   **Command:**
        ```bash
        ros2 launch manymove_planner lite_movegroup_fake_py_trees.launch.py
        ```

## Dual Robot (Lite 6 + UF850) Example

This example demonstrates control of two robots (Lite 6 and UF850) simultaneously.

*   Uses MoveItCPP and BehaviorTree.CPP.
*   **Note:** Running this example with robots in custom positions requires using a specific fork of `xarm_ros2`. If you haven't cloned this specific fork during the dependency installation, you might need to replace the standard `xarm_ros2` with the one from `https://github.com/pastoriomarco/xarm_ros2.git` (humble branch) and rebuild your workspace.
*   **Command:**
    ```bash
    ros2 launch manymove_planner dual_moveitcpp_fake_cpp_trees.launch.py
    ```

## Panda Manipulator Examples

These examples demonstrate control of the Franka Emika Panda manipulator.

**Prerequisites:**

*   Requires the `moveit2_tutorials` package to be installed and sourced. (This was covered in `dependancies_plan.md`).

**Option 1: Standalone Launchers**

These launchers start the necessary MoveIt components along with the `manymove` nodes.

1.  **MoveItCPP with BehaviorTree.CPP:**
    *   **Command:**
        ```bash
        ros2 launch manymove_planner panda_moveitcpp_fake_cpp_trees.launch.py
        ```

2.  **MoveGroupInterface with BehaviorTree.CPP:**
    *   **Command:**
        ```bash
        ros2 launch manymove_planner panda_movegroup_fake_cpp_trees.launch.py
        ```

3.  **MoveGroupInterface with py_trees:**
    *   **Command:**
        ```bash
        ros2 launch manymove_planner panda_movegroup_fake_py_trees.launch.py
        ```

**Option 2: Separate Launchers (Two Terminals)**

This approach uses the standard `moveit2_tutorials` demo launch file and starts the `manymove` components separately.

1.  **Terminal 1:** Launch the standard Panda demo (ensure `moveit2_tutorials` is sourced).
    ```bash
    ros2 launch moveit2_tutorials demo.launch.py
    ```

2.  **Terminal 2:** Launch the `manymove` behavior tree client (ensure your `dev_ws` is sourced).
    *   **Alternative A (MoveGroupInterface + BehaviorTree.CPP):**
        ```bash
        ros2 launch manymove_planner panda_fake_cpp_trees.launch.py
        ```
    *   **Alternative B (MoveGroupInterface + py_trees):**
        ```bash
        ros2 launch manymove_planner panda_fake_py_trees.launch.py
        ```

These examples provide various ways to interact with the `manymove` framework using different robots, planning interfaces (MoveItCpp vs. MoveGroupInterface), and behavior tree implementations (C++ vs. Python).
