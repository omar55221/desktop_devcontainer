# ManyMove Project - Debugging Plan (ROS2 Humble)

This document outlines steps to debug common issues encountered while running the `manymove` examples, particularly the pick-and-place behavior tree examples.

## Common Error: Trajectory Execution Failure / Collision / Attached Body Not Found

This error often occurs during the `MoveManipulatorAction` and might be indicated by messages like:

*`[ERROR] [action_server_node-X]: [MoveManipulator] Execution failed => Trajectory execution failed or collision mid-run`
*`[ERROR] [moveit_robot_state.robot_state]: Attached body '...' not found`
*`[WARN] [object_manager_node-X]: Object '...' is not attached to link 'any link'.`

**Troubleshooting Steps:**

1.**Verify Object Attachment Logic:**
    ***Behavior Tree Order:** Ensure the `AttachDetachObjectAction` is called *before* any `MoveManipulatorAction` that requires the object to be attached. Check the sequence of nodes in your Behavior Tree XML or Python script.
    *   **Action Parameters:** Double-check the parameters passed to `AttachDetachObjectAction`:
        *`object_id`: Must match the ID of the object you intend to attach (e.g., `graspable_mesh`).
        *   `link_name`: Must be the correct end-effector link name (e.g., `link_tcp` for Lite6, `panda_link8` for Panda).
        *`attach`: Should be `true` for attaching and `false` for detaching.
    *   **Touch Links:** Verify the `touch_links` parameter includes the correct gripper links that are allowed to be in collision with the object during the attached phase (e.g., `[panda_leftfinger, panda_rightfinger, panda_hand]` for Panda).

2.**Inspect Planning Scene with RViz:**
    *Launch RViz alongside your example.
    * Add the "PlanningScene" display type.
    *Set the "Planning Scene Topic" to `/monitored_planning_scene`.
    * Step through the behavior tree execution (if possible, or add delays).
    * Observe the "Scene Objects" and "Attached Objects" in the RViz display list. Verify that the target object (`graspable_mesh`) appears under "Attached Objects" *after* the attach action and *before* the move action that fails. Check if it's attached to the correct link.

3.**Check Collision Checking:**
    ***Temporarily Disable:** As a test, you can try disabling collision checking entirely within the `MoveManipulatorAction` or globally in MoveIt configuration to see if the trajectory executes successfully. **This is for debugging only and should not be done on a real robot.**
    * **Allowed Collision Matrix (ACM):** If disabling collision checking works, the issue is likely related to collisions. You need to configure the ACM correctly. Ensure collisions are allowed between the `touch_links` (gripper fingers) and the object (`graspable_mesh`) *when the object is attached*. This is typically configured in the SRDF file for the robot.

4.**Analyze Trajectory Validity:**
    *The error might genuinely be an invalid trajectory planned by MoveIt (e.g., self-collision, environment collision, unreachable pose).
    * Use RViz to visualize the planned trajectory (add the "MotionPlanning" display). Check if the planned path looks valid and collision-free.
    * Simplify the goal pose or the environment to see if planning succeeds.

5.**Check Node Logs:**
    *Increase the logging verbosity for relevant nodes (`action_server_node`, `object_manager_node`, `ros2_control_node`, `bt_client`) to get more detailed information. You can do this by adding launch arguments like `log_level:=debug`.
    * Look for more specific error messages around the time of the failure.

6.**Verify Controller Status:**
    * Ensure the relevant controllers (`lite6_traj_controller`, `panda_arm_controller`, etc.) are loaded and active. Use `ros2 control list_controllers`.

7.**Check TF Tree:**
    * Ensure the TF tree is correct and all necessary transforms are being published. Use `ros2 run tf2_tools view_frames` or RViz's TF display. Pay attention to the transform between the planning frame (`world` or `panda_link0`) and the end-effector link.

**General Debugging Tips:**

* **Isolate the Problem:** Try running simpler sequences in the behavior tree to pinpoint which action is failing.
* **Check Topics:** Use `ros2 topic echo` to monitor relevant topics like `/joint_states`, `/planning_scene`, `/attached_collision_object`.
* **Check Action Servers:** Use `ros2 action list` and `ros2 action info` to verify that the necessary action servers are running and available.
* **Source Environments:** Always ensure you have sourced both the base ROS 2 environment (`/opt/ros/humble/setup.bash`) and your workspace environment (`/workspaces/dev_ws/install/setup.bash`) correctly in every terminal you use.
