# ManyMove Project - Dependency Installation Plan (ROS2 Humble)

This plan outlines the steps to install the necessary dependencies for the `manymove` project in your GitHub Codespaces environment (`/workspaces`).

**Workspace Setup:**

* Choose a workspace directory within your Codespace. We'll use `/workspaces/dev_ws` as an example.

    ```bash
    mkdir -p /workspaces/dev_ws/src
    cd /workspaces/dev_ws
    ```

**1. Install ROS2 Humble:**

* Your Codespace environment (`osrf/ros:humble-desktop` image) should already have ROS2 Humble pre-installed.
* If you need to install it manually or verify the installation, follow the official ROS2 Humble installation guide: [https://docs.ros.org/en/ros2_documentation/humble/Installation.html](https://docs.ros.org/en/ros2_documentation/humble/Installation.html)
* Ensure the ROS2 environment is sourced:

    ```bash
    source /opt/ros/humble/setup.bash
    ```

    *(You might want to add this to your `.bashrc`)*

**2. Install MoveIt2:**

* Install MoveIt2 binaries for ROS2 Humble:

    ```bash
    sudo apt update && sudo apt install ros-humble-moveit -y
    ```

* For more details, refer to the official MoveIt2 installation guide: [https://moveit.ros.org/install-moveit2/binary/](https://moveit.ros.org/install-moveit2/binary/)

**3. Install Gazebo and Gazebo ROS Packages:**

* Install Gazebo (Classic):

    ```bash
    sudo apt update && sudo apt install gazebo -y
    ```

  * Reference: [https://classic.gazebosim.org/tutorials?tut=install_ubuntu](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
* Install Gazebo ROS packages:

    ```bash
    sudo apt update && sudo apt install ros-humble-gazebo-ros-pkgs -y
    ```

  * Reference: [http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)

**4. Install xarm_ros2:**

* Navigate to your workspace's `src` directory:

    ```bash
    cd /workspaces/dev_ws/src
    ```

* Clone the `xarm_ros2` repository (humble branch) recursively:

    ```bash
    git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b humble
    ```

* Install dependencies using `rosdep`:

    ```bash
    cd /workspaces/dev_ws
    sudo apt update # Recommended before rosdep
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

**5. Install moveit2_tutorials:**

* Navigate to your workspace's `src` directory:

    ```bash
    cd /workspaces/dev_ws/src
    ```

* Clone the `moveit2_tutorials` repository:

    ```bash
    git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble
    ```

* Import additional dependencies using `vcs`:

    ```bash
    vcs import < moveit2_tutorials/moveit2_tutorials.repos
    ```

* Install dependencies using `rosdep` (this step might reinstall some already installed packages, which is okay):

    ```bash
    cd /workspaces/dev_ws
    sudo apt update # Recommended before rosdep
    rosdep update
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

**6. Build Workspace:**

* Navigate back to the root of your workspace:

    ```bash
    cd /workspaces/dev_ws
    ```

* Build all packages:

    ```bash
    colcon build --symlink-install
    ```

    *(Using `--symlink-install` can speed up development cycles)*

**7. Source Workspace:**

* After building, source your workspace's setup file:

    ```bash
    source /workspaces/dev_ws/install/setup.bash
    ```

    *(You might want to add this to your `.bashrc` after the ROS2 setup source line)*

**Notes on Other Dependencies:**

* **BehaviorTree.CPP v3.8:** This should be installed automatically as `ros-humble-behaviortree-cpp-v3` by the `rosdep install` commands run in steps 4 and 5.
* **py_trees_ros:** This should also be installed automatically by the `rosdep install` commands.

After completing these steps, all dependencies required by `manymove` (as listed in `knowledge_stack.md` and the respective READMEs) should be installed and your workspace should be ready for the `manymove` installation itself.
