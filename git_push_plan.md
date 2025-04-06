# Git Plan: Pushing dev_ws/src to 'grasp' Branch

This plan outlines the steps to stage, commit, and push the contents of your `/workspaces/dev_ws/src` directory to the `grasp` branch in your GitHub repository (`omar55221/desktop_devcontainer`).

**Prerequisites:**

* You have already created the `grasp` branch locally in your Codespace.
* You are currently on the `grasp` branch (`git checkout grasp`).
* Your Codespace terminal is open, likely in the `/workspaces/dev_ws` or `/workspaces/dev_ws/src` directory.

**Steps:**

1. **Navigate to the correct directory:**
    * Ensure you are in the root directory of your Git repository within the Codespace. This is likely `/workspaces/desktop_devcontainer` based on the repository URL provided.
    * Run this command to be sure:

        ```bash
        cd /workspaces/desktop_devcontainer
        ```

2. **Check Git Status:**
    * Verify you are on the correct branch and see the changes you want to commit.
    * Command:

        ```bash
        git status
        ```

    * You should see the `dev_ws/src/` directory listed under "Untracked files" or "Changes not staged for commit".

3. **Stage the `src` directory:**
    * This command adds all files and subdirectories within `/workspaces/dev_ws/src` to the staging area. **Important:** Note that the path `dev_ws/src` is relative to the repository root (`/workspaces/desktop_devcontainer`).
    * Command:

        ```bash
        git add dev_ws/src
        ```

    * Run `git status` again to confirm the files under `dev_ws/src/` are now staged (listed under "Changes to be committed").

4. **Commit the changes:**
    * Commit the staged changes with a descriptive message.
    * Command:

        ```bash
        git commit -m "Add ROS workspace source files for manymove project"
        ```

    * *(Adjust the commit message as needed)*

5. **Push the changes to the remote `grasp` branch:**
    * This command pushes your local `grasp` branch commits to the remote repository (`origin`). The `-u` flag sets the upstream branch for future pushes/pulls.
    * Command:

        ```bash
        git push -u origin grasp
        ```

**Verification:**

* After pushing, you can check your repository on GitHub (`https://github.com/omar55221/desktop_devcontainer/tree/grasp`) to confirm that the `dev_ws/src` directory and its contents have been successfully pushed to the `grasp` branch.

**Re-installation Notes (After Pushing and Rebuilding Container):**

* **System Dependencies (ROS, MoveIt, Gazebo, etc.):** These are typically defined in your `.devcontainer/devcontainer.json` or Dockerfile. If these haven't changed, they should be automatically installed when the Codespace rebuilds based on the `grasp` branch configuration. You likely won't need to manually reinstall these using `apt`.
* **ROS Dependencies (from source):** Dependencies cloned into your `src` directory (like `xarm_ros2`, `moveit2_tutorials`, `manymove`) are now part of your Git repository. When the Codespace rebuilds, it will pull these source files.
* You **will** need to run `rosdep install` again after the container rebuilds to ensure any *binary* dependencies required by these source packages are installed:

 ```bash
        cd /workspaces/dev_ws
        sudo apt update && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
        ```

  * You **will** need to rebuild your workspace using `colcon build` after the container rebuilds and `rosdep` is run:

        ```bash
        cd /workspaces/dev_ws
        colcon build --symlink-install
        ```

* **Auxiliary Files:** The files copied in Step 4 of the `knowledge_stack.md` installation (mesh files, config file) are *not* typically part of the Git repository unless you explicitly added them. If your `.devcontainer` setup doesn't automatically copy these files upon rebuild, you might need to re-run those `cp` commands manually. Consider adding these copy steps to your `.devcontainer` setup scripts (e.g., `postCreateCommand` in `devcontainer.json`) for automation.
