# Exercise 2: Gazebo + RViz + MoveIt

To get the latest changes in the repo to your local directory, navigate to the course directory in your terminal, run `git pull`, and resolve all possible merge conflicts (you may save your own edits, but in the future it is possible that the templates will not work with your code).

Once inside container again, navigate to `ros2_ws` and build and source the packages:

```bash
cd ~/ros2_ws
colcon build --parallel-workers $(($(nproc)/2))
source install/setup.bash
```

## Launch setups

To launch the pure Gazebo simulation without any controllers, run:

```bash
ros2 launch ex2 gazebo.launch.py
```

To launch the MoveIt integration (this requires creating franka_gripper_moveit package with MoveIt setup assistant), run:

```bash
ros2 launch ex2 moveit.launch.py
```
