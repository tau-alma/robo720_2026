# Exercise 6: Obstacle avoidance

To get the latest changes in the repo to your local directory, navigate to the course directory in your terminal, run `git pull`, and resolve all possible merge conflicts (you may save your own edits, but in the future it is possible that the templates will not work with your code). If you want to save your own code separately and just get the fresh template, navigate to your home directory and run:

```bash
git clone https://github.com/tau-alma/robo720_2026.git
```

Once inside container again, navigate to `ros2_ws` and build and source the packages:

```bash
cd ~/ros2_ws
colcon build --parallel-workers $(($(nproc)/2))
source install/setup.bash
```

## Launch setup

To launch the potential field velocity controller, run:

```bash
ros2 launch ex6 robot.launch.py
```

To launch the potential field effort controller, run:

```bash
ros2 launch ex6 robot.launch.py controller_name:="pf_effort_controller"
```

You can publish goal poses for the controller with goal_pose_publisher.py. Open a new terminal and run:

```bash
source install/setup.bash
ros2 run goal_publishers goal_pose_publisher.py
```
