# Exercise 3: ROS2 Control + control architectures

To get the latest changes in the repo to your local directory, navigate to the course directory in your terminal, run `git pull`, and resolve all possible merge conflicts (you may save your own edits, but in the future it is possible that the templates will not work with your code). If you want to save your own code separately and just get the fresh template, navigate to your home directory and run:

```bash
git clone {repo https}
```

Once inside container again, navigate to `ros2_ws` and build and source the packages:

```bash
cd ~/ros2_ws
colcon build --parallel-workers $(($(nproc)/2))
source install/setup.bash
```

## Launch setups
To launch the Gazebo simulation, controller and trajectory publisher, run

```bash
ros2 launch ex3 robot.launch.py
```

## Switching parameters during run-time

To switch the controller to use feedforward control law, open a new terminal and run:

```bash
ros2 param set /joint_controller use_feedforward true
```

To go back to feedback control law, run:

```bash
ros2 param set /joint_controller use_feedforward false
```