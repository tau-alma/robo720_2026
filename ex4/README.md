# Exercise 4: Robot motion control

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

## Publishing goals

You can publish either goal poses for the high-level controllers, or joint trajectories to low-level controller using the trajectory publisher from exercise 3. To publish goal poses, run:

```bash
ros2 run goal_publishers goal_pose_publisher.py
```

There are preset goal poses mapped to keys 0-8. When the node is running, press any of those keys and a new goal pose is published. You can define your own poses too in goal_pose_publisher.py.

## Launch setups

To launch gravity compensation + PD controller, run:

```bash
ros2 launch ex4 gc_pd.launch.py
```

This controller takes a goal pose as an input.

To launch the low-level joint velocity controller and trajectory publisher, run:

```bash
ros2 launch ex4 joint_velocity_controller.launch.py
```

To launch low-level controller and both high-level controllers, run:

```bash
ros2 launch ex4 kinematic_controller.launch.py
```

Once again, these controllers take goal poses as input.

## Switching between high-level controllers

When the kinematic controller setup is running, you can switch between the high-level controllers during run-time. By default, the joint space controller is active and task space controller is inactive.

To switch from joint space to task space, open new terminal and run:

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['task_space_kinematic_controller'], deactivate_controllers: ['joint_space_kinematic_controller'], strictness: 1}"
```

To switch back from task space to joint space, run:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['joint_space_kinematic_controller'], deactivate_controllers: ['task_space_kinematic_controller'], strictness: 1}"
```

SwitchController service atomically activates and deactivates the controllers, meaning that there is no moment where both of the controllers would be inactive or active. Immediately when the first one is deactivated, the second one activates.
