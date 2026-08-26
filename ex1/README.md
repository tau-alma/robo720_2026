# Exercise 1: Eigen + KDL introduction

Only implement the functions in file `src/kinematics.cpp`. Do not touch anything else.

Once the function(s) have been implemented, you can run the program to check whether your solution is correct or not. To run the program, open a terminal inside the Docker container (VS Code or Ubuntu), and navigate to `~/ros2_ws/src/robo720_2026/ex1` directory:

```bash
cd ~/ros2_ws/src/robo720_2026/ex1
```

Next, build the program:

```bash
mkdir build/
cd build
cmake ..
make
```

If the program was built without any errors, run it inside `build/` directory with:

```bash
./ex1
```

Command `cmake ..` is only required the first time when building the program. After making changes to the code, it is enough to only run `make` and `./ex1`.
