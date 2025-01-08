# Project 1: Control of a Mobile Robot with ROS

## Overview

This project involves implementing a basic obstacle avoidance algorithm for mobile robots using ROS and the Stage simulator. The project includes two missions:

1. **Mission 1**: Navigate to a given target while avoiding obstacles using two algorithms:
   - Simple Avoidance Algorithm
   - Potential Fields Algorithm
2. **Mission 2**: A "Follow the Leader" scenario where one robot follows another while avoiding obstacles.

This project tests your ability to develop control strategies and integrate ROS-based nodes to manage robot behavior.

---

## Getting Started

### Prerequisites

Ensure the following are installed:
- ROS (Robot Operating System)
- Stage simulator
- Catkin workspace (`~/catkin_ws`)

---

## Step-by-Step Guide

### 0. Creating a New Package

1. **Change to the Catkin Workspace Directory**
   ```bash
   cd ~/catkin_ws/src
2. **Create a New Package**
   ```bash
   catkin_create_pkg project1 std_msgs roscpp tf geometry_msgs genmsg nav_msgs
3. **Pull the Project Repository**
   ```bash
    cd ~/catkin_ws/src/project1
    git clone https://github.com/Joaquinecc/ros_project1.git .
4. **Compile the Workspace**
   ```bash
   cd ~/catkin_ws/src
   catkin_make
5. **Initialize the Environment**
   ```bash
   source devel/setup.bash
6. **Run the Launch File**
    Ensure roscore is running, then launch the file:
   ```bash
    roslaunch project1 Prac1_Mission.launch

## Implementation Details

### Files and Programs

1. **`moveRobot.cpp`**
   - Implements robot movement, obstacle avoidance, and "Follow the Leader" behavior.
   - Parameters:
     - **`ID_ROBOT`**: Integer, robot ID.
     - **`ALGOR`**: Integer, algorithm to execute (1: Simple Avoidance, 2: Potential Fields).
     - **`CRIT_DIST`**: Double, maximum distance to avoid obstacles (0.8 ≤ CRIT_DIST ≤ 1.2).
     - **`D_OBJ`**: Double, distance threshold to consider the target reached (0.2 ≤ D_OBJ ≤ 0.6).
     - **`V_MAX_DES`**: Double, maximum linear velocity (0.8 ≤ V_MAX_DES ≤ 1.2).
     - **`V_MAX_ROT`**: Double, maximum rotational velocity (0.8 ≤ V_MAX_ROT ≤ 1.1).
     - **`K_ROT_MIN`**, **`K_ROT_MAX`**: Double, proportional controller constants (0.05 ≤ K_ROT_MIN ≤ 0.15, 0.2 ≤ K_ROT_MAX ≤ 0.3).
     - **`ORI_ERROR`**: Double, allowed orientation error (0.3 ≤ ORI_ERROR ≤ 0.6).
     - **`T_AVOID_OBS`**: Double, time required to avoid an obstacle (1.0 ≤ T_AVOID_OBS ≤ 3.0).
     - **`DIST_LEADER`**: Double, distance between follower and leader (1.5 ≤ DIST_LEADER ≤ 2.5).
     - **`ROBOT_ROL`**: Integer, role of the robot (0: Leader, 1: Follower).
     - **`ID_LEADER`**: Integer, ID of the leader robot if the robot is a follower.
     - **`W_1`**, **`W_2`**: Double, weights for behaviors (1 ≤ W_1 ≤ 2, 2.5 ≤ W_2 ≤ 4).
     - **`T_WAIT`**: Integer, wait time between decisions for the Potential Fields algorithm (100 ≤ T_WAIT ≤ 250 milliseconds).

2. **`talkerGoals.cpp`**
   - Publishes the target points `(X_GOAL, Y_GOAL)` periodically to a topic named `myGoals`.
   - Parameters:
     - **`X_GOAL`**: Double, X-coordinate of the target point.
     - **`Y_GOAL`**: Double, Y-coordinate of the target point.
     - **`T_GOAL`**: Double, time interval between updates (3 ≤ T_GOAL ≤ 7 seconds).

3. **Launch File**
   - File: `Prac1_Mission.launch`
   - Located in the `launch/` directory.
   - Launches the `moveRobot` and `talkerGoals` nodes.
   - Uses the world file `mundo.world` located in the `worlds/` directory.


## 1.3 Submitting Code

### 1.3.1 Pre-commit

This code base uses pre-commit git-hooks in order to check the code before it is committed to the repo. After
you have cloned the repo, install pre-commit package:

```shell
pip install pre-commit
```

and then install the pre-commit checks:

```shell
pre-commit install
```

Pre-commit needs to be installed only once after cloning the repo. Pre-commit is run automatically every time you commit code,
but you can run it manually as follows:

```shell
pre-commit run --all-files
```

When you run the above command, you will get a list of things that need to be fixed before committing.

### 1.3.2 Branching

Always create a new branch for anything that you want to merge to the main-branch, and then create a pull request (PR)
in GitHub when you are ready to submit the code. Before submitting the code, please make sure that it's PEP8 compliant.

All the code needs to be commented before submitting it, so that the input parameters and return values of functions are
clearly commented. Happy coding!!
