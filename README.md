<!--
  README for the `spot_tennis_demo` package.
  Generated content copied from the user's requested text.
-->

# spot_arm_control

Demo for Boston Dynamics Spot to manipulate tennis balls using MoveIt2 and custom service calls.

## Contents
- ROS 2 C++ package `spot_tennis_demo` with console scripts (see `setup.py`):
  - `ball_selector` — selects closest ball from YOLO 3D detections and publishes `/ball_stable_pose` and `/ball_detected`.
  - `ball_nav_target` — computes navigation target from ball pose.
  - `bin_detector` — detects bin and publishes `/bin_nav_pose`.
  - `nav_manager` — wrapper around Nav2 navigation to move base to hand-relative goals.
  - `bt_executor` — mission/behavior controller (survey, detect → nav to ball → nav to bin → resume).
- `launch/` — `demo_bringup.launch.py` which includes robot bringup and AMCL localization.
- `config/`, `maps/`, `models/` — config files, map, and ML models used by the demo.
- `docker/` — Dockerfile and docker-compose for containerized runs.

---

## Requirements / Dependencies


Runtime dependencies (declared in `package.xml` and used in code):
- ROS 2 (use the distro you have installed; environment variable `$ROS_DISTRO` assumed)
- rclpy, launch, launch_ros
- tf2_ros, tf2_geometry_msgs
- geometry_msgs, std_msgs, std_srvs
- nav2 (Nav2 stack; nav2_msgs action NavigateToPose)
- yolo_msgs (3D detection messages)
- spot_ros / spot_bringup / spot_navigation (robot-specific bringup packages)

Developer tooling:
- colcon (`python3-colcon-common-extensions`)
- rosdep

Notes:
- Some packages (e.g., `yolo_msgs`, `spot_bringup`) may be external. If they are not available through apt/rosdep, clone them into the workspace `src/` before building.

### Repository layout & required sibling repositories

This demo expects a multi-repo ROS 2 workspace where several related packages are present in the same `src/` directory. Make sure you have the following repositories cloned into the same workspace `src/` alongside `spot_tennis_demo` before running `rosdep` and `colcon`:

- spot_manipulation
- spot_ros
- spot_tennis_demo
- yolo_ros (or yolo_bringup / yolo_msgs depending on naming)
- spot_arm_control (this repo)

Example layout:

```
project_ws/
└── src/
  ├── spot_manipulation/
  ├── spot_ros/
  ├── spot_tennis_demo/
  ├── yolo_ros/
  └──spot_arm_control/
```

If any of these are missing, `rosdep install` or `colcon build` may fail to resolve keys such as `spot_ros` or `yolo_msgs`.


---

## Run the demo

The repo includes `pick_ball.launch.py` and `drop_ball.launch.py` which launch the executables that will pick up and drop the ball in the bin, respectively. After running all necessary files as detailed in spot_tennis_demo, you can run either of these commands:

1. Ball pick up:
```
ros2 launch spot_arm_control pick_ball.launch.py
```

2. Ball drop:
```ros2 launch spot_arm_control drop_ball.launch.py```

---

## Contact / author
Maintainer: Clara Summerford — clara.summerford@ my.utexas.edu
