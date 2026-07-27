# TIAGo Autonomous Navigation — Final Project (26SS AR)

**Course:** 26SS Autonomous Robotics (RWU)
**Authors:** ab-257929, ak-258066
**Repository:** `26ss_ab-257929_ak-258066_final`

## Overview

This project runs a **TIAGo robot** (camera + lidar) in **Gazebo simulation**, tasked with **following a moving TF frame (`target_pose`) through a cluttered, partly-unmapped environment**. The mission logic is orchestrated by a single ROS2 action server, `fullrun` (package `26ss_ar_final`). Given a goal specifying a task, the robot:

1. Is teleported to that task's ground-truth spawn pose in Gazebo (`/gazebo/set_entity_state`)
2. Localizes itself with **AMCL**, confirmed via an `/initialpose` retry loop
3. Waits for the `target_pose` TF frame to appear
4. Continuously re-sends Nav2 goals toward `target_pose` as it moves, using the **DWB local planner**

Supporting this: a custom costmap setup (lidar + depth-camera point cloud obstacle sources, a periodic stale-obstacle clearer), a calibrated odometry model for the simulated TIAGo base, and a head-sweep behavior that dips the head down periodically to catch near-field ground obstacles the main sensors would otherwise miss.

## Repository Structure

```
.
├── config/
│   ├── fastdds_shm.xml              # Fast DDS shared-memory transport profile
│   └── tiago_custom_fullrunv13.yaml # Nav2 stack config (AMCL, costmaps, DWB controller, etc.)
├── docker/
│   ├── Dockerfile                   # Base image definition (image is pre-built/pulled from registry)
│   ├── fullrun.yml                  # << the compose file actually used to run everything
│   └── task2.yml / task3.yml / task4.yml / tiago_sim.yml   # earlier per-task variants, not needed — fullrun.yml supersedes them
├── maps/                            # map_ar_ss26.pgm/.yaml used for AMCL localization
├── models/                          # Gazebo world/robot models, sim_ar_ss26.launch.py override
├── scripts/
│   ├── spawn_manager.py             # run manually — manages spawning in the arena
│   └── tf_publisher.py              # run manually — publishes the target_pose TF the robot follows
└── src/
    ├── 26ss_ar_final/                # main action server package (fullrun, task1, tf_pauser)
    ├── ar_final_interfaces/          # the ArFinal.action definition
    ├── pmb2_controller_configuration/# calibrated mobile-base controller config (wheel multipliers)
    └── tf_follower/                  # supporting nodes: costmap_clearer, pose_setter, snapshot/head-sweep experiments
```

## Prerequisites

- Docker + Docker Compose, NVIDIA Container Toolkit (`runtime: nvidia`)
- Access to the RWU Docker registry (`fbe-dockerreg.rwu.de`) for the `tiago_base:sim-ar` image
- X11 available for Gazebo/RViz display forwarding
- ROS2 Humble (only needed if you want to run the helper scripts from the host instead of inside the container)

## Quick Start

### 1. Clone the repository

```bash
git clone https://gitlab.rwu.de/stud-iki/vl-ar/26ss_ab-257929_ak-258066_final.git
cd 26ss_ab-257929_ak-258066_final/docker
```

### 2. Bring up the simulation stack

```bash
docker compose -f fullrun.yml up
```

This single command:
- Builds the `iki_ws` overlay workspace inside the container (`colcon build`)
- Launches Gazebo + RViz + Nav2 with the TIAGo robot (camera + lidar) via `sim_ar_ss26.launch.py`, using `config/tiago_custom_fullrunv13.yaml` as the Nav2 config
- After a 25s warm-up, starts the `fullrun` action server (`26ss_ar_final`) and the `tf_pauser` node
- Starts `tf_follower`'s `costmap_clearer` node (3s stale-obstacle clear interval)

> Re-run `docker compose -f fullrun.yml up` (or restart the container) any time you edit `config/tiago_custom_fullrunv13.yaml` — Nav2 params are only picked up at launch.

### 3. Send a task goal

```bash
ros2 action send_goal /fullrun ar_final_interfaces/action/ArFinal "{task: 'task1'}"
```

Valid `task` values: `task1`, `task2`, `task3`, `task4`, or `fullrun` (the full end-to-end run). Each triggers: teleport → AMCL localization confirm → wait for `target_pose` → continuous follow.

### 4. Start the support scripts

These are **not** started automatically by the compose file and must be run manually (e.g. `docker exec` into `tiago-container`, or from the host if sourced):

```bash
python3 scripts/spawn_manager.py
python3 scripts/tf_publisher.py
```

- `tf_publisher.py` publishes the `target_pose` TF frame the robot follows.
- `spawn_manager.py` manages spawning/deleting obstacles as per the robot's location.

## Configuration

The single Nav2 config file, `config/tiago_custom_fullrunv13.yaml`, covers:

| Section | Notes |
|---|---|
| `amcl` | `alpha1: 0.2`, `alpha2–4: 0.05`; `min_particles: 800`, `max_particles: 4000`; `laser_max_range: 100.0`, scan topic `/scan_raw` |
| `controller_server` | `FollowPath` uses **DWB** (`dwb_core::DWBLocalPlanner`); `max_vel_x/theta: 1.0`, critics: Oscillation/BaseObstacle/GoalAlign/PathAlign/PathDist/GoalDist |
| `global_costmap` | `obstacle_layer` uses `/scan_raw`; the depth-camera pointcloud source is currently **commented out** here |
| `local_costmap` | `voxel_layer` fuses both `/scan_raw` and the depth camera pointcloud (`/head_front_camera/depth/points`) as obstacle sources, rolling 5×5 m window |
| `planner_server` | `NavfnPlanner`, `allow_unknown: true` (needed for the unmapped-region portion of the task) |
| `slam_toolbox` | present for mapping mode, not used during the follow task (`navigation:=True slam:=False` at launch) |

`config/fastdds_shm.xml` configures Fast DDS to use shared-memory transport, mounted into the container to avoid DDS discovery issues on `network_mode: host`.

Odometry/wheel calibration for the simulated base lives in `src/pmb2_controller_configuration/`.

## Package Overview (`src/`)

- **`26ss_ar_final`** — the main package. `fullrun.py` contains `FullRunActionServer` (teleport → localize → follow) and `HeadSweepNode` (periodic head-dip to catch near-field ground obstacles). Also includes `task1.py` and `tf_pauser.py`.
- **`ar_final_interfaces`** — defines the `ArFinal` action (goal field: `task`) used by the action server.
- **`pmb2_controller_configuration`** — calibrated mobile base controller parameters (wheel separation/radius multipliers).
- **`tf_follower`** — supporting/utility package: `costmap_clearer.py` (run in production, clears stale costmap obstacles on an interval), `pose_setter.py` (hardcoded AMCL target poses per task, shared with `26ss_ar_final`), and `snapshot_node.py`. The various `tf_follower_fullrunv1–v5.py` files are earlier development iterations of the follow logic kept for reference and are not part of the active runtime path — the current logic lives in `26ss_ar_final/fullrun.py`.

## Docker Compose Files

Only **`fullrun.yml`** is needed to run the project — it covers all four tasks and the full run through the action server's `task` field. `task2.yml`, `task3.yml`, `task4.yml`, and `tiago_sim.yml` are earlier per-task compose variants, superseded by `fullrun.yml`, and kept only for reference.

## Authors

- ab-257929
- ak-258066