# Visual SLAM + Autonomous Navigation

A TurtleBot3 robot that **maps an unknown building by itself** in the Gazebo simulator: no remote control and no map to start from.

It drives toward the edges of what it has seen so far, builds a map as it goes, and stops once nothing is left to explore. You can then use the saved map to send the robot on a multi-stop mission.

Built with ROS 2 Jazzy, Nav2, SLAM Toolbox and Gazebo Harmonic.

---

## Contents

1. [What it does](#what-it-does)
2. [How it works](#how-it-works)
3. [Quick start with Docker (recommended, works on Windows)](#quick-start-with-docker)
4. [Native install (Ubuntu 24.04)](#native-install-ubuntu-2404)
5. [Things you can run](#things-you-can-run)
6. [Configuration](#configuration)
7. [Project layout](#project-layout)
8. [Troubleshooting](#troubleshooting)

---

## What it does

| Feature | Description |
|---|---|
| **Autonomous exploration** | Finds *frontiers* (edges between explored and unexplored space) and drives to them until the whole area is mapped. |
| **LiDAR SLAM** | SLAM Toolbox builds a 2D map from the laser scanner while the robot moves. |
| **Waypoint missions** | Visits a list of goals from a YAML file, in order, on a saved map. |
| **Custom warehouse world** | Shelves, aisles, a dead end, a narrow passage, pillars and crates. |
| **Recovery behaviours** | Spins, backs up or waits when the robot gets stuck. |
| **Metrics** | Logs map coverage and distance driven over time to a CSV file. |
| **Visual SLAM (experimental)** | Integration code for ORB-SLAM3. It needs extra setup that is not included; see [Visual SLAM status](#visual-slam-orb-slam3-status). |

## How it works

```
                 Gazebo simulator (warehouse world)
                 │                │               │
           laser /scan      wheel /odom     camera images
                 │                │               │
                 ▼                │               ▼
          SLAM Toolbox            │        ORB-SLAM3 (experimental)
       builds the 2D /map         │
                 │                │
                 ▼                ▼
       ┌──────────────── Nav2 ────────────────┐
       │ costmaps → A* planner → MPPI control │
       │ + recovery (spin, back up, wait)     │
       └───────────────────┬──────────────────┘
                           │  sends goals to Nav2
             ┌─────────────┴─────────────┐
             ▼                           ▼
     Frontier Explorer            Waypoint Manager
    "go map the unknown"        "visit these points"
```

**The exploration loop** (every second):

1. Read the latest map.
2. Find frontier cells and group them into clusters.
3. Score each cluster: bigger is better, closer is better.
4. Send the best one to Nav2 as a goal.
5. While driving, skip to the next frontier if the current one gets mapped on the way.
6. Blacklist goals that fail, time out (30 s), or still don't clear after two visits.
7. When no frontiers are left, print a summary and stop.

---

## Quick start with Docker

This is the easiest way to run the project, and the only way on Windows. Everything (ROS 2, Gazebo, Nav2, TurtleBot3) is in a container, so you don't install anything else.

### Requirements

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (on Windows, uses the WSL 2 backend)
- **Windows:** Windows 11 (or Windows 10 with WSLg) so Gazebo and RViz windows can appear. Run the commands below in **Git Bash**.
- **Linux:** Docker plus an X server (any normal desktop).

### Step 1: Start the simulator

In the repository folder:

```bash
docker/run.sh ros2 launch autonomous_nav warehouse_sim.launch.py
```

- The **first run builds the Docker image**, which downloads several GB and takes a while. Later runs start in seconds.
- The package is compiled automatically the first time the container starts.
- A **Gazebo** window opens showing the warehouse and the robot.

### Step 2: Start exploring

Open a **second terminal** and run:

```bash
docker exec -it vslam-nav /entrypoint.sh ros2 launch autonomous_nav full_stack.launch.py
```

An **RViz** window opens. Watch the map grow as the robot explores. When it finishes, the terminal prints:

```
No frontiers remaining — exploration complete!
```

Mapping the full warehouse takes roughly 20–30 minutes of simulated time.

### Step 3: Save the map

```bash
docker exec vslam-nav /entrypoint.sh make map-save
```

This writes `src/autonomous_nav/maps/warehouse.pgm` and `warehouse.yaml`.

### Step 4: Run a waypoint mission (optional)

Restart the simulator so the robot is back at its starting point. Then:

```bash
# Terminal 1
docker stop vslam-nav
docker/run.sh ros2 launch autonomous_nav warehouse_sim.launch.py

# Terminal 2
docker exec -it vslam-nav /entrypoint.sh ros2 launch autonomous_nav navigation.launch.py run_mission:=true
```

The robot localizes on the saved map and visits the 10 points in `config/warehouse_mission.yaml`. When it's done, the terminal prints `Mission complete! All 10 waypoints reached.`

### Stop everything

```bash
docker stop vslam-nav
```

### Other useful commands

```bash
docker/run.sh                                     # just open a shell in the container
docker exec -it vslam-nav /entrypoint.sh bash     # extra shell in the running container
docker exec vslam-nav /entrypoint.sh make test    # run the unit tests
```

> **GPU:** On Windows with an NVIDIA/AMD/Intel GPU, `run.sh` turns on hardware-accelerated graphics automatically. If the windows are blank or crash, force software rendering:
> `LIBGL_ALWAYS_SOFTWARE=1 docker/run.sh ...`

---

## Native install (Ubuntu 24.04)

Use this if you already run ROS 2 Jazzy on Ubuntu 24.04 and prefer not to use Docker.

### 1. Install dependencies

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-twist-mux \
    ros-jazzy-rmw-cyclonedds-cpp python3-scipy python3-pil
```

### 2. Build TurtleBot3 from source

```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build --symlink-install
```

### 3. Set up your environment

Add these lines to `~/.bashrc`, then open a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 4. Build this package

```bash
cd ~/visual-slam-autonomous-nav
make build
source install/setup.bash
```

Then run the same `ros2 launch ...` commands as in the Docker steps, without the `docker ...` prefix.

---

## Things you can run

Every launch file needs a simulator running first, in its own terminal.

**Simulators** (pick one):

| Command | World |
|---|---|
| `ros2 launch autonomous_nav warehouse_sim.launch.py` | Custom warehouse. Robot starts at (0, 0). |
| `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` (or `make sim`) | Small standard TurtleBot3 arena. |

**Robot behaviour** (pick one):

| Command | What happens |
|---|---|
| `ros2 launch autonomous_nav full_stack.launch.py` | SLAM + Nav2 + exploration + metrics + RViz. **Start here.** |
| `ros2 launch autonomous_nav autonomous_explore.launch.py` | Same, without metrics. |
| `ros2 launch autonomous_nav slam_mapping.launch.py` | SLAM + Nav2 only, no RViz. Start RViz yourself (`rviz2 -d src/autonomous_nav/config/rviz_config.rviz`) and set goals with its **2D Goal Pose** button. |
| `ros2 launch autonomous_nav navigation.launch.py` | Localize on the saved map (no SLAM). |
| `ros2 launch autonomous_nav navigation.launch.py run_mission:=true` | Same, then run the waypoint mission. |

**Launch options:**

| Option | Default | Used by |
|---|---|---|
| `enable_exploration:=false` | `true` | `full_stack` (turn off the explorer) |
| `enable_metrics:=false` | `true` | `full_stack` (turn off metrics) |
| `map:=/path/to/map.yaml` | `maps/warehouse.yaml` | `navigation` |
| `mission_file:=/path/to/mission.yaml` | `config/warehouse_mission.yaml` | `navigation` |

**Make shortcuts** (run inside the container or a sourced workspace): `make build`, `make test`, `make sim`, `make explore`, `make full`, `make slam`, `make nav`, `make map-save`, `make clean`.

### Metrics output

`full_stack.launch.py` samples progress every 2 seconds. When you stop it (Ctrl+C), it writes a CSV file to:

```
install/autonomous_nav/share/autonomous_nav/metrics_output/exploration_metrics.csv
```

Columns: `time_s`, `completeness_pct` (share of the map's bounding box that is known), `distance_m`, and `localization_error_m`. Localization error stays empty unless a ground-truth `base_link_gt` frame is published, which this project doesn't do by default.

To compare two saved maps (IoU, precision, recall), use `compare_maps()` in `autonomous_nav/metrics.py`.

### Writing your own mission

Edit `config/warehouse_mission.yaml`. Coordinates are in metres, in the map frame; the robot's start point is (0, 0). `yaw` is the direction to face, in radians.

```yaml
waypoints:
  - {x: 1.0, y: -2.0, yaw: 0.0,  label: "Aisle 1 start"}
  - {x: 3.5, y:  4.0, yaw: 1.57, label: "Open area NE"}
```

---

## Configuration

### Navigation: `config/nav2_params.yaml`

| Parameter | Value | Meaning |
|---|---|---|
| `robot_radius` | 0.22 m | Size of the TurtleBot3 Waffle. |
| `inflation_radius` | 0.55 m | How far the robot keeps from walls. |
| `vx_max` | 0.26 m/s | Top forward speed. |
| `use_astar` | true | A* path planning (faster than Dijkstra). |
| `enable_stamped_cmd_vel` | true | Required: Gazebo Harmonic's TurtleBot3 only accepts `TwistStamped` velocity commands. |
| `amcl.initial_pose` | (0, 0, 0) | Where the robot starts when localizing on a saved map. |

### Exploration: set in the launch files

| Parameter | Default | Meaning |
|---|---|---|
| `min_frontier_size` | 5 cells | Ignore frontiers smaller than this (noise). |
| `recheck_interval` | 1.0 s | How often to look for a new frontier. |
| `exploration_timeout` | 30.0 s | Give up on a goal after this long. |
| `blacklist_radius` | 0.5 m | Skip frontiers this close to a failed goal. |

### SLAM: `config/slam_toolbox.yaml`

Map resolution is 5 cm. Loop closure is on, and the map updates every 2 s.

---

## Project layout

```
visual-slam-autonomous-nav/
├── docker/                       Container setup
│   ├── Dockerfile                  Ubuntu 24.04 + ROS 2 Jazzy + Nav2 + TurtleBot3
│   ├── run.sh                      Starts the container with GUI and GPU forwarding
│   └── entrypoint.sh               Sources ROS and builds the package on first start
├── Makefile                      Shortcuts (build, test, map-save, ...)
└── src/autonomous_nav/           The ROS 2 package
    ├── autonomous_nav/
    │   ├── frontier_explorer.py    Autonomous exploration node
    │   ├── waypoint_manager.py     Runs multi-waypoint missions
    │   ├── metrics.py              Coverage/distance logging and map comparison
    │   ├── orb_slam_bridge.py      Turns ORB-SLAM3 poses into ROS transforms
    │   └── utils.py                Pose and angle helpers
    ├── config/
    │   ├── nav2_params.yaml        Navigation settings
    │   ├── slam_toolbox.yaml       LiDAR SLAM settings
    │   ├── orb_slam3.yaml          Camera settings for ORB-SLAM3
    │   ├── warehouse_mission.yaml  Waypoint list
    │   └── rviz_config.rviz        RViz layout
    ├── launch/                   See "Things you can run"
    ├── maps/                     Saved maps (warehouse.pgm / .yaml)
    ├── test/                     Unit tests (run with `make test`)
    └── worlds/warehouse.world    The custom Gazebo world
```

---

## Troubleshooting

**The robot doesn't move and Nav2 logs "Failed to make progress".**
Nav2 must publish `TwistStamped` velocity commands. Check that `enable_stamped_cmd_vel: true` is still set in `nav2_params.yaml`. Then confirm both sides of `/cmd_vel` use the same type:
`ros2 topic info /cmd_vel -v`

**Nothing shows up in RViz, or transform (TF) errors appear.**
Every node must use simulation time (`use_sim_time: true`). Check the transform chain with:
`ros2 run tf2_tools view_frames` (expected: `map → odom → base_footprint → base_link → sensors`).

**The waypoint mission is rejected, or the robot goes to the wrong places.**
The mission assumes the robot starts at (0, 0) in the warehouse world. Restart the simulator before running a mission, and make sure the map in `maps/` was made in the same world.

**Gazebo or RViz windows don't appear (Docker on Windows).**
Run `docker/run.sh` from **Git Bash**, not PowerShell or CMD, and make sure Docker Desktop is running. Try `LIBGL_ALWAYS_SOFTWARE=1` if the windows are black.

**Exploration ends with coverage well under 100%.**
That's normal. Coverage is measured over the map's rectangular bounding box, so corners outside the walls, and the insides of shelves, can never be seen.

### Visual SLAM (ORB-SLAM3) status

`orb_slam3.launch.py` and `full_stack.launch.py slam_method:=orb_slam3` need things this repository doesn't provide:

- an ORB-SLAM3 ROS 2 wrapper package called `ros2_orb_slam3` with an `rgbd_node` executable
- the ORB vocabulary file at `~/ORB_SLAM3/Vocabulary/ORBvoc.txt`
- a **depth** camera on the robot: the simulated TurtleBot3 Waffle only has a colour camera

Until those are added, use the default LiDAR SLAM (SLAM Toolbox).

| | SLAM Toolbox (LiDAR) | ORB-SLAM3 (Visual) |
|---|---|---|
| Output | 2D occupancy grid | 3D sparse point cloud + 6-DOF pose |
| Sensor | 2D laser scanner | RGB-D camera |
| Good at | Corridors, fast and robust | Feature-rich scenes, outdoors |
| Struggles with | No height information | Blank walls (needs texture) |

---

**Tech stack:** ROS 2 Jazzy · Nav2 · SLAM Toolbox · ORB-SLAM3 · Gazebo Harmonic · TurtleBot3 · Python · Docker
