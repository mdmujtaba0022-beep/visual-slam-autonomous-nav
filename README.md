# Visual SLAM + Autonomous Navigation

A complete autonomous navigation stack for TurtleBot3 in Gazebo Harmonic, featuring dual SLAM methods (LiDAR + Visual), frontier-based autonomous exploration, and multi-waypoint mission execution.

The robot can explore and map an unknown environment entirely on its own — no teleop, no pre-built map.

![Architecture](assets/architecture.png)

## Features

- **Dual SLAM:** LiDAR-based (SLAM Toolbox) and Visual (ORB-SLAM3 RGB-D)
- **Frontier-based autonomous exploration:** Detects unexplored regions and navigates to them automatically
- **Multi-waypoint missions:** Programmatic goal sequencing with FollowWaypoints
- **Custom Gazebo warehouse:** Corridors, shelves, dead ends, narrow passages
- **Quantitative metrics:** Map completeness, path efficiency, localization error, map IoU comparison
- **Recovery behaviors:** Spin, backup, wait — handles tight spaces and stuck states
- **Full RViz visualization:** Map, costmaps, laser scan, paths, frontier markers, waypoints

## Architecture

```
Gazebo Harmonic (Warehouse World)
    ├── LiDAR (/scan) ──────────► SLAM Toolbox ──► /map (2D occupancy grid)
    ├── RGB-D Camera ───────────► ORB-SLAM3 ────► 3D point cloud + 6DOF pose
    └── Odometry (/odom) ──────►┐
                                 │
                            Nav2 Stack
                         ┌───────┴───────┐
                         │  Costmaps     │
                         │  Planner (A*) │
                         │  MPPI Controller │
                         │  Recovery     │
                         └───────┬───────┘
                                 │
                    ┌────────────┴────────────┐
                    │                         │
            Frontier Explorer         Waypoint Manager
         (autonomous exploration)   (mission execution)
```

## Quick Start

### Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic

### Install Dependencies

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-twist-mux

# TurtleBot3 (build from source for Jazzy)
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build --symlink-install

# Environment
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
```

### Build

```bash
cd ~/visual-slam-autonomous-nav
colcon build --packages-select autonomous_nav --symlink-install
source install/setup.bash
```

### Run

```bash
# Terminal 1: Launch TurtleBot3 in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Autonomous exploration (maps environment by itself)
ros2 launch autonomous_nav autonomous_explore.launch.py

# OR: Full stack with metrics
ros2 launch autonomous_nav full_stack.launch.py

# OR: Run a waypoint mission on a saved map
ros2 launch autonomous_nav navigation.launch.py map:=./src/autonomous_nav/maps/warehouse.yaml
```

### Custom warehouse world + waypoint mission

```bash
# Terminal 1: warehouse world (robot spawns at the origin used by the mission file)
ros2 launch autonomous_nav warehouse_sim.launch.py

# Terminal 2: explore and map it, then save the map (see below)
ros2 launch autonomous_nav full_stack.launch.py

# Or localize on the saved map and run config/warehouse_mission.yaml
ros2 launch autonomous_nav navigation.launch.py run_mission:=true
```

### Docker (any OS, including Windows via WSLg)

`docker/` contains an Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + Nav2 + TurtleBot3 image.
`docker/run.sh` mounts this repo at `/ws`, builds the package on first start, and forwards the
GUI (WSLg on Windows, with GPU-accelerated OpenGL through Mesa D3D12). From Git Bash or Linux:

```bash
docker/run.sh                                     # builds the image on first run, opens a shell
docker exec -it vslam-nav /entrypoint.sh bash     # extra terminals in the same container
make test                                         # unit tests (inside the container)
```

### Save a Map

```bash
ros2 run nav2_map_server map_saver_cli -f ./src/autonomous_nav/maps/warehouse
```

## SLAM Comparison

| Metric | SLAM Toolbox (LiDAR) | ORB-SLAM3 (Visual) |
|--------|---------------------|-------------------|
| Map type | 2D occupancy grid | 3D sparse point cloud + 6DOF pose |
| Sensor | 2D LaserScan | RGB-D camera |
| Strengths | Robust in corridors, fast | Rich features, works outdoors |
| Weaknesses | No height info | Needs texture, fragile on blank walls |

## Configuration

### Nav2 Parameters (`config/nav2_params.yaml`)

Key parameters to tune:
- `inflation_radius`: Keep robot away from walls (default: 0.55m)
- `robot_radius`: TurtleBot3 Waffle is 0.22m
- `vx_max`: Max forward speed (default: 0.26 m/s)
- `use_astar`: A* planner (faster than Dijkstra)

### Frontier Explorer Parameters

- `min_frontier_size`: Minimum cells to consider a frontier (default: 5)
- `blacklist_radius`: Distance to blacklist failed goals (default: 0.5m)
- `recheck_interval`: Seconds between frontier scans (default: 5.0)
- `exploration_timeout`: Nav goal timeout (default: 30.0s)

## Project Structure

```
src/autonomous_nav/
├── autonomous_nav/
│   ├── frontier_explorer.py   # Autonomous exploration node
│   ├── waypoint_manager.py    # Multi-goal mission executor
│   ├── orb_slam_bridge.py     # ORB-SLAM3 pose → Nav2 TF bridge
│   ├── metrics.py             # Exploration metrics collection
│   └── utils.py               # Pose/quaternion helpers
├── config/
│   ├── nav2_params.yaml       # Nav2 stack parameters
│   ├── slam_toolbox.yaml      # SLAM Toolbox configuration
│   ├── orb_slam3.yaml         # ORB-SLAM3 camera config
│   ├── warehouse_mission.yaml # Waypoint mission definition
│   └── rviz_config.rviz       # RViz visualization layout
├── launch/
│   ├── slam_mapping.launch.py       # SLAM + Nav2
│   ├── navigation.launch.py         # Nav2 with saved map
│   ├── orb_slam3.launch.py          # Visual SLAM launch
│   ├── autonomous_explore.launch.py # Full autonomous exploration
│   └── full_stack.launch.py         # Everything + metrics
└── worlds/
    └── warehouse.world        # Custom Gazebo warehouse
```

## Common Issues

- **`use_sim_time` must be True everywhere** — if even one node uses wall clock, TF lookups fail silently
- **TwistStamped vs Twist on /cmd_vel** — Gazebo Harmonic may publish `TwistStamped`; use `twist_mux` if Nav2 complains
- **ORB-SLAM3 needs texture** — add materials to walls in your SDF; blank walls cause tracking loss
- **Check TF tree** — run `ros2 run tf2_tools view_frames` to verify `map → odom → base_link → sensors`

## Tech Stack

ROS2 Jazzy | Nav2 | SLAM Toolbox | ORB-SLAM3 | Gazebo Harmonic | TurtleBot3 | Python | SDF
