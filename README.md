# Landmark-Based Robot Localization (ROS 2)

A ROS 2 workspace for estimating the 2D pose of a simulated robot from noisy odometry and noisy landmark observations. It contains two localization approaches, an Unscented Kalman Filter (UKF) and Monte Carlo Localization (MCL, a particle filter), together with a simulator and tools for evaluating the results.

## Packages

| Package | Type | Description |
|---|---|---|
| [`fake_robot`](src/fake_robot) | C++ (ament_cmake) | Simulates a robot driving in a circle. Publishes ground truth, noisy odometry, and noisy landmark observations within an observation radius. |
| [`kalman_positioning`](src/kalman_positioning) | C++ (ament_cmake) | UKF-based localizer that fuses odometry with landmark observations. |
| [`mcl_positioning`](src/mcl_positioning) | C++ (ament_cmake) | Particle filter (MCL) localizer using the same inputs. It also publishes the particle cloud. |
| [`combined_launch_pkg`](src/combined_launch_pkg) | Python (ament_python) | Combined launch file, a live RMSE / convergence plotter, and offline rosbag analysis scripts. |

## Data flow

```
                 /robot_gt (Odometry) ─────────────────────────────┐
                 /landmarks_gt (PointCloud2)                       │
 fake_robot ──── /robot_noisy (Odometry) ──────┐                   ▼
                 /landmarks_observed (PointCloud2) ─┐        mcl_liveplot / liveplot
                                               │    │              ▲
                                               ▼    ▼              │
                        kalman_positioning  ──► /robot_estimated_odometry
                        mcl_positioning     ──► /robot_estimated_odometry_mcl
                                            ──► /mcl_particle_cloud (PoseArray)
```

Frames: everything is expressed in `map`. `fake_robot` broadcasts TF for `robot` (ground truth) and `robot_noisy`. Landmark observations are published in the `robot` frame.

## Requirements

- ROS 2 (developed on **Kilted**)
- Eigen3 (for `kalman_positioning`)
- Python: `numpy`, `matplotlib` (TkAgg backend), and `scipy` for the analysis scripts
- `rosbag2_py` with MCAP storage for the offline scripts

## Build

```bash
cd ~/ros2_ws
source /opt/ros/kilted/setup.bash
colcon build
source install/setup.bash
```

To rebuild from scratch:

```bash
rm -rf build/ install/ log/
```

## Running

### Full MCL pipeline with live evaluation

Starts the simulator, the MCL node (after a 1 s delay), and the live plotter:

```bash
ros2 launch combined_launch_pkg combined.launch.py
```

### Individual components

Run each command in its own terminal, from the workspace root. The positioning launch files resolve `landmarks.csv` relative to the current directory.

```bash
ros2 launch fake_robot fake_robot.launch.py
ros2 launch kalman_positioning positioning.launch.py
ros2 launch mcl_positioning mcl_positioning.launch.py
```

Parameters can be overridden on the command line, for example:

```bash
ros2 launch fake_robot fake_robot.launch.py circle_radius:=10.0 observation_radius:=5.0
ros2 launch kalman_positioning positioning.launch.py process_noise_xy:=0.01 measurement_noise_xy:=0.01
ros2 launch mcl_positioning mcl_positioning.launch.py num_particles:=2000 motion_noise_xy:=0.05
```

## Parameters

### `fake_robot`

| Parameter | Launch default | Description |
|---|---|---|
| `landmarks_file` | `share/fake_robot/data/landmarks.csv` | Landmark positions (`id,x,y`) |
| `circle_radius` | `5.0` | Radius of the circular trajectory [m] |
| `circle_speed` | `1.0` | Speed along the circle [m/s] |
| `circle_center_x`, `circle_center_y` | `0.0` | Circle center [m] |
| `observation_radius` | `3.0` | Landmarks within this range are observed [m] |
| `measurement_noise_variance` | `0.01` | Gaussian noise variance on landmark observations |
| `robot_noise_variance_x` | `0.01` | Odometry noise variance on traveled distance [m²] |
| `robot_noise_variance_y` | `0.01` | Currently unused (`x` is used for both) |
| `robot_noise_variance_theta` | `0.0001` | Odometry noise variance on orientation [rad²] |

### `kalman_positioning`

| Parameter | Launch default | Description |
|---|---|---|
| `landmarks_csv_path` | `src/kalman_positioning/landmarks.csv` | Landmark map |
| `process_noise_xy` | `0.0001` | Process noise for x, y. Higher values mean the filter trusts odometry less. |
| `process_noise_theta` | `0.0001` | Process noise for orientation |
| `measurement_noise_xy` | `0.01` | Measurement noise for landmark observations |
| `observation_radius` | `5.0` | Maximum observation range [m] |

### `mcl_positioning`

| Parameter | Launch default | Description |
|---|---|---|
| `landmarks_csv_path` | `src/mcl_positioning/landmarks.csv` | Landmark map |
| `num_particles` | `1000` | Number of particles |
| `motion_noise_xy` | `0.1` | Motion model noise for x, y |
| `motion_noise_theta` | `0.2` | Motion model noise for orientation |
| `measurement_noise` | `0.1` | Measurement model noise |

Both positioning launch files also accept `log_level` and `namespace`.

## Evaluation

### Live plotting

`mcl_liveplot` (started by `combined.launch.py`) compares `/robot_estimated_odometry_mcl` against `/robot_gt` and:

- plots a sliding-window position RMSE in real time
- detects **convergence** (RMSE below 0.25 m for 5 s) and **stability** (the local RMSE change stays below 0.02 m)
- stops after 40 s, appends a row to `mcl_convergence_results.csv` in the current directory, and saves `plots/mcl_position_rmse.png`

[`liveplot.py`](src/combined_launch_pkg/combined_launch_pkg/liveplot.py) does the same for the UKF (`/robot_estimated_odometry`) and also tracks yaw RMSE. It writes to `stabilization_results.csv`. It is not currently registered as an entry point in `setup.py`.

### Offline analysis (rosbag)

Record a run:

```bash
ros2 bag record -s mcap -o src/combined_launch_pkg/my_bag /robot_gt /robot_noisy /robot_estimated_odometry
```

Then use the scripts in [`src/combined_launch_pkg/scripts/`](src/combined_launch_pkg/scripts):

| Script | Purpose |
|---|---|
| `plot_odometry.py` | Plot the ground-truth trajectory from a bag |
| `plot_rmse.py` | Compute and plot RMSE between ground truth and the UKF estimate |
| `rmse_calc.py` | Time-synchronized RMSE computation (interpolation-based) |
| `plot_pn_rmse.py` | Plot RMSE against a swept noise parameter from a CSV file |

The scripts expect the bag at `~/ros2_ws/src/combined_launch_pkg/my_bag`.

## Repository layout

```
ros2_ws/
├── notes.md                 # experiment notes and parameter-tuning results
└── src/
    ├── fake_robot/
    ├── kalman_positioning/
    ├── mcl_positioning/
    └── combined_launch_pkg/
```

## License

`fake_robot` and `kalman_positioning` are licensed under Apache-2.0 (see their `LICENSE` files). `mcl_positioning` and `combined_launch_pkg` do not declare a license yet.
