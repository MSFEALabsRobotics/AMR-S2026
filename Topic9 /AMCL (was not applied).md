# Minimal AMCL Tutorial for a Differential-Drive Robot in Gazebo

## Goal

Localize a differential-drive robot on a known 2D map using:

* Gazebo robot simulation
* wheel odometry
* 2D LiDAR
* map server
* AMCL
* RViz

---

## 1) What your robot must already provide

Before AMCL, your robot should already have:

* `/odom`
* `/scan`
* TF for `odom -> base_link`
* TF for sensor frames, such as `base_link -> laser_frame`

Your TF tree should look like this:

```text
map -> odom -> base_link -> laser_frame
```

Where:

* `odom -> base_link` comes from your robot odometry
* `map -> odom` comes from AMCL

---

## 2) Files you need in your package

A small package layout can be:

```text
my_robot_localization/
├── launch/
│   └── amcl_localization.launch.py
├── maps/
│   ├── room_map.pgm
│   └── room_map.yaml
└── config/
    └── amcl_params.yaml
```

---

## 3) Create a map YAML file

Example `maps/room_map.yaml`:

```yaml
image: room_map.pgm
resolution: 0.05
origin: [-5.0, -5.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
mode: trinary
```

---

## 4) Create a minimal AMCL parameter file

Example `config/amcl_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: true

    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_link
    scan_topic: /scan

    min_particles: 200
    max_particles: 1000

    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2

    z_hit: 0.5
    z_short: 0.05
    z_max: 0.05
    z_rand: 0.5
    sigma_hit: 0.2
    lambda_short: 0.1

    laser_model_type: likelihood_field
    max_beams: 60

    update_min_d: 0.20
    update_min_a: 0.20

    resample_interval: 1
    transform_tolerance: 1.0
    recovery_alpha_slow: 0.0
    recovery_alpha_fast: 0.0

map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""

lifecycle_manager_localization:
  ros__parameters:
    use_sim_time: true
    autostart: true
    node_names:
      - map_server
      - amcl
```

---

## 5) Create the launch file

Example `launch/amcl_localization.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = "my_robot_localization"
    pkg_share = get_package_share_directory(pkg_name)

    map_file = os.path.join(pkg_share, "maps", "room_map.yaml")
    params_file = os.path.join(pkg_share, "config", "amcl_params.yaml")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {"yaml_filename": map_file}
        ]
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[params_file]
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager
    ])
```

---

## 6) Run Gazebo first

Start your Gazebo world and diff-drive robot first, so these are already available:

* `/odom`
* `/scan`
* TF from robot base to laser

Check:

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /scan
ros2 run tf2_tools view_frames
```

If `/scan` or `/odom` is missing, AMCL will not work.

---

## 7) Launch localization

In a new terminal:

```bash
ros2 launch my_robot_localization amcl_localization.launch.py
```

---

## 8) Open RViz

Open RViz:

```bash
rviz2
```

Add these displays:

* Map
* LaserScan
* TF
* PoseWithCovariance
* ParticleCloud

Set fixed frame to:

```text
map
```

Then use **2D Pose Estimate** to give the robot an initial pose guess.

---

## 9) Drive the robot and watch AMCL work

Now drive the robot manually, for example with keyboard teleop.

As the robot moves:

* odometry predicts motion
* laser scans are compared to the map
* particles reweight and resample
* the pose estimate improves

---

## 10) Useful checks

### Check if map is being published

```bash
ros2 topic echo /map
```

### Check AMCL pose

```bash
ros2 topic echo /amcl_pose
```

### Check particles

```bash
ros2 topic list | grep particle
```

### Check TF

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

If AMCL is working, you should see a live `map -> odom` transform.

---

## 11) Common problems

### Problem 1: No particles / no pose update

Usually one of these:

* `/scan` missing
* `/odom` missing
* wrong frame names
* fixed frame in RViz not set to `map`

### Problem 2: Robot appears but localization never converges

Usually:

* map does not match the world
* bad laser topic name
* laser frame missing in TF
* initial pose estimate too far off

### Problem 3: AMCL starts but nothing becomes active

Usually:

* lifecycle manager not launched
* node names in `node_names` do not match actual node names

---

## 12) What to explain to students

This tutorial is good for showing:

* odometry drift
* pose uncertainty
* particle filters
* local localization
* global localization
* kidnapped robot recovery

A very good demo is:

1. start robot
2. localize with 2D Pose Estimate
3. drive in a square
4. watch particles tighten
5. teleport robot in Gazebo
6. observe loss and recovery

---

## 13) Smallest concept summary

You do **not** need the full Nav2 stack.

For this chapter, only use:

* Gazebo robot
* `/odom`
* `/scan`
* static map
* `nav2_map_server`
* `nav2_amcl`
* `nav2_lifecycle_manager`
* RViz

That keeps the lesson focused on localization only, without planners, controllers, costmaps, or behavior trees.

---

## 14) Super-short classroom version

```text
Gazebo robot publishes:
  /odom
  /scan
  odom -> base_link

AMCL provides:
  map -> odom

Map server provides:
  /map

RViz:
  fixed frame = map
  use 2D Pose Estimate
```
