install nav2

```bash
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
```

ros jazzy

to start Localization
```bash
source /opt/ros/<ros2-distro>/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=False headless:=False
```


# Terminal 2 – teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```



## Commands
```bash
ros2 topic info /map          # shows type (e.g., nav_msgs/msg/OccupancyGrid)
ros2 interface show nav_msgs/msg/OccupancyGrid   # view fields
```

# MAPPTING

## 1. Build the map (SLAM mode)

Start the TB3 sim with SLAM enabled:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=False
```

Drive the robot around until you are happy with the map, then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f my_maze
```

This creates two files in the current folder:

- `my_maze.pgm`
- `my_maze.yaml`

These are your static map.

---

## 2. Later: use the saved map for localization (no SLAM)

Next time, you don’t run SLAM; you start Nav2 in localization mode with the saved map:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py   slam:=False headless:=Flase  map:=/full/path/to/my_maze.yaml
```

Then in RViz:

1. Use **2D Pose Estimate** to set the initial robot pose on the map.
2. Use **Nav2 Goal** to send a goal; the robot should localize and navigate using `my_maze`.
