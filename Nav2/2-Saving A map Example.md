# Mapping + Relocalization in Nav2 (ROS2 Jazzy)

## 0. One-time fix: comment `route_server` in Nav2

Nav2 on Jazzy may hang on  
`lifecycle_manager_navigation: Waiting for service route_server/get_state...`  

To avoid this, disable `route_server` in `navigation_launch.py`.

```bash
sudo nano /opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py
```

In that file:

1. **In the `lifecycle_nodes` list**, remove the `'route_server'` entry:

   ```python
   lifecycle_nodes = [
       'controller_server',
       'smoother_server',
       'planner_server',
       # 'route_server',
       'behavior_server',
       'velocity_smoother',
       'collision_monitor',
       'bt_navigator',
       'waypoint_follower',
       'docking_server',
   ]
   ```

Save and exit.

---

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
