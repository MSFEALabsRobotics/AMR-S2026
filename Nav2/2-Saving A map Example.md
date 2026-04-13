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

# 1. Build the map (SLAM mode)

Start the TB3 sim with SLAM enabled:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=False
```

# Terminal – teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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



## Ros Commands, Extract the map to python
```bash
ros2 topic info /map          # shows type (e.g., nav_msgs/msg/OccupancyGrid)
ros2 interface show nav_msgs/msg/OccupancyGrid   # view fields
```

## Message Type: `nav_msgs/OccupancyGrid`

### header
- **frame_id**: typically "map" (the fixed world frame).
- **stamp**: time the map corresponds to.

### info (`nav_msgs/MapMetaData`)
- **resolution** (float32): meters per cell (e.g., 0.05 → each pixel is 5 cm).
- **width, height** (uint32): number of cells (columns, rows).
- **origin** (`geometry_msgs/Pose`): pose of the cell (0,0) (bottom-left of the grid array) in the map frame.
  - **origin.position.x/y**: world coordinates (meters) of the lower-left corner.
  - **origin.orientation**: rotation of the grid w.r.t. the map frame (usually identity; if the map is rotated, it’s a quaternion).

### data (flattened row-major `int8` array)
- **Length** = `width * height`.
- Each cell is:
  - **-1** → unknown
  - **0..100** → occupancy probability in %, where higher = more likely **occupied**
- Conventionally, planners threshold around **50** (free < 50, occupied ≥ 50).

---

## Python: Display One Map From `/map`
Subscribes to `/map` and shows one occupancy grid frame.

```python
# map_show_once.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class MapOnce(Node):
    def __init__(self):
        super().__init__('map_show_once')
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, 10)
        self.img = None
        self.meta = None

    def cb(self, msg: OccupancyGrid):
        # Convert occupancy grid -> displayable image
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        img = np.zeros_like(data, dtype=np.uint8)
        img[data == -1] = 127              # unknown
        img[(data >= 0) & (data < 50)] = 255  # free
        img[data >= 50] = 0                # occupied

        self.img  = np.flipud(img)  # flip to match RViz view
        self.meta = (w, h, msg.info.resolution)

def main():
    rclpy.init()
    node = MapOnce()

    # Spin until the first map arrives
    while rclpy.ok() and node.img is None:
        rclpy.spin_once(node, timeout_sec=0.5)

    rclpy.shutdown()

    if node.img is None:
        print("No /map received.")
        return

    w, h, res = node.meta
    plt.figure("Occupancy Grid (/map)")
    plt.imshow(node.img, cmap='gray', origin='upper')
    plt.title(f"size: {w}x{h}, res: {res:.3f} m/px")
    plt.axis('off')
    plt.tight_layout()
    plt.show()  # blocks until you close the window

if __name__ == '__main__':
    main()
```

### Run
```bash
source /opt/ros/jazzy/setup.bash
python3 map_show_once.py
```



# 2. Later: use the saved map for localization (no SLAM)

Next time, you don’t run SLAM; you start Nav2 in localization mode with the saved map:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=False headless:=False map:=$HOME/my_maze.yaml
```

Then in RViz:

1. Use **2D Pose Estimate** to set the initial robot pose on the map.
2. Use **Nav2 Goal** to send a goal; the robot should localize and navigate using `my_maze`.



