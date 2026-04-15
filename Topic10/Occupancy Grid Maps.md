# Occupancy Grid Mapping in ROS 2 — In-Class Solution

## Goal

Build a simple **2D occupancy grid map** using:

* `/scan` from LiDAR
* `/odom` from odometry

and publish:

* `/map` as `nav_msgs/msg/OccupancyGrid`

This is **not SLAM**. We assume the robot pose is already known from odometry.

---

## Main Idea

For each laser beam:

1. get the robot pose `(x_r, y_r, theta)`
2. compute the beam endpoint in world coordinates
3. convert robot position and endpoint into grid cells
4. mark all cells along the beam as **free**
5. mark the last cell as **occupied**

---

## ROS 2 Messages Used

### Inputs

* `sensor_msgs/msg/LaserScan`
* `nav_msgs/msg/Odometry`

### Output

* `nav_msgs/msg/OccupancyGrid`

---

## Occupancy Grid Convention

In ROS occupancy grids:

* `-1` = unknown
* `0` = free
* `100` = occupied

In this solution, we keep an internal 2D grid using these same values.

---

## Map Parameters

Example map settings:

```python
resolution = 0.05   # meters per cell
width = 200         # number of columns
height = 200        # number of rows
origin_x = -5.0     # world x of map corner
origin_y = -5.0     # world y of map corner
```

This gives a map covering:

```text
10 m x 10 m
```

---

## Coordinate Conversion

To convert world coordinates `(x, y)` into grid indices `(row, col)`:

```python
col = int((x - origin_x) / resolution)
row = int((y - origin_y) / resolution)
```

---

## Beam Endpoint

If:

* robot pose is `(x_r, y_r, theta)`
* beam angle in robot frame is `phi`
* beam range is `r`

then:

```python
global_angle = theta + phi
x_end = x_r + r * math.cos(global_angle)
y_end = y_r + r * math.sin(global_angle)
```

---

## Ray Tracing

We need all cells between:

* robot cell
* beam end cell

A simple way is to use **Bresenham’s line algorithm**.

Then:

* all cells except the last one → free
* last cell → occupied

---

## Complete ROS 2 Python Solution

```python
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose


def quaternion_to_yaw(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OccupancyGridMapper(Node):
    def __init__(self):
        super().__init__('occupancy_grid_mapper')

        # Map parameters
        self.resolution = 0.05
        self.width = 200
        self.height = 200
        self.origin_x = -5.0
        self.origin_y = -5.0

        # Internal map: start with unknown
        self.grid = [-1] * (self.width * self.height)

        # Latest robot pose
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Timer for publishing map
        self.timer = self.create_timer(0.5, self.publish_map)

        self.get_logger().info('Occupancy grid mapper started.')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def scan_callback(self, msg: LaserScan):
        if self.robot_x is None:
            return

        angle = msg.angle_min

        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            if r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            global_angle = self.robot_yaw + angle

            x_end = self.robot_x + r * math.cos(global_angle)
            y_end = self.robot_y + r * math.sin(global_angle)

            start_cell = self.world_to_grid(self.robot_x, self.robot_y)
            end_cell = self.world_to_grid(x_end, y_end)

            if start_cell is None or end_cell is None:
                angle += msg.angle_increment
                continue

            cells = self.bresenham(
                start_cell[1], start_cell[0],
                end_cell[1], end_cell[0]
            )

            if len(cells) == 0:
                angle += msg.angle_increment
                continue

            # Mark free cells along the ray, excluding last cell
            for row, col in cells[:-1]:
                self.set_cell(row, col, 0)

            # Mark endpoint occupied
            self.set_cell(cells[-1][0], cells[-1][1], 100)

            angle += msg.angle_increment

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)

        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return None

        return (row, col)

    def set_cell(self, row, col, value):
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return

        idx = row * self.width + col

        # Simple rule:
        # occupied has priority over free
        if value == 100:
            self.grid[idx] = 100
        elif value == 0 and self.grid[idx] != 100:
            self.grid[idx] = 0

    def bresenham(self, x0, y0, x1, y1):
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            cells.append((y, x))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## How the Code Works

### 1. Odometry callback

Stores the latest robot pose:

* `x`
* `y`
* `yaw`

### 2. Scan callback

For every valid LiDAR range:

* computes the beam endpoint
* converts start and end into grid cells
* finds all crossed cells using Bresenham
* marks intermediate cells as free
* marks the last cell as occupied

### 3. Map publisher

Publishes the current grid as a ROS occupancy grid.

---

## Important Note About Frame Choice

In this example, the map is published in the `odom` frame:

```python
msg.header.frame_id = 'odom'
```

This is acceptable here because the exercise assumes odometry gives the robot pose directly in the same world used for mapping.

---

## Running the Node

After adding the file to your ROS 2 Python package:

```bash
chmod +x occupancy_grid_mapper.py
```

Then run:

```bash
ros2 run your_package_name occupancy_grid_mapper
```

---

## RViz

In RViz, add:

* `Map`
* topic: `/map`
* fixed frame: `odom`

You should see:

* walls becoming occupied
* empty space becoming free
* unexplored zones remaining unknown

---

## Limitations of This Simple Solution

This is a basic classroom solution.

Limitations:

* no probabilistic update
* no log-odds
* free cells can be overwritten many times
* if odometry drifts, the map will distort
* dynamic obstacles are not handled

---

## Better Version for Discussion

A better implementation would store a floating-point belief per cell using log-odds:

```text
L(cell) = L(cell) + L_free
L(cell) = L(cell) + L_occ
```

Then convert to ROS values only during publishing.

But for class, the above direct version is simple and very clear.

---

## In-Class Summary

### Inputs

* LiDAR scan
* robot odometry

### Process

* convert scan beam to endpoint
* trace cells along the beam
* set free cells
* set occupied endpoint

### Output

* publish occupancy grid map

---

## Very Short Pseudocode

```text
for each scan beam:
    get range r and angle phi
    compute endpoint in world
    convert robot and endpoint to grid cells
    trace line between them
    mark line cells as free
    mark final cell as occupied
publish map
```

---

## Suggested In-Class Extension

After showing this basic solution, a very good extension is:

* replace direct assignment with log-odds update
* compare the quality of both maps
