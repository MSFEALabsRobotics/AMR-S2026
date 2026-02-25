# Add a range sensor in Gazebo (Gazebo Sim / Ignition)

---

## 1) Add a LiDAR reference frame to the robot

Place this **under the `<model name="vehicle_blue">` tag** in your world or model SDF:

```xml
<frame name="lidar_frame" attached_to='chassis'>
  <pose>0.8 0 0.5 0 0 0</pose>
</frame>
```

- This creates a rigid frame named `lidar_frame` attached to `chassis`, located **0.8 m forward**, **0 m lateral**, **0.5 m up**, with **no rotation**.

---

## 2) Enable the Sensors system in the world

Put this **under `<world>`** (sibling of models) so Gazebo publishes sensor data:

```xml
<plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

---

## 3) Add the 2D GPU LiDAR under the chassis link

Add this **inside the `<link name="chassis">`** (or the link that should carry the sensor):

```xml
<sensor name='gpu_lidar' type='gpu_lidar'>
  <pose relative_to='lidar_frame'>0 0 0 0 0 0</pose>
  <topic>lidar</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>0.01</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <visualize>true</visualize>
</sensor>
```

---

## 4) Add a model for the walls

Add this **under `<world>`** (sibling of your robot model), to create a chain of wall segments:

```xml
<model name="side_wall">
  <static>true</static>

  <!-- A chain of short wall segments along X with varying Y -> not straight -->
  <!-- Each segment is a box: length in X, thickness in Y, height in Z -->

  <link name="seg_0">
    <pose>  2.0  2.10 0.75 0 0 0</pose>
    <collision name="col">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
      <material>
        <ambient>0.6 0.6 0.6 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
  </link>

  <link name="seg_1">
    <pose>  4.0  2.35 0.75 0 0 0</pose>
    <collision name="col">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </visual>
  </link>

  <link name="seg_2">
    <pose>  6.0  2.05 0.75 0 0 0</pose>
    <collision name="col">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </visual>
  </link>

  <link name="seg_3">
    <pose>  8.0  2.55 0.75 0 0 0</pose>
    <collision name="col">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </visual>
  </link>

  <link name="seg_4">
    <pose> 10.0  2.20 0.75 0 0 0</pose>
    <collision name="col">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </visual>
  </link>

  <link name="seg_5">
    <pose> 12.0  2.70 0.75 0 0 0</pose>
    <collision name="col">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>2.0 0.10 1.5</size></box></geometry>
    </visual>
  </link>

</model>
```

---

## 5) Test topics

```bash
gz topic -e -t /lidar
```

---

## 6) Bridge the LiDAR to ROS 2

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

Echo just the ranges:

```bash
ros2 topic echo /lidar --field ranges
```

---

## 7) Bridge Twist and Odometry

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

---

## 8) Drive the robot with ROS 2

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 9) Python file to test range

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.sub = self.create_subscription(LaserScan, '/lidar', self.cb, 10)

    def cb(self, msg: LaserScan):
        # If you configured samples=1:
        if msg.ranges:
            r = msg.ranges[0]
            print(r)
        else:
            self.get_logger().warn("No ranges in message")

def main():
    rclpy.init()
    node = LidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 10) Python file to do particles

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
import copy

import matplotlib.pyplot as plt

class ParticlePlotter:
    def __init__(self):
        plt.ion()                           # <-- non-blocking / interactive
        self.fig, self.ax = plt.subplots()
        self.scat = None
        self.ax.set_yticks([])
        self.ax.set_xlabel("x position")
        self.ax.set_title("Particles (dot size ~ weight)")

    def update(self, particles):
        xs = [p["xi"] for p in particles]
        sizes = [max(10.0, p["wi"] * 5000.0) for p in particles]

        if self.scat is None:
            self.scat = self.ax.scatter(xs, [0]*len(xs), s=sizes)
        else:
            # update positions and sizes
            self.scat.set_offsets([[x, 0] for x in xs])
            self.scat.set_sizes(sizes)

        self.ax.set_xlim(0, 20)
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

class RangeOdomPrinter(Node):
    def __init__(self):
        super().__init__('range_odom_printer')
        self.range = 0      # range (m)
        self.odom = 0       # odom x (m)

        self.plotter = ParticlePlotter()

        self.create_subscription(LaserScan, '/lidar', self.scan_cb, 10)
        self.create_subscription(Odometry,  '/model/vehicle_blue/odometry',  self.odom_cb, 10)

        ###### INITIALIZE FILTER

        self.N = 5
        self.xmin, self.xmax = 0.0, 20.0

        self.particles = []

        for i in range(self.N):
            xi = random.uniform(self.xmin, self.xmax)  # particle position
            wi = 1.0 / self.N                          # particle weight
            self.particles.append({"xi": xi, "wi": wi})

        self.start = copy.deepcopy(self.particles)

        # MAIN LOOP AT 5hz
        self.create_timer(0.01, self.mainloop)

    def scan_cb(self, msg: LaserScan):
        if msg.ranges:
            self.range = round(msg.ranges[0], 3)

    def odom_cb(self, msg: Odometry):
        self.odom = round(msg.pose.pose.position.x, 3)

    def mainloop(self):
        print(self.odom, self.range)

        # later in your loop
        for p, p0 in zip(self.particles, self.start):
            p["xi"] = p0["xi"] + self.odom
            p["wi"] = 0.2 * self.odom

        self.plotter.update(self.particles)

        #######

def main():
    rclpy.init()
    node = RangeOdomPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
