## Euler to Quaternions
https://articulatedrobotics.xyz/tools/rotation-calculator/

## Compare quaternions and euler
https://quaternions.online/



# RViz2 Particles Display with `PoseArray` (ROS 2)

This mini tutorial shows how to **visualize a particle cloud in RViz2 without any sensors** by publishing a `geometry_msgs/PoseArray`.

---

## 1) What is `PoseArray`?

**Message type:** `geometry_msgs/msg/PoseArray`

A `PoseArray` is simply:
- **Header** (`stamp`, `frame_id`) → *when* the data was published and *which TF frame* it belongs to (example: `"map"`).
- **poses[]** → a list of `Pose` messages.

Each **Pose** contains:
- `position` → `x, y, z` (meters)
- `orientation` → quaternion (`x, y, z, w`) for rotation

In particle filters, **each particle is one Pose**, and the whole particle set is a `PoseArray`.

---

## 2) Python node: publish random particles

Save as `particles_pub.py`:

```python
#!/usr/bin/env python3
import math, random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import quaternion_from_euler

class ParticlesPublisher(Node):
    def __init__(self):
        super().__init__("particles_publisher")
        self.pub = self.create_publisher(PoseArray, "/particles", 10)
        self.create_timer(0.1, self.publish_particles)  # 10 Hz

    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for _ in range(200):  # num_particles
            p = Pose()
            p.position.x = random.uniform(-5.0, 5.0)  # xmin, xmax
            p.position.y = random.uniform(-5.0, 5.0)  # ymin, ymax

            q = quaternion_from_euler(0.0, 0.0, random.uniform(-math.pi, math.pi))
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
            msg.poses.append(p)

        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ParticlesPublisher())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## 3) Run

Install dependency (if needed):
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-tf-transformations
```

Run the node:
```bash
python3 particles_pub.py
```

Check the topic:
```bash
ros2 topic echo /particles --once
```

---

## 4) Show in RViz2

1. Open RViz2:
   ```bash
   rviz2
   ```
2. Set **Fixed Frame** to: `map`
3. **Add** → **PoseArray**
4. Set **Topic** to: `/particles`

You should see a cloud of arrows (particles) inside the rectangle **x ∈ [-5, 5], y ∈ [-5, 5]**.

---

### Tip (common issue)
If RViz shows **No transform from [map]**, you need a TF frame named `map` (or change `frame_id` to an existing frame).
