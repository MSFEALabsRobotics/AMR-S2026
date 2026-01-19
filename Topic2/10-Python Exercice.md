# Exercise: Python `time` Library + ROS 2 Timed Motion Publisher

This exercise has two parts:

1) Learn the basics of Python’s `time` library (measuring elapsed time).  
2) Write a ROS 2 publisher node that commands a robot:
   - Move straight for the first **5 seconds**
   - Rotate in place for the next **5 seconds**
   - Then **stop**

---

## Part A — Mini Tutorial: Python `time`

### Example: Measure elapsed time

```python
import time

# Get the start time (in seconds since epoch)
start_time = time.time()
print("Start time:", start_time)

# Do something (simulate work with sleep)
time.sleep(2)   # wait 2 seconds

# Get current time again
current_time = time.time()
print("Current time:", current_time)

# Calculate elapsed time
elapsed = current_time - start_time
print("Elapsed time:", elapsed, "seconds")

# Compare elapsed time
if elapsed < 5:
    print("Less than 5 seconds passed")
else:
    print("5 or more seconds passed")
```

### Notes
- `time.time()` returns seconds as a floating number (e.g., `1737301234.123`).
- `time.sleep(x)` pauses the program for **x seconds**.
- `elapsed = time.time() - start_time` is the most common pattern for timing.

---

## Part B — ROS 2 Exercise: Timed `/cmd_vel` Publisher

### Goal
Write a ROS 2 node that publishes `geometry_msgs/msg/Twist` on `/cmd_vel`:

- **0 to 5 seconds:** move forward  
  - `linear.x = 0.3` (example)  
  - `angular.z = 0.0`

- **5 to 10 seconds:** rotate in place  
  - `linear.x = 0.0`  
  - `angular.z = 0.6` (example)

- **After 10 seconds:** stop  
  - `linear.x = 0.0`  
  - `angular.z = 0.0`

---

## Starter Code (Python, rclpy)

> Save as: `timed_cmdvel_publisher.py`

```python
#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TimedCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('timed_cmdvel_publisher')

        # Publisher to /cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timing
        self.start_time = time.time()

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Timed /cmd_vel publisher started.")

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        msg = Twist()

        # Phase 1: straight for 5 seconds
        if elapsed < 5.0:
            msg.linear.x = 0.3
            msg.angular.z = 0.0

        # Phase 2: rotate for the next 5 seconds
        elif elapsed < 10.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.6

        # Phase 3: stop
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub.publish(msg)

        # Optional: print phase info occasionally
        # self.get_logger().info(f"Elapsed: {elapsed:.2f}s")

def main(args=None):
    rclpy.init(args=args)
    node = TimedCmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## How to Run (quick method)

1) Make the script executable:
```bash
chmod +x timed_cmdvel_publisher.py
```

2) Run it:
```bash
ros2 run <your_package_name> timed_cmdvel_publisher
```

> If you are not using a package yet, you can run directly with:
```bash
python3 timed_cmdvel_publisher.py
```
(As long as your ROS 2 environment is sourced.)

---

## Checklist
- [ ] The robot moves forward for ~5 seconds  
- [ ] Then rotates for ~5 seconds  
- [ ] Then stops and stays stopped  
- [ ] `/cmd_vel` shows changing values when you `ros2 topic echo /cmd_vel`  
