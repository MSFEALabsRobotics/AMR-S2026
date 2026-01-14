# 📄 ROS 2 – Gazebo Messages and Driving the Robot

## 1. Twist Message

### In ROS 2
- **Type:** `geometry_msgs/msg/Twist`
- **Purpose:** Represents velocity commands.
- **Fields:**
  - `linear` → (x, y, z) linear velocity in m/s
  - `angular` → (x, y, z) angular velocity in rad/s

Example (move forward with 0.5 m/s and turn with 0.2 rad/s):
```yaml
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2
```

### In Gazebo (gz)
- **Type:** `gz.msgs.Twist`
- Contains the same structure: `linear` and `angular` vectors.
- Used by Gazebo plugins (like diff_drive) to move robots.

---

## 2. Odometry Message

### In ROS 2
- **Type:** `nav_msgs/msg/Odometry`
- **Purpose:** Provides robot position and velocity estimates.
- **Fields:**
  - `header` → timestamp + frame id
  - `pose` → position (x, y, z) + orientation (quaternion)
  - `twist` → linear and angular velocity

### In Gazebo (gz)
- **Type:** `gz.msgs.Odometry`
- Contains:
  - `pose` → position + orientation
  - `twist` → linear + angular velocity
- Gazebo publishes it on topics like `/model/vehicle_blue/odometry`.

---

## 3. Working with Topics in Gazebo

### List all topics
```bash
gz topic -l
```

### Echo (see the data) of a topic
```bash
gz topic -e -t /model/vehicle_blue/odometry
```

This will continuously print the odometry data published by Gazebo.

---

## 4. Drive the Robot from Gazebo Terminal

You can publish directly to the velocity command topic in Gazebo.

### Example: Move forward
```bash
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5}"
```

### Example: Turn in place
```bash
gz topic -t /cmd_vel -m gz.msgs.Twist -p "angular: {z: 0.3}"
```

### Example: Move forward and turn
```bash
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.2}"
```

---

✅ With these commands, you can list topics, view their data, and drive your robot directly from Gazebo without using ROS 2.
