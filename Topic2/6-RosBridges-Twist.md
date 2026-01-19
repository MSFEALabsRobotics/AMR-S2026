# 🚗 ROS 2 – Gazebo Bridges
---

## 1. List Gazebo Topics
Check what topics Gazebo is publishing:

```bash
gz topic -l
```

👉 Look for:
- **Velocity command** topic → e.g. `/cmd_vel`

---

## 2. Create the Bridges
Use `ros_gz_bridge` to connect Gazebo and ROS 2:
The parameter_bridge connects topics between Gazebo (gz) and ROS 2.
- Use the   **"ros2 run ros_gz_bridge parameter_bridge"**    Command
- ros2 run ros_gz_bridge parameter_bridge  <topic_name>@<ros2_msg_type>@<gz_msg_type>


```bash
ros2 run ros_gz_bridge parameter_bridge  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

---

## 3. Check ROS 2 Topics
Verify that topics are visible in ROS 2:

```bash
ros2 topic list
```

Expected:
- `/cmd_vel`

---

## 4. Velocity Command (`/cmd_vel`)
- **Gazebo type:** `gz.msgs.Twist`  
- **ROS 2 type:** `geometry_msgs/msg/Twist`

**Fields (diff drive):**
- `linear.x` → Forward/backward speed (m/s)  
- `angular.z` → Rotation around vertical axis (rad/s)  
- Other components usually `0`

**Frame: Robot Frame.**

**Example command:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist   "{linear: {x: 0.3}, angular: {z: 0.1}}" -r 10
```

