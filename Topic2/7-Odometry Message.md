# ROS 2 ↔ Gazebo (gz) Bridging: `/cmd_vel` and Odometry

---

## 1. Odometry Message

### 1.1 In ROS 2
- **Type:** `nav_msgs/msg/Odometry`  
- **Purpose:** Provides robot position and velocity estimates.

**Main fields:**
- `header` → timestamp + frame id  
- `pose` → position (x, y, z) + orientation (quaternion)  
- `twist` → linear and angular velocity  

### 1.2 In Gazebo (gz)
- **Type:** `gz.msgs.Odometry`  
- Contains:
  - `pose` → position + orientation  
  - `twist` → linear + angular velocity  
- Gazebo publishes it on topics like:
  - `/model/vehicle_blue/odometry`

---

## 2. Working with Topics in Gazebo (gz)

### 2.1 List all Gazebo topics
```bash
gz topic -l
```

### 2.2 Echo (see the data) of a Gazebo topic
```bash
gz topic -e -t /model/vehicle_blue/odometry
```

This will continuously print the odometry data published by Gazebo.

---

## 3. Create the Bridges (Gazebo ↔ ROS 2)

Use `ros_gz_bridge` to connect Gazebo and ROS 2.

### 3.1 Bridge command format
```bash
ros2 run ros_gz_bridge parameter_bridge <topic>@<ros2_msg_type>@<gz_msg_type>
```

### 3.2 Bridge odometry + cmd_vel (example)
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

---

## 4. Check ROS 2 Topics

Verify that the topics are visible in ROS 2:

```bash
ros2 topic list
```

Expected (at least):
- `/model/vehicle_blue/odometry`
- `/cmd_vel`

---

## 5. Odometry Topic (`/model/<robot_name>/odometry`)

- **Gazebo type:** `gz.msgs.Odometry`  
- **ROS 2 type:** `nav_msgs/msg/Odometry`

### 6.1 ROS 2 message structure (reference)
```yaml
nav_msgs/msg/Odometry:
  header:                     # Standard ROS 2 message header
    stamp:                    # Time
      sec: int32
      nanosec: uint32
    frame_id: string          # Reference frame (e.g., "odom")
  child_frame_id: string      # Robot base frame (e.g., "base_link")

  pose:                       # PoseWithCovariance
    pose:                     # geometry_msgs/Pose
      position:               # geometry_msgs/Point
        x: float64            # Position in meters (world frame)
        y: float64
        z: float64
      orientation:            # geometry_msgs/Quaternion
        x: float64            # Orientation (quaternion form)
        y: float64
        z: float64
        w: float64
    covariance: [float64[36]] # 6x6 covariance matrix

  twist:                      # TwistWithCovariance
    twist:                    # geometry_msgs/Twist
      linear:                 # geometry_msgs/Vector3
        x: float64            # Linear velocity in m/s
        y: float64
        z: float64
      angular:                # geometry_msgs/Vector3
        x: float64            # Angular velocity (rad/s)
        y: float64
        z: float64
    covariance: [float64[36]] # 6x6 covariance matrix
```

### 6.2 What it contains (typical)
- **Header**
  - `frame_id: vehicle_blue/odom` → Odometry reference frame  
  - `child_frame_id: vehicle_blue/chassis` → The robot base frame  
- **Twist:** robot’s instantaneous velocity (robot frame)  
- **Pose:** robot’s position/orientation (world frame)  
  - Pose ≈ integrated twist over time  
  - Errors accumulate in real robots; simulation is exact unless noise added

### 6.3 View odometry in ROS 2
```bash
ros2 topic echo /model/vehicle_blue/odometry
```

---

## 7. Summary

- **`/cmd_vel`**: command you send to the robot (robot frame).  
- **Odometry `twist`**: robot’s actual measured velocity.  
- **Odometry `pose`**: integrated position/orientation in the world frame.  
