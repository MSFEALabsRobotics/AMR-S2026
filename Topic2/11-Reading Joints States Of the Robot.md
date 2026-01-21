# 🚗 Reading Joint States in Gazebo Sim & ROS 2

To publish joint states, you must add the **JointStatePublisher** plugin to your robot’s `.sdf`:

```xml
<plugin
  name="gz::sim::systems::JointStatePublisher"
  filename="gz-sim-joint-state-publisher-system">
  <publish_rate>50</publish_rate>
</plugin>
```

---

## 1. Echo Joint State in Gazebo

Use `gz topic` to view the raw joint state message:

```bash
gz topic -e -t /world/car_world/model/vehicle_blue/joint_state
```

This message contains all robot joints, including drive wheels and caster wheels.

---

## 2. Bridge Joint States to ROS 2 (UPDATED NOW WORKING)

Run the ROS ↔ Gazebo bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge   /world/car_world/model/vehicle_blue/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model
```

---

## 3. Echo in ROS 2

Now you can view the joint states inside ROS 2:

```bash
ros2 topic echo /world/car_world/model/vehicle_blue/joint_state
```

---

## 4. Key Fields (for wheel joints)

- **`axis1.position`** → wheel rotation angle (**radians**)  
- **`axis1.velocity`** → wheel angular speed (**rad/s**)  

Example (excerpt from Gazebo):

```yaml
joint {
  name: "left_wheel_joint"
  axis1 {
    position: 1260.65        # wheel rotation angle (rad)
    velocity: 2.50           # wheel angular speed (rad/s)
  }
}
```

---

## 5. Structure of `JointState` Message

In ROS 2 (`sensor_msgs/msg/JointState`), the message looks like:

```yaml
header:
  stamp: {sec: 123, nanosec: 456}
  frame_id: "world"
name: ["left_wheel_joint", "right_wheel_joint", "caster_wheel"]
position: [1260.65, 1259.80, 0.001]
velocity: [2.50, 2.48, 0.00]
effort:   [0.0, 0.0, 0.0]
```

---

✅ With this, you can read each wheel’s **rotation angle** (like encoder ticks) and **angular velocity** (rad/s) for odometry and control.
