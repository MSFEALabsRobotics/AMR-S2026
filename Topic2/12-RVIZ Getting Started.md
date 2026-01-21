# 🚗 RVIZ Tutorial

This tutorial shows how to run a differential drive robot in **Gazebo Sim**, bridge the topics into **ROS 2**, and visualize everything in **RViz 2**.  

---

## 1. Start your robot in Gazebo
Make sure you have a robot `.sdf` file (example: `vehicle_blue.sdf`) and run:

```bash
gz sim vehicle_blue.sdf
```

---

## 2. Start ROS ↔ Gazebo Bridges
Bridge **odometry** and **velocity commands**:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

---

## 3. BRIDGE /TF FROM GAZEBO TO ROS 
Gazebo → ROS 2, Bridge the TF transform.


```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  --ros-args -r /model/vehicle_blue/tf:=/tf
```
---

## 4. Check Available ROS 2 Topics
List all topics:

```bash
ros2 topic list
```

Check that `/tf` is being published:

```bash
ros2 topic echo /tf
```

---

## 5. Start RViz2
Launch RViz:

```bash
rviz2 rviz2
```

### Configure RViz:
1. **Fixed Frame** → set to `odom` (from odometry frame).  
2. Add display: **Odometry** → set topic to `/model/vehicle_blue/odometry`.  
3. Add display: **TF** → visualize the transform tree (`odom` → `chassis`).  

You should now see your robot’s odometry and TF frames updating live.

---

✅ Done! You now have Gazebo ↔ ROS 2 odometry + velocity bridges, a manual TF broadcaster, and visualization in RViz2.
