# Add an IMU in Gazebo Sim and Bridge It to ROS 2


---

## 1) Enable the IMU system in your **world**
Open your world SDF (e.g., `my_world.sdf`) and add the IMU system plugin **alongside your other world-level plugins**:

```xml
<plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu">
</plugin>
```

> Put this under `<world>` (where the rest of your world plugins live).

---

## 2) Add an IMU sensor to your robot
Edit your robot’s SDF/SDFormat (URDF converted to SDF also works) and add the `<sensor>` block to the **chassis link** (or whichever link you mount the IMU on). For example:

```xml
<link name="chassis">
  <!-- your visuals / collisions / inertials here -->

  <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>1</update_rate>      <!-- Hz -->
      <visualize>true</visualize>       <!-- optional: show axis in GUI -->
      <topic>imu</topic>                <!-- Gazebo topic name => /imu -->
  </sensor>
</link>
```

> Notes
> - `update_rate` is in Hz; adjust to your needs (e.g., 50–200 Hz for controllers, 1 Hz if you just want to test).
> - `topic` may be set to `imu` (as shown). In Gazebo it will appear as `/imu`.
> - You can position the sensor with a `<pose>` tag if needed.

---

## 3) Launch Gazebo and verify the IMU topic
With your world and robot loaded, list and inspect Gazebo topics:

```bash
gz topic -l
gz topic -e -t /imu
```

You should see messages flowing on `/imu` with type `gz.msgs.IMU`.

---

## 4) Bridge the IMU to ROS 2
Run the parameter bridge so Gazebo’s `/imu` becomes a native ROS 2 topic of type `sensor_msgs/msg/Imu`:

```bash
ros2 run ros_gz_bridge parameter_bridge "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
```

Now confirm from ROS 2:

```bash
ros2 topic list | grep /imu
ros2 topic echo /imu
```

> Tip: If your downstream ROS nodes use simulation time, also bridge `/clock` and set `use_sim_time` accordingly.

---

## 5) What’s inside a ROS 2 `sensor_msgs/Imu`?
The IMU message contains three grouped measurements, all expressed in the IMU’s coordinate frame:

- **`header`**
  - `stamp`: time of the measurement (use sim time if bridged)
  - `frame_id`: TF frame for the IMU (e.g., `base_imu`)

- **`orientation`** (`geometry_msgs/Quaternion`)
  - Unit quaternion `(x, y, z, w)` representing rotation from the IMU frame to the world (or the chosen reference).  
  - **`orientation_covariance[9]`**: Row-major covariance matrix. Use `-1` to indicate “orientation not provided”.

- **`angular_velocity`** (`geometry_msgs/Vector3`)
  - Angular rate in **rad/s** about the IMU frame axes.  
  - **`angular_velocity_covariance[9]`**: Covariance of the angular velocity.

- **`linear_acceleration`** (`geometry_msgs/Vector3`)
  - Proper acceleration in **m/s²** along the IMU frame axes (often includes gravity if not explicitly removed by the driver/simulator).  
  - **`linear_acceleration_covariance[9]`**: Covariance of the linear acceleration.


