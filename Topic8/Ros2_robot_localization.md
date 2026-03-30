<img width="1244" height="644" alt="image" src="https://github.com/user-attachments/assets/43caea25-2668-44bb-aab0-5e4f12b51177" />

# https://docs.ros.org/en/noetic/api/robot_localization/html/index.html

# EKF Localization in ROS 2 + Gazebo Sim

This tutorial shows how to apply **EKF localization** to your differential-drive robot using:

* **wheel odometry** from the Gazebo diff-drive plugin
* **IMU** from a Gazebo IMU sensor
* **`robot_localization`** in ROS 2

---

## What we want to achieve

We want this pipeline:

Gazebo robot  →  odometry + IMU  →  ROS 2 bridge  →  `robot_localization` EKF  →  `/odometry/filtered`

The EKF will combine:

* odometry = good short-term motion estimate
* IMU = good angular motion estimate

This gives a smoother and more realistic robot pose estimate than odometry alone.

---

## 1) Install the required ROS 2 packages

For ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install ros-jazzy-robot-localization
```
---

## 2) Edit your SDF file

Your robot SDF already has a diff-drive plugin.

Two things should be added or adjusted:

1. add the **IMU world system plugin**
2. add an **IMU sensor** inside the `chassis` link
3. increase odometry publish rate from **1 Hz** to something better like **30 Hz**

---

## 3) Add the IMU system plugin in the `<world>`

In your world, add this plugin near the other world plugins:

```xml
<plugin
    filename="gz-sim-imu-system"
    name="gz::sim::systems::Imu">
</plugin>
```
---

## 4) Add the IMU sensor inside the `chassis` link

Inside this link:

```xml
<link name='chassis'>
```

add this IMU sensor block before `</link>`:

```xml
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>50</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```

---

## 5) Increase the odometry rate

In your current diff-drive plugin, you have:

```xml
<odom_publish_frequency>1</odom_publish_frequency>
```

Change it to:

```xml
<odom_publish_frequency>30</odom_publish_frequency>
```

Why?

* `1 Hz` is too slow for localization
* `30 Hz` is much better for EKF demos
* IMU at `50 Hz` and odometry at `30 Hz` is a good simple combination

---

## 6) Save and run Gazebo

Example:

```bash
gz sim /full/path/to/your_robot_world.sdf
```

---

## 7) Check Gazebo topics

In another terminal:

```bash
gz topic -l
```

You want to find:

* the IMU topic, likely:

```bash
/imu
```

* the odometry topic, likely something like:

```bash
/model/vehicle_blue/odometry
```

Check both:

```bash
gz topic -e -t /imu
```

```bash
gz topic -e -t /model/vehicle_blue/odometry
```

If the odometry topic name is slightly different on your machine, use the exact name from `gz topic -l`.

---

## 8) Bridge Gazebo topics to ROS 2

Open a new terminal and bridge the topics.

### Bridge the IMU

```bash
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
```

### Bridge the odometry

```bash
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry
```

### Optional: bridge the clock

```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```

Now check from ROS 2:

```bash
ros2 topic list
```

And inspect:

```bash
ros2 topic echo /imu
```

```bash
ros2 topic echo /model/vehicle_blue/odometry
```

---

## 9) Create the EKF config file

Create a file called `ekf.yaml`.

Example:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0                # EKF output/update rate in Hz
    sensor_timeout: 0.1            # if a sensor is silent for > 0.1 s, EKF keeps predicting without it
    two_d_mode: true               # force planar motion: ignore z, roll, pitch, etc.
    publish_tf: true               # publish TF transform from odom -> base frame
    print_diagnostics: true        # print useful diagnostics while debugging

    map_frame: map                 # global map frame name
    odom_frame: odom               # local odometry frame name
    base_link_frame: vehicle_blue/chassis   # robot body frame (must match your TF/frame naming)
    world_frame: odom              # use odom as the main world frame for local EKF

    odom0: /model/vehicle_blue/odometry     # wheel odometry topic
    imu0: /imu                              # IMU topic

    # State order used by robot_localization:
    # [x, y, z,
    #  roll, pitch, yaw,
    #  vx, vy, vz,
    #  vroll, vpitch, vyaw,
    #  ax, ay, az]

    odom0_config: [
      false, false, false,   # x, y, z       -> do NOT use odom position
      false, false, false,   # roll, pitch, yaw -> do NOT use odom orientation
      true,  true,  false,   # vx, vy, vz    -> use linear velocity x and y, ignore z
      false, false, true,    # vroll, vpitch, vyaw -> use yaw rate only
      false, false, false    # ax, ay, az    -> do NOT use odom accelerations
    ]

    imu0_config: [
      false, false, false,   # x, y, z       -> IMU does not provide position
      false, false, true,    # roll, pitch, yaw -> use yaw from IMU
      false, false, false,   # vx, vy, vz    -> IMU does not provide linear velocity
      false, false, true,    # vroll, vpitch, vyaw -> use yaw rate from IMU
      false, false, false    # ax, ay, az    -> do NOT use linear acceleration here
    ]

    odom0_differential: false      # use odometry values directly, not as changes between samples
    odom0_relative: false          # odometry is not treated as relative-to-first-measurement

    imu0_differential: false       # use IMU yaw directly, not as delta yaw
    imu0_relative: false           # IMU is not treated as relative-to-first-measurement
    imu0_remove_gravitational_acceleration: true  # remove gravity if IMU acceleration is fused
```

---

## 10) Why these settings?

### `two_d_mode: true`

Your robot is moving on a flat plane, so we treat it as a 2D robot.

### `world_frame: odom`

This is the normal choice when doing local continuous localization from odometry and IMU.

### `base_link_frame: vehicle_blue/chassis`

Your robot link name is `chassis` inside the model `vehicle_blue`.

### `odom0`

This is your wheel odometry source.

### `imu0`

This is your IMU source.

### `odom0_config`

We mainly use odometry for:

* forward velocity `vx`
* sideways velocity `vy` if reported as zero
* yaw rate `vyaw`

### `imu0_config`

We mainly use IMU for:

* yaw
* yaw rate

This is a simple and clean Topic 8 example.

---

## 11) Run the EKF node

In a new terminal:

```bash
ros2 run robot_localization ekf_node --ros-args --params-file /full/path/to/ekf.yaml
```

Now check:

```bash
ros2 topic echo /odometry/filtered
```

This is your EKF output.

---

