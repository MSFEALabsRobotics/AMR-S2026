# Topic 8 Tutorial — EKF Localization in ROS 2 + Gazebo Sim

This tutorial shows how to apply **EKF localization** to your differential-drive robot using:

* **wheel odometry** from the Gazebo diff-drive plugin
* **IMU** from a Gazebo IMU sensor
* **`robot_localization`** in ROS 2

It is written specifically for your robot model:

* model name: `vehicle_blue`
* main body link: `chassis`
* diff-drive plugin already present in the SDF

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
sudo apt install ros-jazzy-robot-localization ros-jazzy-ros-gz-bridge
```

Check installation:

```bash
ros2 pkg list | grep robot_localization
ros2 pkg list | grep ros_gz_bridge
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

So the top plugin section becomes like this:

```xml
<plugin
    filename="gz-sim-physics-system"
    name="gz::sim::systems::Physics">
</plugin>
<plugin
    filename="gz-sim-user-commands-system"
    name="gz::sim::systems::UserCommands">
</plugin>
<plugin
    filename="gz-sim-scene-broadcaster-system"
    name="gz::sim::systems::SceneBroadcaster">
</plugin>
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

A good result is:

```xml
<link name='chassis'>
    <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
    <inertial>
        <mass>1.14395</mass>
        <inertia>
            <ixx>0.095329</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.381317</iyy>
            <iyz>0</iyz>
            <izz>0.476646</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <box>
                <size>2.0 1.0 0.5</size>
            </box>
        </geometry>
        <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
            <specular>0.0 0.0 1.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <box>
                <size>2.0 1.0 0.5</size>
            </box>
        </geometry>
    </collision>

    <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <visualize>true</visualize>
        <topic>imu</topic>
    </sensor>
</link>
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

So the plugin becomes:

```xml
<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>30</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
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
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    print_diagnostics: true

    map_frame: map
    odom_frame: odom
    base_link_frame: vehicle_blue/chassis
    world_frame: odom

    odom0: /model/vehicle_blue/odometry
    imu0: /imu

    # state order:
    # [x, y, z,
    #  roll, pitch, yaw,
    #  vx, vy, vz,
    #  vroll, vpitch, vyaw,
    #  ax, ay, az]

    # Use wheel odometry mainly for body velocities
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]

    # Use IMU mainly for heading and yaw rate
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]

    odom0_differential: false
    odom0_relative: false

    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true
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

## 12) Check TF

You can inspect the TF tree:

```bash
ros2 run tf2_tools view_frames
```

Or check if the transform exists:

```bash
ros2 run tf2_ros tf2_echo odom vehicle_blue/chassis
```

If your TF setup uses `base_link` instead of `vehicle_blue/chassis`, adjust `base_link_frame` in the YAML.

---

## 13) Simple test procedure

1. start Gazebo
2. verify `/imu` and odometry in Gazebo
3. bridge both topics to ROS 2
4. run the EKF
5. drive the robot using `cmd_vel`
6. compare:

   * raw odometry
   * filtered odometry

Useful commands:

```bash
ros2 topic echo /model/vehicle_blue/odometry
```

```bash
ros2 topic echo /odometry/filtered
```

You should see the filtered estimate become smoother, especially in heading behavior.

---

## 14) Command velocity test

If your robot already listens to `cmd_vel`, you can test with:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}" -r 10
```

If your ROS 2 command topic is bridged differently, use your existing topic setup.

---

## 15) Important notes

### A) Topic names may vary

The exact odometry topic can vary slightly.
Always confirm with:

```bash
gz topic -l
```

### B) IMU frame and TF matter

For more advanced use, make sure the IMU frame is consistent with your robot frame.
For this simple tutorial, the goal is just to get a working EKF example.

### C) Only one node should publish the same TF

Avoid having two different nodes publishing the same `odom -> base_link` transform.

### D) Start simple

First make sure each part works alone:

* Gazebo IMU works
* Gazebo odometry works
* bridges work
* EKF works

Then combine everything.

---

## 16) Minimal summary

What you changed in the SDF:

* added world IMU plugin
* added IMU sensor in `chassis`
* changed odometry rate from `1` to `30`

What you run in ROS 2:

* bridge IMU
* bridge odometry
* run `robot_localization` EKF

Main output:

```bash
/odometry/filtered
```

---

## 17) Good Topic 8 explanation for students

This example demonstrates the two EKF localization steps:

* **prediction step**: use wheel odometry to predict robot motion
* **correction step**: use IMU to correct the state estimate

So the robot does not rely on only one sensor.
Instead, it fuses multiple noisy sources into one better estimate.

---

## 18) Suggested next step

After this works, the next improvement is:

* add a LiDAR or camera landmark source
* compare:

  * odometry only
  * odometry + IMU
  * odometry + IMU + landmark correction

That makes the Topic 8 lecture much stronger.
