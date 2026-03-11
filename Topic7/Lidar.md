<img width="962" height="581" alt="image" src="https://github.com/user-attachments/assets/6cd4dfd8-0bc1-4370-9e56-1530f8e42132" />


# Add a 2D LiDAR in Gazebo (gz-sim), Bridge to ROS 2, and Visualize in RViz


---

## 1) Add a LiDAR reference frame to the robot

Place this **under the `<model name="vehicle_blue">` tag** in your world or model SDF:

```xml
<frame name="lidar_frame" attached_to='chassis'>
  <pose>0.8 0 0.5 0 0 0</pose>
</frame>
```

- This creates a rigid frame named `lidar_frame` attached to `chassis`, located **0.8 m forward**, **0 m lateral**, **0.5 m up**, with **no rotation**.

---

## 2) Enable the Sensors system in the world

Put this **under `<world>`** (sibling of models) so Gazebo publishes sensor data:

```xml
<plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

---

## 3) Add the 2D GPU LiDAR under the chassis link

Add this **inside the `<link name="chassis">`** (or the link that should carry the sensor):

```xml
<sensor name='gpu_lidar' type='gpu_lidar'>
  <pose relative_to='lidar_frame'>0 0 0 0 0 0</pose>
  <topic>lidar</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.396263</min_angle>
        <max_angle>1.396263</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>0.01</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <visualize>true</visualize>
</sensor>
```

### What the parameters mean (and how to tune)

- **`<pose relative_to='lidar_frame'>…</pose>`**: Mount pose relative to the frame you added earlier.
- **`<topic>`**: Name of the Gazebo transport topic for the LiDAR output (`/lidar`).
- **`<update_rate>`**: Frequency in **Hz**. Here `10` → new scan every **0.1 s**.
- **`<horizontal>` / `<vertical>`**: Number of simulated rays and angular limits.
  - **`<samples>`**: number of rays per sweep (horizontal) / per column (vertical).
  - **`<resolution>`**: multiplied by `samples` to get the number of returned points.
  - **`<min_angle>` / `<max_angle>`**: angular span in radians (here ≈ ±80°).
- **`<range>`**: per-ray distance properties.
  - **`<min>` / `<max>`**: minimum/maximum return distance (m).
  - **`<resolution>`**: linear distance resolution.
- **`<always_on>`**: when `true`, runs at the given `update_rate` continuously.
- **`<visualize>`**: draws the sensor frustum/points in the GUI (**debug-only** cost).

**Common tuning tips**:
- Wider FoV → increase `min_angle`/`max_angle` span (e.g., ±135° is `±2.35619` rad).
- Higher angular density → increase `samples` (tradeoff with performance).
- Faster scans → increase `update_rate` (CPU/GPU cost goes up).
- Range fidelity → adjust `<range><resolution>` and `<max>` to your scene scale.

---

## 4) Run Gazebo and confirm topics

Start your world (SDF) as usual, then in a terminal:

```bash
gz topic -l
```

You should see entries like:
```
/lidar
/lidar/points
```
(`points` exists when the GPU LiDAR also publishes a point cloud).

---

## 5) Bridge LiDAR to ROS 2

Use the **parameter bridge** to convert Gazebo messages into ROS 2 messages:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```


Verify in ROS 2:

```bash
# list topics
ros2 topic list

# inspect some data
ros2 topic echo /lidar --once
ros2 topic echo /lidar/points --once
```

---

## 6) Visualize in RViz

Open **RViz** and add these displays:

**LaserScan**
- *Add → By display type →* **LaserScan**
- **Topic**: `/lidar`
- **Style**: *Points*
- **Size (Pixels)**: `2–4`
- **Color Transformer**: *Intensity* (if available) or *Flat Color*

**PointCloud2**
- *Add → By display type →* **PointCloud2**
- **Topic**: `/lidar/points`
- **Style**: *Points*
- **Size (m)**: `0.02–0.05`
- **Color Transformer**: *Intensity* (or *Z* / *RGB* if present)

> ⚠️ make sure you have already TF available and working so rviz will visualize

---

## 7) Add the previously given maze in the world around the robot


## 8) Create a node that will subscribe to the lidar points, and do a scattered plot, add a custom estimation plot for the points


