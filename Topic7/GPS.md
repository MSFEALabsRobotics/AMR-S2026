# GZ Sim GPS Sensor + ROS 2 Bridge Tutorial

## Goal

Add a GPS sensor to a simple robot in **GZ Sim**, verify that Gazebo publishes the GPS topic, then bridge it to **ROS 2** as a `NavSatFix` message.

---

## 1) Add the NavSat system to the world

Inside your `<world>` tag, add:

```xml
<plugin
    filename="gz-sim-navsat-system"
    name="gz::sim::systems::NavSat">
</plugin>

<spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <latitude_deg>33.8938</latitude_deg>
    <longitude_deg>35.5018</longitude_deg>
    <elevation>0</elevation>
    <heading_deg>0</heading_deg>
</spherical_coordinates>
```

### What this does

* The **NavSat system** enables GPS-like sensors in the world.
* The **spherical coordinates** define the world origin as a real Earth location.
* In this example, the origin is set near **Beirut**.

---

## 2) Add the GPS sensor to the robot

Inside the robot link where you want the GPS mounted, for example inside:

```xml
<link name='chassis'>
```

add:

```xml
<sensor name="gps_sensor" type="navsat">
    <always_on>true</always_on>
    <update_rate>5</update_rate>
    <topic>/gps</topic>
    <pose>0 0 0 0 0 0</pose>
</sensor>
```

### What this does

* Creates a GPS sensor named `gps_sensor`
* Publishes on topic `/gps`
* Updates at **5 Hz**
* Places the GPS at the origin of the chassis link

---

## 3) Run the world in Gazebo

Launch your world normally:

```bash
gz sim your_world.sdf
```

---

## 4) Check that Gazebo publishes the GPS topic

List Gazebo topics:

```bash
gz topic -l
```

You should see:

```bash
/gps
```

Then echo the GPS topic:

```bash
gz topic -e -t /gps
```

Example output:

```text
header {
  stamp {
    nsec: 1000000
  }
  data {
    key: "seq"
    value: "0"
  }
}
latitude_deg: 33.893800000006145
longitude_deg: 35.501805405446781
altitude: 0.39999680407345295
velocity_east: 0.001677640737272279
velocity_north: 0.00069467232298275634
velocity_up: -0.0032165840263241709
frame_id: "vehicle_blue::chassis::gps_sensor"
```

### Notes

* `latitude_deg` and `longitude_deg` are the simulated GPS coordinates.
* `altitude` depends on the height of the sensor above the world.
* Small velocity values may appear even when the robot looks still.

---

## 5) Bridge GPS from Gazebo to ROS 2

Use `ros_gz_bridge`:

```bash
ros2 run ros_gz_bridge parameter_bridge /gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat
```

This creates a bridge from:

* **Gazebo topic**: `/gps`
* **Gazebo type**: `gz.msgs.NavSat`
* **ROS 2 type**: `sensor_msgs/msg/NavSatFix`

---

## 6) Check the GPS topic in ROS 2

In another terminal:

```bash
ros2 topic echo /gps
```

Now you should receive a standard ROS 2 GPS message.

---


---
