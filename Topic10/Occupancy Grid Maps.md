# Occupancy Grid Mapping in ROS 2

## Exercise Title

Occupancy Grid Mapping Using LiDAR and Odometry in ROS 2

## Objective

In this exercise, you will build a **2D occupancy grid map** of an environment using:

* **LiDAR measurements**
* **Robot odometry**

You are **not required to use Nav2**.

The robot pose is assumed to be known from odometry, and your task is to construct a map showing:

* **occupied cells**
* **free cells**
* **unknown cells**

---

## Background

An occupancy grid map divides the environment into a rectangular grid of small cells.

Each cell stores information about whether that part of the environment is:

* occupied by an obstacle
* free space
* still unknown

This is one of the most common map representations in mobile robotics.

In this exercise:

* the robot moves in Gazebo
* LiDAR provides range measurements
* odometry provides the robot pose
* your node updates a grid map and publishes it as a ROS 2 topic

---

## ROS 2 Topics

You may assume the simulator already provides:

* `/scan` of type `sensor_msgs/msg/LaserScan`
* `/odom` of type `nav_msgs/msg/Odometry`

Your node should publish:

* `/map` of type `nav_msgs/msg/OccupancyGrid`

---

## What Is an Occupancy Grid Map?

A 2D occupancy grid is a matrix.

Each cell corresponds to a small square region in the world.

For example, if:

* map resolution = `0.05 m/cell`
* map width = `200 cells`
* map height = `200 cells`

then the represented physical area is:

* `10 m x 10 m`

Each cell can be stored as:

* `-1` = unknown
* `0` = free
* `100` = occupied

This is the convention used by `nav_msgs/OccupancyGrid`.

---

## Main Idea of the Algorithm

For each LiDAR beam:

1. get the robot pose from odometry
2. compute the beam angle in the world frame
3. compute the beam endpoint
4. mark the cells **along the beam** as free
5. mark the **last cell** at the obstacle location as occupied

Repeat this continuously while the robot moves.

Over time, the map becomes more complete.

---

## Coordinate Frames

You will work mainly with two coordinate systems:

### 1. World coordinates

These are continuous coordinates in meters:

* `x`
* `y`

### 2. Grid coordinates

These are integer cell indices:

* `i`
* `j`

You need a conversion between the two.

If the map origin is `(x_min, y_min)`, then:

```text
j = floor((x - x_min) / resolution)
i = floor((y - y_min) / resolution)
```

Here:

* `j` is the column index
* `i` is the row index

---

## Robot Pose from Odometry

From `/odom`, extract:

* robot position: `x_r`, `y_r`
* robot orientation: yaw angle `theta`

The yaw angle is obtained from the quaternion in the odometry message.

---

## LiDAR Beam Model

From `LaserScan`, each beam has:

* range `r`
* angle relative to robot frame

If the beam angle in robot frame is `phi`, then the beam angle in the world frame is:

```text
global_angle = theta + phi
```

The endpoint of the beam is:

```text
x_end = x_r + r * cos(global_angle)
y_end = y_r + r * sin(global_angle)
```

---

## How to Update the Grid

### Free cells

All cells between the robot and the measured obstacle are considered **free**.

### Occupied cell

The cell at the end of the beam is considered **occupied**, provided that:

* the range is valid
* the range is smaller than `range_max`

### Unknown cells

Cells never observed remain **unknown**.

---

## Ray Tracing

To mark all cells between the robot position and the beam endpoint, use a simple line traversal method.

The most common choice is:

* **Bresenham line algorithm**

This gives all grid cells crossed by the beam.

Then:

* all intermediate cells → free
* final cell → occupied

---

## Simple Version of the Exercise

A basic implementation can use direct assignment:

* unknown = `-1`
* free = `0`
* occupied = `100`

This is easy to implement, but it has one limitation:
repeated measurements do not accumulate confidence.

---

## Better Version: Probabilistic Update

A better map stores an internal probability or log-odds value for each cell.

Example idea:

* if a beam passes through a cell → decrease occupancy belief
* if a beam ends in a cell → increase occupancy belief

Then convert to ROS occupancy values when publishing.

For example:

* high probability of obstacle → `100`
* low probability of obstacle → `0`
* uncertain value → `-1` or intermediate value

---

## Suggested Internal Representation

Use an internal 2D array of floating-point values.

Two options:

### Option 1: probability

Each cell stores a value between `0` and `1`.

### Option 2: log-odds

Each cell stores a log-odds score.

This is usually more convenient because updates are additive.

Example:

```text
L(cell) = L(cell) + L_occ   if occupied
L(cell) = L(cell) + L_free  if free
```

where:

* `L_occ` is positive
* `L_free` is negative

Then clamp values to avoid very large magnitude.

---

## Publishing the Map

Your node must publish a `nav_msgs/msg/OccupancyGrid` message.

Important fields include:

* header
* info.resolution
* info.width
* info.height
* info.origin
* data

The `data` field is a 1D array, so your 2D grid must be flattened.

Typical flattening:

```text
index = i * width + j
```

---

## Suggested Node Structure

Your ROS 2 node may contain:

* subscriber to `/scan`
* subscriber to `/odom`
* publisher to `/map`
* map array
* latest robot pose

Typical logic:

1. receive odometry and store latest pose
2. receive LiDAR scan
3. process each valid beam
4. update the internal grid
5. publish updated occupancy grid

---

## Recommended Simplifications

To keep the exercise manageable, use the following assumptions:

* 2D map only
* robot moves on a flat floor
* odometry is accurate enough
* no SLAM required
* LiDAR is mounted at the robot center
* ignore dynamic obstacles

---

## Parameters Students Should Define

Students should choose and justify:

* map resolution
* map width and height
* map origin
* update rate
* occupied threshold
* free threshold
* valid LiDAR range limits

Example values:

```text
resolution = 0.05 m/cell
width = 200
height = 200
origin = (-5.0, -5.0)
```

---

## Expected Result

When visualized in RViz, the map should gradually show:

* walls as occupied regions
* open floor as free regions
* unexplored parts as unknown

As the robot moves, more of the environment should become visible.

---

## Deliverables

Each student or group should submit:

1. ROS 2 package
2. mapping node source code
3. short report
4. screenshot of the final map in RViz
5. short video or demo of the mapping process

---

## Suggested Tasks

### Part A - Basic mapping

Implement an occupancy grid mapper using:

* `/scan`
* `/odom`
* direct free/occupied marking

### Part B - Probabilistic improvement

Improve the map by using:

* probability or log-odds updates
* repeated evidence accumulation

### Part C - Analysis

Comment on:

* map quality
* effect of resolution
* effect of odometry drift
* effect of LiDAR noise

---

## Bonus Ideas

Optional extensions:

* use Bresenham line tracing
* use log-odds instead of direct assignment
* publish map at fixed timer rate
* save the map to file
* compare two map resolutions
* inflate obstacles for safety margin

---

## Grading Suggestion

### Implementation - 40%

* node works correctly
* subscribes and publishes correctly
* map updates correctly

### Mapping quality - 25%

* obstacles appear correctly
* free space is identified correctly
* map is consistent

### Code quality - 15%

* clear structure
* comments
* readable implementation

### Report - 10%

* explanation of method
* discussion of assumptions

### Demo / visualization - 10%

* RViz result shown clearly

---

## Important Note

This exercise is **mapping with known pose**, not full SLAM.

That means:

* localization is assumed available from odometry
* the main focus is the map construction process

---

## Very Short Algorithm Summary

For every scan beam:

```text
get robot pose
compute beam endpoint
convert robot and endpoint to grid cells
mark cells along the ray as free
mark endpoint cell as occupied
publish map
```

---

## Suggested Next Step

After this exercise, a natural continuation would be:

* occupancy grid mapping with noisy pose
* scan matching
* SLAM
* integration with navigation
