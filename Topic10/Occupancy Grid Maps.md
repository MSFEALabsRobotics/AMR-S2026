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
