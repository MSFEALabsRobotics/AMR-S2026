
<img width="856" height="638" alt="image" src="https://github.com/user-attachments/assets/983259f7-7cc7-4323-b362-d33c032a5c5a" />

# 🧩 Gazebo Maze Model + Open Loop Motion Planning

This tutorial shows how to **add a maze model** next to your robot in Gazebo, and perform a simple **open loop motion planning** strategy to exit the maze.

---

## 1. Add Maze Model to Your World

Insert the following `<model>` block inside your Gazebo world `.sdf` file (next to your robot model):

```xml
<model name="maze_model">
    <static>true</static>

    <!-- Wall 1 -->
    <link name="wall1">
        <pose>0 -5 0.5 0 0 0</pose>
        <collision name="collision">
            <geometry><box><size>10 0.2 1</size></box></geometry>
        </collision>
        <visual name="visual">
            <geometry><box><size>10 0.2 1</size></box></geometry>
            <material>
                <diffuse>0.8 0.2 0.2 1</diffuse>
                <ambient>0.8 0.2 0.2 1</ambient>
            </material>
        </visual>
    </link>

    <!-- Wall 2 -->
    <link name="wall2">
        <pose>-1 5 0.5 0 0 0</pose>
        <collision name="collision">
            <geometry><box><size>8 0.2 1</size></box></geometry>
        </collision>
        <visual name="visual">
            <geometry><box><size>8 0.2 1</size></box></geometry>
            <material>
                <diffuse>0.8 0.2 0.2 1</diffuse>
                <ambient>0.8 0.2 0.2 1</ambient>
            </material>
        </visual>
    </link>

    <!-- Wall 3 -->
    <link name="wall3">
        <pose>-5 0 0.5 0 0 0</pose>
        <collision name="collision">
            <geometry><box><size>0.2 10 1</size></box></geometry>
        </collision>
        <visual name="visual">
            <geometry><box><size>0.2 10 1</size></box></geometry>
            <material>
                <diffuse>0.2 0.2 0.8 1</diffuse>
                <ambient>0.2 0.2 0.8 1</ambient>
            </material>
        </visual>
    </link>

    <!-- Wall 4 -->
    <link name="wall4">
        <pose>5 0 0.5 0 0 0</pose>
        <collision name="collision">
            <geometry><box><size>0.2 10 1</size></box></geometry>
        </collision>
        <visual name="visual">
            <geometry><box><size>0.2 10 1</size></box></geometry>
            <material>
                <diffuse>0.2 0.2 0.8 1</diffuse>
                <ambient>0.2 0.2 0.8 1</ambient>
            </material>
        </visual>
    </link>

    <!-- Obstacle 1 -->
    <link name="obstacle1">
        <pose>-2 0 0.5 0 0 0</pose>
        <collision name="collision">
            <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <visual name="visual">
            <geometry><box><size>1 1 1</size></box></geometry>
            <material>
                <diffuse>0.2 0.8 0.2 1</diffuse>
                <ambient>0.2 0.8 0.2 1</ambient>
            </material>
        </visual>
    </link>

    <!-- Obstacle 2 -->
    <link name="obstacle2">
        <pose>2 1.5 0.5 0 0 0</pose>
        <collision name="collision">
            <geometry><box><size>5 0.5 1</size></box></geometry>
        </collision>
        <visual name="visual">
            <geometry><box><size>5 0.5 1</size></box></geometry>
            <material>
                <diffuse>0.2 0.8 0.2 1</diffuse>
                <ambient>0.2 0.8 0.2 1</ambient>
            </material>
        </visual>
    </link>
</model>
```

---

## 2. Run the World
Start your simulation:

```bash
gz sim my_world.sdf
```

You should now see your robot and the maze next to it.

---

## 3. Open Loop Motion Planning
Open loop means we **predefine the motion sequence** without using sensors or feedback.

Example ROS 2 Python node (`open_loop_maze.py`) to exit the maze:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


```

---

## 4. Run the Node
```bash
python3 open_loop_maze.py
```

The robot will execute the sequence and move through the maze.

---

✅ Now you have a **maze model** in Gazebo and a simple **open loop controller** to make the robot exit.
