# 🚗 Differential Drive Kinematics – Python Tutorial

This tutorial shows how to implement and use a simple **differential drive kinematics class** in Python.  
We separate the code into two files:

<img width="364" height="329" alt="image" src="https://github.com/user-attachments/assets/1d8b0c25-e918-4829-91b1-2ddeb77d6230" />

---

## 1. Create the Class File

Create a file called **`diff_drive.py`**:

```python
# diff_drive.py

import math

class DiffDriveKinematics:
    def __init__(self, wheel_radius=0.05, wheel_base=0.2):
        self.r = wheel_radius   # wheel radius [m]
        self.L = wheel_base     # distance between wheels [m]

        # Robot pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0   # [rad]

    def forward(self, w_r, w_l):
        """
        Forward kinematics:
        From wheel angular speeds (rad/s) -> robot (v, omega)
        """
        v_r = self.r * w_r
        v_l = self.r * w_l
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.L
        return v, omega

    def inverse(self, v, omega):
        """
        Inverse kinematics:
        From robot (v, omega) -> wheel angular speeds (rad/s)
        """
        v_r = v + (self.L / 2.0) * omega
        v_l = v - (self.L / 2.0) * omega
        w_r = v_r / self.r
        w_l = v_l / self.r
        return w_r, w_l

    def update_pose(self, w_r, w_l, dt):
        """
        Update the robot pose by integrating velocity over time.
        """
        v, omega = self.forward(w_r, w_l)

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        return self.x, self.y, self.theta
```

---

## 2. Use the Class in Another Script

Create a second file called **`test_diff.py`** in the same folder:

```python
# test_diff.py
from diff_drive import DiffDriveKinematics

# Create a robot model
robot = DiffDriveKinematics(wheel_radius=0.05, wheel_base=0.2)

# Example 1: Forward kinematics
v, omega = robot.forward(w_r=10, w_l=10)
print("Forward Kinematics:")
print(f"  v = {v:.2f} m/s, omega = {omega:.2f} rad/s")

# Example 2: Inverse kinematics
w_r, w_l = robot.inverse(v=0.5, omega=0.5)
print("\nInverse Kinematics:")
print(f"  w_r = {w_r:.2f} rad/s, w_l = {w_l:.2f} rad/s")
```

---

## ✅ Summary

- **Forward kinematics**: wheel speeds → \((v, \omega)\).  
- **Inverse kinematics**: \((v, \omega)\) → wheel speeds.  
