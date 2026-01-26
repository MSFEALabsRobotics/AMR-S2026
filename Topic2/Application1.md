<img width="762" height="563" alt="image" src="https://github.com/user-attachments/assets/89d24c86-31ea-4222-83e7-8fa151d87134" />


# 🚗 Lab Task – Robot Driving Scenarios

## Instructions

Write Python ROS 2 nodes that drive the robot in **two scenarios**:

- **Scenario 1:**  
  Go straight for **10 seconds** at `0.2 m/s`, then turn for **10 seconds** with `Xdot = 0.3 m/s`, `ω = 0.2 rad/s`.

- **Scenario 2:**  
  Do the opposite: first turn (`Xdot = 0.3 m/s`, `ω = -0.2 rad/s`) for **10 seconds**, then go straight at `0.2 m/s` for **10 seconds`.

---

## Requirements

1. Publish velocity commands (`/cmd_vel`) using Python (`rclpy`).
2. Link your robot to **RViz** and show TF + odometry in real time.
3. At the end of each run, **print the final pose (x, y, θ)** from your code.
4. Enable **joint state readings**.  
   - Read the encoder of the **left wheel**.  
   - Compare values between Scenario 1 and Scenario 2.  
   - State what you conclude.

---

## Deliverables

- Python code for both scenarios.  
- Console output with **end poses**.  
- RViz screenshots.  
- Encoder comparison + short conclusion.
