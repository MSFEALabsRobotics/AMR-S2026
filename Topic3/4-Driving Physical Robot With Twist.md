# LEGO NXT — ROS 2 `/cmd_vel` Subscriber (Control at Robot CoG)

## Goal
Write a ROS 2 Python node that **subscribes to `/cmd_vel` (Twist)** and drives a **LEGO NXT differential-drive** robot by interpreting `linear.x` and `angular.z` as velocities **at the robot center of gravity (CoG / base center)**, then converting them to left/right wheel commands.

---

<img width="1946" height="564" alt="image" src="https://github.com/user-attachments/assets/086191ad-7c42-4706-9826-5d7e63ba6138" />
