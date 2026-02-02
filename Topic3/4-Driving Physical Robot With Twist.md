# LEGO NXT — ROS 2 `/cmd_vel` Subscriber (Control at Robot CoG)

## Goal
# 1- Twist

Write a ROS 2 Python node that **subscribes to `/cmd_vel` (Twist)** and drives a **LEGO NXT differential-drive** robot by interpreting `linear.x` and `angular.z` as velocities **at the robot center of gravity (CoG / base center)**, then converting them to left/right wheel commands.

---

# 2- Odometry

Upgrade your script so that you read the tachometers of the robot (From Wheels) you convert them to the center of gravity and you publish the corresponding odometry messages, Show Results in RVIZ

---

<img width="1946" height="564" alt="image" src="https://github.com/user-attachments/assets/086191ad-7c42-4706-9826-5d7e63ba6138" />
