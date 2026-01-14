# DiffDrive Plugin (Gazebo Sim) — Add Inside Your `<model>`

Add this plugin **inside the `<model> ... </model>` block** (typically near the end of the model, after links/joints).

## 1) Paste the plugin

```xml
<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>1.2</wheel_separation>
  <wheel_radius>0.4</wheel_radius>
  <odom_publish_frequency>1</odom_publish_frequency>
  <topic>cmd_vel</topic>
</plugin>
```

## 2) Where it goes (example)

```xml
<model name="vehicle_blue" canonical_link="chassis">
  ...
  <!-- links + joints here -->

  <!-- DiffDrive plugin goes here -->
  <plugin
      filename="gz-sim-diff-drive-system"
      name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
  </plugin>
</model>
```

## 3) Quick notes

- `left_joint` / `right_joint` **must match your joint names** exactly.
- `wheel_separation` is the distance between the wheel contact centers (meters).
- `wheel_radius` must match your wheel radius (meters).
- Publish command velocity on the topic you set (here: `cmd_vel`).
