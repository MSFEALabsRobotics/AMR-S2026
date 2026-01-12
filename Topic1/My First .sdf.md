# First SDF World with a Box (Minimal)

## 1️⃣ Create a New File
```bash
nano box.sdf
```

Paste this content:
```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="default">
    <model name="box_model">
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

---

## 2️⃣ Run in Gazebo Sim
```bash
gz sim box.sdf
```

---

## 3️⃣ In a New Terminal – List Topics
```bash
gz topic -l
```

---

## 4️⃣ Echo the Clock Topic
```bash
gz topic -e --topic /clock
```

---

## 5️⃣ Stop the Simulation
Press **Ctrl + C** to exit.
