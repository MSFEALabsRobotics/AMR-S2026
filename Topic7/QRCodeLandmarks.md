<img width="770" height="596" alt="image" src="https://github.com/user-attachments/assets/4b809d8b-6737-4932-96e4-3d06b7805a02" />


# QR Code Detection and Gazebo QR Tutorial

## 1) Detect a QR code in Python using OpenCV (`cv2`)

### Code

```python
import cv2

# load image
img = cv2.imread("qr_sample.png")

# create detector
detector = cv2.QRCodeDetector()

# detect and decode
data, points, _ = detector.detectAndDecode(img)

if points is not None and data:
    print("QR detected")
    print("Decoded data:", data)

    # draw box around QR
    points = points[0].astype(int)
    for i in range(len(points)):
        pt1 = tuple(points[i])
        pt2 = tuple(points[(i + 1) % len(points)])
        cv2.line(img, pt1, pt2, (0, 255, 0), 2)

    cv2.imshow("QR Detection", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No QR code found")
```

### Install

```bash
pip install opencv-python
```

### Notes

* Put `qr_sample.png` in the same folder as the script, or use a full path.
* `data` = decoded text inside the QR code.
* `points` = 4 corners of the QR code.

---

## 2) Add a QR code to a wall in Gazebo

### Put the QR image here

```bash
/home/test/QRSample.png
```

### Add this model

```xml
<model name="maze_model">
  <static>true</static>

  <link name="wall1">
    <pose>0 -5 0.5 0 0 0</pose>

    <collision name="collision">
      <geometry>
        <box>
          <size>10 0.2 1</size>
        </box>
      </geometry>
    </collision>

    <!-- red wall -->
    <visual name="wall_visual">
      <geometry>
        <box>
          <size>10 0.2 1</size>
        </box>
      </geometry>
      <material>
        <diffuse>0.8 0.2 0.2 1</diffuse>
        <ambient>0.8 0.2 0.2 1</ambient>
      </material>
    </visual>

    <!-- QR code panel -->
    <visual name="qr_visual">
      <pose>0 0.101 0 0 0 0</pose>
      <geometry>
        <box>
          <size>1 0.01 1</size>
        </box>
      </geometry>
      <material>
        <ambient>1 1 1 1</ambient>
        <diffuse>1 1 1 1</diffuse>
        <pbr>
          <metal>
            <albedo_map>file:///home/test/QRSample.png</albedo_map>
            <roughness>1.0</roughness>
            <metalness>0.0</metalness>
          </metal>
        </pbr>
      </material>
    </visual>
  </link>
</model>
```

### Notes

* `qr_visual` is a thin panel placed slightly in front of the wall.
* `albedo_map` loads the QR image.
* Use an absolute path.

### Move the QR on the wall

```xml
<pose>0 0.101 0 0 0 0</pose>
```

* first value: left/right
* second value: distance in front of wall
* third value: up/down

Examples:

```xml
<pose>-2 0.101 0 0 0 0</pose>
```

left

```xml
<pose>2 0.101 0 0 0 0</pose>
```

right

```xml
<pose>0 0.101 0.2 0 0 0</pose>
```

higher

---

## 3) Add a camera and make the robot react to the QR code

### Goal

1. robot rotates right
2. camera checks for QR
3. when QR is detected, switch behavior
4. robot moves left

### You need

* a camera sensor on the robot
* a ROS 2 node subscribing to the camera image
* OpenCV QR detection inside the node
* velocity commands published to `/cmd_vel`

### ROS topics

* subscribe: `/camera/image_raw`
* publish: `/cmd_vel`

### Internal state

```python
self.qr_found = False
```

### Pseudocode

```text
start node
set qr_found = False

when image arrives:
    run QR detection
    if QR detected:
        qr_found = True

in control loop:
    if qr_found is False:
        rotate right
    else:
        move left
```

### Exercise note

Do not implement the full solution here.
This part is a good ROS 2 exercise combining:

* camera sensor
* image topic
* cv_bridge
* OpenCV
* robot motion control
