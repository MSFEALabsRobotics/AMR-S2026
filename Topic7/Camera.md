# Gazebo Sim  Camera → ROS 2 → OpenCV 
## 1) Enable sensors in your world
Add this **plugin** inside your `<world>`:
```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
```

## 2) Add a forward-looking camera to the robot (inside the `chassis` link)
```xml
<sensor name="front_cam" type="camera">
  <!-- x   y   z    R   P      Y -->
  <pose>-0.2  0  0.35  0  -0.10  3.14159</pose>  <!-- yaw=π flips it forward; slight down pitch -->
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.05</horizontal_fov>
    <image><width>640</width><height>480</height><format>R8G8B8</format></image>
  </camera>
  <always_on>true</always_on>
</sensor>
```

## 3) Bridge camera topics to ROS 2
Keep this running in a terminal (adjust paths if different):
```bash
ros2 run ros_gz_bridge parameter_bridge   /world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/image@sensor_msgs/msg/Image@gz.msgs.Image   /world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo   /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```

## 4) Install OpenCV + cv_bridge (once)
```bash
sudo apt install ros-jazzy-cv-bridge python3-opencv
```

## 5) (Optional) Make the robot drive in a circle (in Gazebo)
```bash
gz topic -t /cmd_vel -m gz.msgs.Twist -p "angular: {z: 0.3}"
```

## 6) Gazebo GUI check
Add **Image Display** plugin (top-right “+” → *Image Display*) and select your camera topic (similar to Lidar visualization plugin).

## 7) Minimal ROS 2 script to show the image (auto-discovers topic)
Save as `camera_cv2_autoview.py` and run with `python3 camera_cv2_autoview.py`:
```python
# camera_cv2_autoview.py (ROS 2 Jazzy)
import rclpy, cv2, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class AutoCam(Node):
    def __init__(self):
        super().__init__('camera_cv2_autoview')
        self.bridge = CvBridge()
        self.sub = None
        self.topic = None
        self.timer = self.create_timer(0.5, self.try_subscribe)  # try until a camera topic exists
        cv2.namedWindow("camera", cv2.WINDOW_AUTOSIZE)

    def try_subscribe(self):
        if self.sub:  # already subscribed
            return
        # Pick the first Image topic (prefer one ending with '/image')
        candidates = []
        for name, types in self.get_topic_names_and_types():
            if 'sensor_msgs/msg/Image' in types:
                candidates.append(name)
        if not candidates:
            self.get_logger().info("No sensor_msgs/Image topics yet...")
            return
        # Prefer topics ending with '/image', else take the first
        candidates.sort(key=lambda n: (not n.endswith('/image'), len(n)))
        self.topic = candidates[0]
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(Image, self.topic, self.cb, qos)
        self.get_logger().info(f"Subscribed to {self.topic}")

    def cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        ## ADD CUSTOM CV2 CODE HERE ########################
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AutoCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Add Canny edges (optional)
Put this in the “ADD CUSTOM CV2 CODE HERE” section to visualize edges:
```python
# --- Canny basics ---
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5, 5), 1.2)
edges = cv2.Canny(blur, 30, 90)  # (lower, upper) thresholds

# Show edges in a second window
cv2.imshow("edges", edges)
```

## 8) Quick verification
```bash
ros2 topic hz /world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/image
ros2 run rqt_image_view rqt_image_view
```
You should see a steady Hz and the image in `rqt_image_view`. If not, check your bridge topic names and the SDF camera `pose`/`update_rate/always_on`.
