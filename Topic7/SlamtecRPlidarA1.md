# RPLIDAR A1 – Quick Start (.md Tutorial)

This guide walks you through installing RoboStudio, drivers, and running a Python visualizer for the **Slamtec RPLIDAR A1** on Windows.


---

## 1) Download & Install RoboStudio

1. Go to **RoboStudio** page: <https://www.slamtec.com/en/ROBOSTUDIO>  
2. Click **Download** and install.
3. Launch RoboStudio, create an account, and **register your email** to sign in.

> RoboStudio is useful to quickly verify that your LIDAR is working before coding.


---

## 2) Install the USB–UART Driver (CP210x)

The A1 uses a USB‑to‑UART bridge (often **CP210x**). Install the **CP210x Windows driver** (available from Silicon Labs / mirrors on GitHub).

- After installation, plug the RPLIDAR A1 and check **Device Manager → Ports (COM & LPT)**. Note the **COM#** (e.g., `COM5`).


---

## 3) Connect and Test with RoboStudio

1. Plug in the **RPLIDAR A1** via USB.
2. In RoboStudio, select the detected serial device (your **COM#**).
3. Click **Connect / Start Scan**. You should see live scan data.

> If you don’t see scans: try another USB port, disable power saving on USB hubs, and confirm the COM# in Device Manager.


---

## 4) Python Setup (Windows)

### 4.2 Install the RPLIDAR library
```powershell
pip install rplidar-roboticia
# Project page: https://github.com/Roboticia/RPLidar
```
---

## 5) Python Example – Polar Scatter Animation

The script below opens the RPLIDAR on your **COM port** (update `PORT_NAME`) and shows a live polar scatter of distances with measurement intensity.

```python
#pip install rplidar-roboticia
#https://github.com/Roboticia/RPLidar

#!/usr/bin/env python3
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

PORT_NAME = 'COM5'
DMAX = 4000
IMIN = 0
IMAX = 50

def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def run():
    lidar = RPLidar(PORT_NAME)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                           cmap=plt.cm.Greys_r, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)

    iterator = lidar.iter_scans()
    ani = animation.FuncAnimation(fig, update_line,
        fargs=(iterator, line), interval=50)
    plt.show()
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    run()
```

### Run it
```powershell
python rplidar_polar_live.py
```

