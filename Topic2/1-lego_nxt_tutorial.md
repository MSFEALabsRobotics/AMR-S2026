# LEGO NXT Robot Tutorial (Python)

This guide shows you how to set up and program your LEGO Mindstorms NXT robot using Python.

---

## 1. Install the Required Library

First, install the Python library for NXT:

```bash
pip install nxt-python
```

---

## 2. Install USB Driver (Windows)

The NXT brick requires a **libusb** driver to work with Python.  

1. Download Zadig: [https://zadig.akeo.ie/](https://zadig.akeo.ie/)  
2. Plug in your NXT via **USB**.  
3. Open **Zadig → Options → check “List All Devices”**.  
4. In the dropdown, select **LEGO NXT** (sometimes shows as *Unknown device* with LEGO vendor ID `0x0694`).  
5. Replace its driver with **libusb-win32** or **libusbK**.  
6. Click **Install Driver**.  

---

## 3. Hello World Program

Here’s a simple Python script to make your NXT motors run.

```python
import time
import nxt.locator
from nxt import motor

# Connect to the NXT brick
brick = nxt.locator.find()

# Motors on ports A and B (left and right wheels)
mL = brick.get_motor(motor.Port.A)
mR = brick.get_motor(motor.Port.B)

# Run motors at speed (forward and backward)
mL.run(power=100, regulated=True)
mR.run(power=-100, regulated=True)

# Keep running for 5 seconds
time.sleep(5)

# Stop the motors
mL.brake()
mR.brake()
```

✅ Make sure your motors are plugged into ports **A** and **B**.  

---

## 4. Notes

- `power` ranges from **-127 to +127** (negative = reverse).  
- `regulated=True` uses the NXT’s built-in control to maintain speed.  
- Use `brake()` to stop immediately, or `idle()` to let the motor coast.  

---

## 5. Next Steps

You can expand this by:  
- Controlling a third motor on port C.  
- Reading sensors (touch, ultrasonic, light).  
- Writing functions for driving straight, turning, etc.  

---

Now you’re ready to control your LEGO NXT robot with Python 🚀


---

## 6. Hello World: Reading Tacho Counts

This example shows how to read the **tacho counts** (motor rotation in degrees) from the NXT motors.

```python
import time
import nxt.locator
from nxt import motor

def get_deg(m):
    """
    Return motor angle in degrees.
    Works with both 'rotation_count' and 'tacho_count'.
    """
    t = m.get_tacho()
    return getattr(t, "rotation_count", getattr(t, "tacho_count"))

def main():
    # Connect to NXT
    brick = nxt.locator.find()
    name, *_ = brick.get_device_info()
    try:
        mv = brick.get_battery_level()
        print(f"Connected to: {name} | Battery: {mv/1000:.2f} V")
    except Exception:
        print(f"Connected to: {name}")

    # Motors on A (left) and B (right)
    mL = brick.get_motor(motor.Port.A)
    mR = brick.get_motor(motor.Port.B)

    # Start both motors so tachos change
    power = 70  # try 70–100 if your robot needs more torque
    mL.run(power)
    mR.run(power)

    # Stream tachos for 3 seconds
    print("Streaming tachos for ~3 s (Ctrl+C to stop):")
    dL_prev = get_deg(mL)
    dR_prev = get_deg(mR)
    t_prev = time.time()
    end_time = t_prev + 3.0

    try:
        while time.time() < end_time:
            time.sleep(0.1)
            t_now = time.time()
            dL = get_deg(mL)
            dR = get_deg(mR)
            dt = t_now - t_prev
            vL = (dL - dL_prev) / dt  # deg/s
            vR = (dR - dR_prev) / dt  # deg/s
            print(f"L: {dL:8.1f}° ({vL:6.1f}°/s)  |  R: {dR:8.1f}° ({vR:6.1f}°/s)")
            dL_prev, dR_prev, t_prev = dL, dR, t_now
    finally:
        mL.brake()
        mR.brake()
        print("Stopped.")

if __name__ == "__main__":
    main()
```

This program:
- Connects to the NXT brick  
- Runs motors A and B  
- Continuously reads the **tacho counts** (angles)  
- Prints both **absolute angle** and **velocity in deg/s**  
- Stops the motors after 3 seconds  

