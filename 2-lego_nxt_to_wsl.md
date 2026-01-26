# 🚀 Connecting LEGO NXT to WSL (AMR Image)

These instructions explain how to attach your LEGO Mindstorms NXT brick (USB) to a specific WSL distribution (e.g., `AMR`).

---

## 1. Install `usbipd-win` on Windows
Open **PowerShell (Admin)** and run:

```powershell
winget install usbipd
```

Check installation:

```powershell
usbipd --version
```

---

## 2. Plug in the LEGO NXT brick
After plugging in the device, list USB devices:

```powershell
usbipd list
```

Example output:

```
BUSID  VID:PID    DEVICE                              STATE
1-7    0694:0002  LEGO Group Mindstorms NXT           Not shared
1-5    1bcf:2ced  Hy-FHD(9807)-Camera                 Not shared
```

Note the **BUSID** of your LEGO NXT (e.g., `1-7`).

---

## 3. Bind and attach NXT to WSL (AMR)
Still in PowerShell (Admin):

```powershell
usbipd bind --busid <BUSID>
usbipd attach --wsl=Distribution --busid 1-10
```

Example (Make sure you have at least one WSL terminal Open):
```powershell
usbipd bind --busid 1-7
usbipd attach --wsl=AMR --busid 1-7  
```

---

## 4. Verify inside WSL
Open your AMR distribution:

```powershell
wsl -d AMR
```

Check USB devices:

```bash
lsusb
```

Expected output:
```
Bus 001 Device 004: ID 0694:0002 Lego Group Mindstorms NXT
```

---

## 5. Install Python support in WSL

1. **Verify that the LEGO brick is detected**
   ```bash
   lsusb
   ```
   You should see a line like:
   ```
   Bus 001 Device 002: ID 0694:0002 Lego Group Mindstorms NXT
   ```

2. **Create a udev rule to grant access**
   ```bash
   sudo nano /etc/udev/rules.d/99-lego-nxt.rules
   ```
   Add this line inside the file:
   ```
   SUBSYSTEM=="usb", ATTR{idVendor}=="0694", ATTR{idProduct}=="0002", MODE="0666"
   ```

3. **Reload the udev rules and trigger**
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. **Run your LEGO test script**
   - If `nxt` module is not found, install it:
     ```bash
     pip install nxt-python
     ```
   - Then run:
     ```bash
     python3 helloLego.py
     ```

You should now see your NXT brick connecting successfully.

