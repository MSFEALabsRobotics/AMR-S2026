# WSL Tutorial: Use `nano` to create a Python file, run it, then edit with VS Code

Goal:
1) Create a simple `hello.py` in WSL using **nano**
2) Run it with **python**
3) Install **VS Code on Windows**
4) From WSL, run `code .` to open the folder in VS Code and edit the file

---

## 1) Open WSL (Ubuntu)

Open nano:
```bash
nano hello.py
```

Type this code:
```python
print("Hello from WSL")
name = input("Your name? ")
print("Nice to meet you,", name)
```

### Save and exit nano
- **Save**: press `CTRL + O` → then press `Enter`
- **Exit**: press `CTRL + X`

Confirm the file exists:
```bash
ls
```

(Optional) View the file:
```bash
cat hello.py
```

---

## 4) Run the file in Python

Run:
```bash
python3 hello.py
```

You should see the print message, then it will ask for your name.

---

## 5) Install VS Code on Windows

1) Download and install **Visual Studio Code** from the official site.
2) During install, ensure the option is enabled (if offered):  
   ✅ **Add “Open with Code” / Add to PATH** (wording may vary)

---

## 6) Install the VS Code WSL extension

Open **VS Code** in Windows.

Go to Extensions (left sidebar) and install:
- **WSL** (by Microsoft)

This enables VS Code to work directly with your WSL files.

---

## 7) Use `code .` from WSL to open the folder in VS Code

Back in WSL, inside your folder (`~/py_test`), run:
```bash
code .
```

✅ VS Code should open, connected to **WSL**.

You can now:
- click `hello.py`
- edit the code
- save normally (CTRL+S)

---

## 8) Run the updated code from the VS Code terminal (WSL)

Inside VS Code:
- Terminal → New Terminal

Run:
```bash
python3 hello.py
```

---

## Common issues

### A) `code: command not found`
Install the WSL integration from VS Code:

1) Open VS Code (Windows)
2) Press `CTRL + SHIFT + P`
3) Search and run:
**“WSL: Install 'code' command in PATH”**
4) Close WSL terminal and open it again, then retry:
```bash
code .
```

### B) You edited files under `/mnt/c/...` and it feels slow
Keep dev projects in the Linux home folder, e.g.:
```bash
~/
```

---

Done ✅
